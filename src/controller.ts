import * as THREE from 'three';
import * as RAPIER from '@dimforge/rapier3d-compat';
import { Rig, PhysBone, RAY_GROUPS } from './rig';
import { axisAngleShortest, integrateQuatLocal, clampVecNorm, wrapAngle, clamp } from './math';

const MAX_SPEED = 2.4;
const KP_HOVER = 9000, KD_HOVER = 700;
const YAW_RATE = 4.0, YAW_RATE_MAX = 7.0;
const KV_DRIVE = 7;
// Hairibar's spring parameterization: k = I * alpha / dt^2 (alpha = fraction of
// error corrected per step, framerate- and inertia-independent), d = ratio *
// critical. Their "walking tight" profile ships alpha 0.3, damping 1.0. Sanity
// check: thigh inertia ~0.47 kg·m² at 60 Hz gives k ≈ 507 — DeepMimic's
// hand-tuned hip kp is 500.
export let ROT_ALPHA = 0.3;
const DAMP_RATIO = 0.7;
const REF_DT = 1 / 60; // alpha is defined per animation frame, not per substep
export const setGainScale = (v: number) => { ROT_ALPHA = v; };
const RECOVER_TIME = 0.6; // s, master-alpha ramp after un-ragdolling (Hairibar t^2 easing)

// sergioabreu DefaultBehaviour: limbs go limp in the air
const AIR_SCALE: Record<string, number> = { leg: 0.05, head: 0.1, arm: 1, trunk: 1 };

const tmpQ1 = new THREE.Quaternion();
const tmpQ2 = new THREE.Quaternion();
const tmpQ3 = new THREE.Quaternion();
const tmpV1 = new THREE.Vector3();
const tmpV2 = new THREE.Vector3();
const tmpV3 = new THREE.Vector3();
const tmpV4 = new THREE.Vector3();

export class Controller {
  world: RAPIER.World;
  rig: Rig;
  ragdoll = false;
  grounded = false;
  yawTarget = 0;
  smoothSpeed = 0;
  masterAlpha = 1;
  private recoverT = 1; // ramps 0 -> 1 after recovery
  private hoverTarget: number;
  private fwdLocal = new THREE.Vector3(); // character-forward in the hips body frame
  private upLocal = new THREE.Vector3();  // character-up in the hips body frame

  constructor(world: RAPIER.World, rig: Rig) {
    this.world = world;
    this.rig = rig;
    this.hoverTarget = rig.restHipsHeight - 0.02;
    // The hips bone frame is not the character frame (Mixamo rigs point bone-Y
    // along the spine), so capture forward/up in the hips frame at build time,
    // when the character is known to face +Z and stand upright.
    const q0 = rig.hips.restQuat.clone().invert();
    this.fwdLocal.set(0, 0, 1).applyQuaternion(q0);
    this.upLocal.set(0, 1, 0).applyQuaternion(q0);
    // Stabilizer: sergioabreu's FREEZE_ROTATIONS balance mode — pitch/roll are
    // constraint-locked, yaw stays free (torque-PD on a light pelvis explodes).
    rig.hips.body.setEnabledRotations(false, true, false, true);
  }

  yaw(): number {
    const r = this.rig.hips.body.rotation();
    tmpV1.copy(this.fwdLocal).applyQuaternion(tmpQ1.set(r.x, r.y, r.z, r.w));
    return Math.atan2(tmpV1.x, tmpV1.z);
  }

  upY(): number {
    const r = this.rig.hips.body.rotation();
    return tmpV1.copy(this.upLocal).applyQuaternion(tmpQ1.set(r.x, r.y, r.z, r.w)).y;
  }

  setRagdoll(on: boolean) {
    this.ragdoll = on;
    const hips = this.rig.hips.body;
    if (on) {
      this.masterAlpha = 0;
      hips.setEnabledRotations(true, true, true, true);
    } else {
      const p = hips.translation();
      const yaw = this.yaw();
      const ray = new RAPIER.Ray({ x: p.x, y: p.y + 0.3, z: p.z }, { x: 0, y: -1, z: 0 });
      const hit = this.world.castRay(ray, 3.0, true, undefined, RAY_GROUPS);
      const groundY = hit ? p.y + 0.3 - hit.timeOfImpact : p.y - 0.7;
      this.rig.resetPose(p.x, p.z, yaw, groundY);
      hips.setEnabledRotations(false, true, false, true);
      this.yawTarget = yaw;
      this.recoverT = 0;
    }
  }

  respawn() {
    this.ragdoll = false;
    this.rig.resetPose(0, 0, 0, 0);
    this.rig.hips.body.setEnabledRotations(false, true, false, true);
    this.yawTarget = 0;
    this.recoverT = 0;
  }

  shove() {
    const a = Math.random() * Math.PI * 2;
    const m = this.rig.mass;
    this.rig.hips.body.applyImpulse({ x: Math.cos(a) * m * 2.0, y: m * 0.9, z: Math.sin(a) * m * 2.0 }, true);
    this.rig.bones.get('chest')!.body.applyImpulse({ x: Math.cos(a) * m * 0.8, y: 0, z: Math.sin(a) * m * 0.8 }, true);
  }

  // Stabilizer + locomotion forces. Run before sampling animation targets.
  prePhysics(dt: number, moveDir: { x: number; z: number }) {
    const hips = this.rig.hips.body;
    const p = hips.translation(), v = hips.linvel(), w = hips.angvel();

    for (const pb of this.rig.ordered) { pb.body.resetForces(true); pb.body.resetTorques(true); }

    if (p.y < -5) { this.respawn(); return; }

    if (this.recoverT < 1) {
      this.recoverT = Math.min(1, this.recoverT + dt / RECOVER_TIME);
      this.masterAlpha = this.recoverT * this.recoverT;
    }

    const ray = new RAPIER.Ray({ x: p.x, y: p.y, z: p.z }, { x: 0, y: -1, z: 0 });
    const hit = this.world.castRay(ray, 2.0, true, undefined, RAY_GROUPS);
    const dist = hit ? hit.timeOfImpact : Infinity;
    this.grounded = dist < this.hoverTarget + 0.35;

    const speedNow = Math.hypot(v.x, v.z);
    this.smoothSpeed += (speedNow - this.smoothSpeed) * Math.min(1, dt * 8);

    if (this.ragdoll) return;

    const vdx = moveDir.x * MAX_SPEED, vdz = moveDir.z * MAX_SPEED;
    const moving = Math.hypot(moveDir.x, moveDir.z) > 0.05;

    if (moving) this.yawTarget = Math.atan2(vdx, vdz);
    const yawErr = wrapAngle(this.yawTarget - this.yaw());
    hips.setAngvel({ x: w.x, y: clamp(YAW_RATE * yawErr, -YAW_RATE_MAX, YAW_RATE_MAX), z: w.z }, true);

    if (this.grounded) {
      const fy = clamp((this.hoverTarget - dist) * KP_HOVER - v.y * KD_HOVER, -700, 2800);
      const fx = clamp((vdx - v.x) * KV_DRIVE * this.rig.mass, -450, 450);
      const fz = clamp((vdz - v.z) * KV_DRIVE * this.rig.mass, -450, 450);
      hips.addForce({ x: fx, y: fy, z: fz }, true);
    }
  }

  // Quaternion-space PD toward the animated pose. DeepMimic's explicit PD with
  // the cheap Stable-PD prediction (error measured on q + dt*w), Hairibar's
  // inertia-scaled springs and feed-forward target angular velocity,
  // equal-and-opposite torques.
  applyTracking(dt: number) {
    if (this.masterAlpha <= 0) return;
    for (const pb of this.rig.ordered) {
      if (!pb.parent) continue;
      const alpha = ROT_ALPHA * this.masterAlpha * (this.grounded ? 1 : AIR_SCALE[pb.def.limb]);
      this.applyJointPD(pb, dt, Math.min(alpha, 0.9));
    }
  }

  private applyJointPD(pb: PhysBone, dt: number, alpha: number) {
    const rp = pb.parent!.body.rotation();
    const rc = pb.body.rotation();
    const qP = tmpQ1.set(rp.x, rp.y, rp.z, rp.w);
    const qC = tmpQ2.set(rc.x, rc.y, rc.z, rc.w);
    const qRel = tmpQ3.copy(qP).invert().multiply(qC); // child relative to parent

    const wc = pb.body.angvel(), wp = pb.parent!.body.angvel();
    const wRelChild = tmpV1.set(wc.x - wp.x, wc.y - wp.y, wc.z - wp.z)
      .applyQuaternion(tmpQ1.copy(qC).invert()); // qP no longer needed

    // predicted relative pose (SPD position term)
    const qPred = integrateQuatLocal(qRel, wRelChild, dt, tmpQ1);
    const qErr = qPred.invert().multiply(pb.qTar);
    const theta = axisAngleShortest(qErr, tmpV2);
    pb.lastErr = theta;
    const err = tmpV2.multiplyScalar(theta); // rotation-vector error, child frame

    // feed-forward target angular velocity: parent frame -> child frame
    const wErr = tmpV3.copy(pb.wTar).applyQuaternion(tmpQ1.copy(qRel).invert())
      .sub(wRelChild);

    // per-axis gains: split into twist (capsule axis) and swing components so
    // the low-inertia twist DOF isn't driven above the sampling limit.
    // `spd` is DeepMimic's implicit-damping correction reduced to a scalar:
    // tau *= I/(I + dt*kd). Without it kd*dt/I = 2*ratio*sqrt(alpha) can exceed
    // 1 and explicitly-sampled damping pumps energy instead of removing it.
    // The torque acts on BOTH bodies, so the relative DOF sees the reduced
    // inertia, not the child's — gains must scale with that or mid-chain
    // joints (forearm-vs-upperarm) run ~2x hot and the chain oscillates.
    // root (hips) is rotation-locked: treat as infinite inertia. Test by rig
    // topology, not by iPerp value — the builder floors iPerp to an epsilon.
    const iP = pb.parent!.def.parent === null ? Infinity : pb.parent!.iPerp;
    const redPerp = iP === Infinity ? pb.iPerp : (pb.iPerp * iP) / (pb.iPerp + iP);
    const redTwist = pb.iTwist * (redPerp / pb.iPerp);
    // Stiffness is anchored to the 60 Hz animation frame, NOT the physics
    // substep — alpha/dt^2 at 180 Hz is 9x stiffer and rams every large joint
    // into its DeepMimic torque clamp (saturated PD = bang-bang = drift).
    // Sanity: thigh I_red 0.48 gives k ≈ 518; DeepMimic's hand-tuned hip kp is 500.
    const kFac = alpha / (REF_DT * REF_DT);
    const dFac = 2 * DAMP_RATIO * Math.sqrt(alpha) / REF_DT;
    const spd = 1 / (1 + dt * dFac);
    const a = pb.axisLocal;
    const errTw = err.dot(a), wErrTw = wErr.dot(a);
    const tau = tmpV4
      .copy(err).addScaledVector(a, -errTw).multiplyScalar(kFac * redPerp)       // swing spring
      .addScaledVector(a, kFac * redTwist * errTw);                              // twist spring
    wErr.addScaledVector(a, -wErrTw);
    tau.addScaledVector(wErr, dFac * redPerp)                                    // swing damping
      .addScaledVector(a, dFac * redTwist * wErrTw);                             // twist damping
    tau.multiplyScalar(spd);
    clampVecNorm(tau, pb.def.clamp);
    tau.applyQuaternion(qC); // child local -> world

    pb.body.addTorque(tau, true);
    pb.parent!.body.addTorque({ x: -tau.x, y: -tau.y, z: -tau.z }, true);
  }

  meanErrDeg(): number {
    let sum = 0, n = 0;
    for (const pb of this.rig.ordered) if (pb.parent) { sum += pb.lastErr; n++; }
    return (sum / Math.max(1, n)) * (180 / Math.PI);
  }
}
