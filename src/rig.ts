import * as THREE from 'three';
import * as RAPIER from '@dimforge/rapier3d-compat';
import { axisAngleShortest } from './math';

const G = (memberships: number, filter: number) => ((memberships << 16) | filter) >>> 0;
export const GROUP_WORLD = 0x0001;
export const GROUP_BODY = 0x0002;
export const BODY_GROUPS = G(GROUP_BODY, GROUP_WORLD); // body parts never collide with each other
export const WORLD_GROUPS = G(GROUP_WORLD, 0xffff);
export const RAY_GROUPS = G(0xffff, GROUP_WORLD); // hover ray only sees the world

// Per-joint PD gains follow DeepMimic's humanoid3d_ctrl.txt (kd = kp/10, clamp =
// that character's torque limits); masses follow sergioabreu's ActiveRagdoll
// prefab (~69 kg total). Both were tuned for a human-sized ~45-70 kg biped.
interface BoneDef {
  key: string;
  bone: string;
  parent: string | null; // key of physics parent
  toward: string | null; // bone the capsule extends toward (null => ball)
  radius: number;
  mass: number;
  kp: number;      // Nm/rad (before global scale; kd = kp/10)
  clamp: number;   // Nm, vector-norm torque limit
  friction: number;
  limb: 'leg' | 'head' | 'arm' | 'trunk';
}

const M = (n: string) => `mixamorig:${n}`;

// GLTFLoader sanitizes node names (strips PropertyBinding-reserved chars like
// ':'), so "mixamorig:Hips" may arrive as "mixamorigHips" — accept both.
export function findBone(model: THREE.Object3D, name: string): THREE.Bone {
  const bone = (model.getObjectByName(name) ?? model.getObjectByName(name.replace(':', ''))) as THREE.Bone | undefined;
  if (!bone) throw new Error(`bone not found: ${name}`);
  return bone;
}

export const BONE_DEFS: BoneDef[] = [
  { key: 'hips',   bone: M('Hips'),         parent: null,    toward: null,              radius: 0.10, mass: 17,  kp: 0,    clamp: 0,   friction: 0.4, limb: 'trunk' },
  { key: 'chest',  bone: M('Spine1'),       parent: 'hips',  toward: M('Neck'),         radius: 0.12, mass: 14,  kp: 1000, clamp: 1000, friction: 0.4, limb: 'trunk' },
  { key: 'head',   bone: M('Head'),         parent: 'chest', toward: M('HeadTop_End'),  radius: 0.09, mass: 4.5, kp: 100,  clamp: 500,  friction: 0.4, limb: 'head' },
  { key: 'thighL', bone: M('LeftUpLeg'),    parent: 'hips',  toward: M('LeftLeg'),      radius: 0.07, mass: 7,   kp: 500,  clamp: 1000, friction: 0.4, limb: 'leg' },
  { key: 'shinL',  bone: M('LeftLeg'),      parent: 'thighL', toward: M('LeftFoot'),    radius: 0.055, mass: 3.3, kp: 500, clamp: 500, friction: 0.6, limb: 'leg' },
  { key: 'footL',  bone: M('LeftFoot'),     parent: 'shinL', toward: M('LeftToeBase'),  radius: 0.04, mass: 1,   kp: 400,  clamp: 500,  friction: 0.9, limb: 'leg' },
  { key: 'thighR', bone: M('RightUpLeg'),   parent: 'hips',  toward: M('RightLeg'),     radius: 0.07, mass: 7,   kp: 500,  clamp: 1000, friction: 0.4, limb: 'leg' },
  { key: 'shinR',  bone: M('RightLeg'),     parent: 'thighR', toward: M('RightFoot'),   radius: 0.055, mass: 3.3, kp: 500, clamp: 500, friction: 0.6, limb: 'leg' },
  { key: 'footR',  bone: M('RightFoot'),    parent: 'shinR', toward: M('RightToeBase'), radius: 0.04, mass: 1,   kp: 400,  clamp: 500,  friction: 0.9, limb: 'leg' },
  { key: 'uarmL',  bone: M('LeftArm'),      parent: 'chest', toward: M('LeftForeArm'),  radius: 0.045, mass: 3,  kp: 400,  clamp: 1000, friction: 0.4, limb: 'arm' },
  { key: 'farmL',  bone: M('LeftForeArm'),  parent: 'uarmL', toward: M('LeftHand'),     radius: 0.04, mass: 2,   kp: 300,  clamp: 500,  friction: 0.4, limb: 'arm' },
  { key: 'uarmR',  bone: M('RightArm'),     parent: 'chest', toward: M('RightForeArm'), radius: 0.045, mass: 3,  kp: 400,  clamp: 1000, friction: 0.4, limb: 'arm' },
  { key: 'farmR',  bone: M('RightForeArm'), parent: 'uarmR', toward: M('RightHand'),    radius: 0.04, mass: 2,   kp: 300,  clamp: 500,  friction: 0.4, limb: 'arm' },
];

export interface PhysBone {
  def: BoneDef;
  bone: THREE.Bone;
  parent: PhysBone | null;
  body: RAPIER.RigidBody;
  // animation-matching targets (child rotation relative to physics parent)
  qTar: THREE.Quaternion;
  qTarPrev: THREE.Quaternion;
  wTar: THREE.Vector3; // finite-differenced target angular velocity, parent frame
  hasTarget: boolean;
  // inertia about the joint pivot, split by axis: explicit PD gains must scale
  // with inertia per axis or low-inertia DOFs (feet, twist) sample-alias at
  // 60 Hz and thrash (Hairibar does k = m*alpha/dt^2 inside Unity's solver)
  axisLocal: THREE.Vector3; // capsule/twist axis in the body frame
  iPerp: number;
  iTwist: number;
  // rest pose (for resetPose)
  restOffset: THREE.Vector3; // from hips, world at build
  restQuat: THREE.Quaternion;
  lastErr: number; // rad, diagnostic
}

const tmpQ1 = new THREE.Quaternion();
const tmpQ2 = new THREE.Quaternion();
const tmpV1 = new THREE.Vector3();
const tmpV2 = new THREE.Vector3();
const tmpM = new THREE.Matrix4();
const Y_AXIS = new THREE.Vector3(0, 1, 0);

export class Rig {
  world: RAPIER.World;
  model: THREE.Object3D;
  bones = new Map<string, PhysBone>();
  ordered: PhysBone[] = []; // parents before children
  hips: PhysBone;
  mass = 0;
  restHipsHeight = 0;

  constructor(world: RAPIER.World, model: THREE.Object3D) {
    this.world = world;
    this.model = model;
    model.updateMatrixWorld(true);

    for (const def of BONE_DEFS) {
      const bone = findBone(model, def.bone);
      const pos = bone.getWorldPosition(new THREE.Vector3());
      const quat = bone.getWorldQuaternion(new THREE.Quaternion());

      const body = world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
          .setTranslation(pos.x, pos.y, pos.z)
          .setRotation(quat)
          .setLinearDamping(0)
          .setAngularDamping(0.05)
          .setCanSleep(false),
      );
      const shape = this.buildCollider(def, bone, body, pos, quat);

      const pb: PhysBone = {
        def, bone, body,
        parent: def.parent ? this.bones.get(def.parent)! : null,
        qTar: new THREE.Quaternion(), qTarPrev: new THREE.Quaternion(),
        wTar: new THREE.Vector3(), hasTarget: false,
        axisLocal: shape.axisLocal, iPerp: shape.iPerp, iTwist: shape.iTwist,
        restOffset: pos.clone(), restQuat: quat.clone(),
        lastErr: 0,
      };
      this.bones.set(def.key, pb);
      this.ordered.push(pb);
      this.mass += body.mass();
    }

    this.hips = this.bones.get('hips')!;
    this.restHipsHeight = this.hips.restOffset.y;
    for (const pb of this.ordered) pb.restOffset.sub(this.hips.body.translation() as THREE.Vector3);

    // spherical joints at each child bone's pivot
    for (const pb of this.ordered) {
      if (!pb.parent) continue;
      const pivot = pb.bone.getWorldPosition(new THREE.Vector3());
      const a1 = worldPointToBody(pb.parent.body, pivot);
      const a2 = worldPointToBody(pb.body, pivot);
      const params = RAPIER.JointData.spherical(a1, a2);
      world.createImpulseJoint(params, pb.parent.body, pb.body, true);
    }
  }

  private buildCollider(
    def: BoneDef, bone: THREE.Bone, body: RAPIER.RigidBody,
    pos: THREE.Vector3, quat: THREE.Quaternion,
  ): { axisLocal: THREE.Vector3; iPerp: number; iTwist: number } {
    const invQ = tmpQ1.copy(quat).invert();
    let desc: RAPIER.ColliderDesc;
    let axisLocal = new THREE.Vector3(0, 1, 0);
    let iPerp: number, iTwist: number;
    if (def.key === 'hips') {
      // capsule spanning the two hip pivots
      const l = findBone(this.model, M('LeftUpLeg')).getWorldPosition(new THREE.Vector3());
      const r = findBone(this.model, M('RightUpLeg')).getWorldPosition(new THREE.Vector3());
      const mid = l.clone().add(r).multiplyScalar(0.5).sub(pos).applyQuaternion(invQ);
      const dir = l.clone().sub(r).applyQuaternion(invQ);
      const len = dir.length();
      desc = RAPIER.ColliderDesc.capsule(Math.max(0.01, len / 2), def.radius)
        .setTranslation(mid.x, mid.y, mid.z)
        .setRotation(new THREE.Quaternion().setFromUnitVectors(Y_AXIS, dir.normalize()));
      iPerp = iTwist = 0; // root: not PD-driven
    } else if (def.toward) {
      const child = findBone(this.model, def.toward);
      const d = child.getWorldPosition(new THREE.Vector3()).sub(pos).applyQuaternion(invQ);
      const len = d.length();
      const halfH = Math.max(0.01, len / 2 - def.radius * 0.5);
      const mid = d.clone().multiplyScalar(0.5);
      axisLocal = d.clone().normalize();
      desc = RAPIER.ColliderDesc.capsule(halfH, def.radius)
        .setTranslation(mid.x, mid.y, mid.z)
        .setRotation(new THREE.Quaternion().setFromUnitVectors(Y_AXIS, axisLocal));
      // rod-about-end approximation for the joint-pivot inertia
      iPerp = def.mass * (0.25 * def.radius * def.radius + (len * len) / 3);
      iTwist = 0.5 * def.mass * def.radius * def.radius;
    } else {
      desc = RAPIER.ColliderDesc.ball(def.radius);
      iPerp = iTwist = 0.4 * def.mass * def.radius * def.radius;
    }
    desc.setMass(def.mass).setFriction(def.friction).setCollisionGroups(BODY_GROUPS);
    this.world.createCollider(desc, body);
    return { axisLocal, iPerp: Math.max(iPerp, 1e-4), iTwist: Math.max(iTwist, 1e-4) };
  }

  // Cache animation-matching targets from the (mixer-posed) skeleton.
  // Call after mixer.update + model.updateMatrixWorld, before the physics step.
  updateTargets(dt: number) {
    for (const pb of this.ordered) {
      if (!pb.parent) continue;
      pb.qTarPrev.copy(pb.qTar);
      const pq = pb.parent.bone.getWorldQuaternion(tmpQ1);
      const cq = pb.bone.getWorldQuaternion(tmpQ2);
      pb.qTar.copy(pq.invert().multiply(cq));
      if (!pb.hasTarget) {
        pb.qTarPrev.copy(pb.qTar);
        pb.hasTarget = true;
      }
      // finite-difference target angular velocity (parent frame), Hairibar-style.
      // Clamped: clip-blend weight changes can jump the target pose between
      // differently-phased cycles, and an unclamped w spike kicks the limb.
      tmpQ1.copy(pb.qTarPrev).invert().premultiply(pb.qTar); // qTar * inv(qTarPrev)
      const theta = axisAngleShortest(tmpQ1, tmpV1);
      pb.wTar.copy(tmpV1).multiplyScalar(Math.min(theta / dt, 12));
    }
  }

  // Overwrite the rendered skeleton with the simulated pose (mapped bones only;
  // in-between bones keep their animated local rotations). Hairibar's mapping.
  writeBack() {
    for (const pb of this.ordered) {
      const q = pb.body.rotation();
      tmpQ1.set(q.x, q.y, q.z, q.w);
      const parent = pb.bone.parent!;
      parent.getWorldQuaternion(tmpQ2); // refreshes ancestor world matrices
      pb.bone.quaternion.copy(tmpQ2.invert().multiply(tmpQ1));
      if (!pb.parent) {
        const t = pb.body.translation();
        tmpM.copy(parent.matrixWorld).invert();
        pb.bone.position.copy(tmpV1.set(t.x, t.y, t.z).applyMatrix4(tmpM));
      }
      pb.bone.updateMatrixWorld(true);
    }
  }

  // Teleport the whole rig to its rest pose at (x, groundY, z) facing `yaw`.
  // Snapping only the pelvis explodes the solver (learned in v1), so recovery
  // resets every body.
  resetPose(x: number, z: number, yaw: number, groundY: number) {
    const yawQ = tmpQ2.setFromAxisAngle(Y_AXIS, yaw);
    for (const pb of this.ordered) {
      const o = tmpV1.copy(pb.restOffset).applyQuaternion(yawQ);
      pb.body.setTranslation({ x: x + o.x, y: groundY + this.restHipsHeight + o.y, z: z + o.z }, true);
      pb.body.setRotation(tmpQ1.copy(yawQ).multiply(pb.restQuat), true);
      pb.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      pb.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      pb.hasTarget = false;
    }
  }
}

function worldPointToBody(body: RAPIER.RigidBody, p: THREE.Vector3) {
  const t = body.translation();
  const r = body.rotation();
  tmpV2.set(p.x - t.x, p.y - t.y, p.z - t.z);
  tmpQ2.set(r.x, r.y, r.z, r.w).invert();
  const v = tmpV2.applyQuaternion(tmpQ2);
  return { x: v.x, y: v.y, z: v.z };
}
