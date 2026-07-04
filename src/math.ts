import * as THREE from 'three';

// Shortest-path axis-angle of a quaternion. Port of DeepMimic
// MathUtil::QuaternionToAxisAngle with the double-cover fix: flipping to the
// w >= 0 hemisphere first is equivalent to wrapping theta into [-pi, pi].
// Returns theta in [0, pi]; axis is written into `outAxis`.
export function axisAngleShortest(q: THREE.Quaternion, outAxis: THREE.Vector3): number {
  let { x, y, z, w } = q;
  if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
  w = Math.min(1, w);
  const sinHalf = Math.sqrt(Math.max(0, 1 - w * w));
  if (sinHalf <= 1e-6) { outAxis.set(0, 0, 1); return 0; }
  outAxis.set(x / sinHalf, y / sinHalf, z / sinHalf);
  return 2 * Math.acos(w);
}

// One-step quaternion integration with a body-frame (local) angular velocity:
// q' = normalize(q ⊗ (1, dt/2 * w)). Used for the Stable-PD predicted pose.
export function integrateQuatLocal(
  q: THREE.Quaternion, wLocal: THREE.Vector3, dt: number, out: THREE.Quaternion,
): THREE.Quaternion {
  const hx = wLocal.x * dt * 0.5, hy = wLocal.y * dt * 0.5, hz = wLocal.z * dt * 0.5;
  out.set(
    q.w * hx + q.x + q.y * hz - q.z * hy,
    q.w * hy - q.x * hz + q.y + q.z * hx,
    q.w * hz + q.x * hy - q.y * hx + q.z,
    q.w - q.x * hx - q.y * hy - q.z * hz,
  );
  return out.normalize();
}

export function clampVecNorm(v: THREE.Vector3, max: number): THREE.Vector3 {
  const n = v.length();
  if (n > max && n > 0) v.multiplyScalar(max / n);
  return v;
}

export const wrapAngle = (a: number): number => Math.atan2(Math.sin(a), Math.cos(a));

export const clamp = (v: number, lo: number, hi: number): number =>
  Math.min(hi, Math.max(lo, v));
