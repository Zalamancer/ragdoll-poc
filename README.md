# Ragdoll PoC

An active ragdoll physics demonstration built with three.js and Rapier, featuring a character that can walk, run, ragdoll, and respond to forces. The system uses procedural inverse kinematics and PD controllers to track animated bone targets while maintaining physics-based movement.

## Run

```bash
npm install
npm run build
open index.html  # or serve locally with a static server
```

Build output is written to `dist/main.js`. Development: `npm run check` verifies TypeScript types.

## Structure

- **src/main.ts** — Entry point; sets up three.js scene, Rapier world (180 Hz physics), animation mixer, character rig, controller, and input handling
- **src/rig.ts** — Physics rig for the Soldier character; defines body segments (hips, limbs, head, spine), collision groups, and bone tracking
- **src/controller.ts** — PD controllers and state machine for character locomotion, ragdoll toggle, and force application
- **src/math.ts** — Utility functions (clamp, etc.)
- **assets/Soldier.glb** — Mixamo character model with Idle/Walk/Run animations
- **index.html** — Canvas container, HUD with keyboard/button controls, error fallbacks
- **dist/** — Compiled ESM bundle (gitignored; rebuild after pulling)

## Notes

`node_modules` and `dist/` are gitignored and must be reinstalled/rebuilt after cloning. The physics engine runs at 180 Hz (3 substeps per 60 Hz simulation frame) with animation sampled at 60 Hz; PD tracking applies once per physics substep to avoid aliasing.
