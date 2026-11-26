# Changelog

## [0.1.2]

- Wheel (1-DOF spherical) joint positions are continuous instead of wrapping at +-pi; drive targets beyond +-pi converge; limited axes clamp drive targets.
- Link order of GetLinkIndex / sorted link lists matches the joint data buffers.
- driveForce of Velocity/Target drives is reported; rebuilds keep root velocity and joint forces; explicit centers of mass are used for inertia.
- Capsule colliders had their cylinder part removed (half height converted twice).
- Heightfield cell diagonals match Unity; Rigidbody max linear velocity is no longer capped at 100 m/s; mass properties match Unity.
- No spurious collision/trigger exit+enter on filter changes; bounded event queues; exit events keep their ids on removals.
- Joints can connect to articulation links (grippers).

## [0.1.1]

- Fix: a link could read the joint position/velocity of another link (wrong DOF order when sibling links have children), which broke wheel odometry and PID control.
- Fix: Velocity/Target articulation drives work with zero gains (wheels of the NR).
- Links of one articulation no longer collide with each other (`Physics.IgnoreCollision` ids are not forwarded to custom backends).
- Windows x86_64 library (built by CI, not yet run in a Windows editor).

## [0.1.0]

- Initial scaffold: native plugin installer, backend selection menu, build backend selector, runtime diagnostics.
