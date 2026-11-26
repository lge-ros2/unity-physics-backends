# Changelog

## [0.1.1]

- Fix: a link could read the joint position/velocity of another link (wrong DOF order when sibling links have children), which broke wheel odometry and PID control.
- Fix: Velocity/Target articulation drives work with zero gains (wheels of the NR).
- Links of one articulation no longer collide with each other (`Physics.IgnoreCollision` ids are not forwarded to custom backends).
- Windows x86_64 library (built by CI, not yet run in a Windows editor).

## [0.1.0]

- Initial scaffold: native plugin installer, backend selection menu, build backend selector, runtime diagnostics.
