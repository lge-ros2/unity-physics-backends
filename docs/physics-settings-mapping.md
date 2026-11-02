# Unity Physics Settings -> backend commands

Observed with `PHYSX5_BACKEND_TRACE=1` on Unity 6000.6.7f1 (test project, default settings).
"PhysX 5 target" is the intended mapping and is not implemented or verified yet.

| Unity setting | Command | Observed value | PhysX 5 target |
|---|---|---|---|
| Log Verbosity | `SDK::SetErrorVerbosityLevel` | `Verbose` | `PxErrorCallback` filtering |
| Gravity | `World::SetGravity` | (0,-9.81,0) | `PxScene::setGravity` |
| Bounce Threshold | `World::SetBounceThreshold`, `CreateWorld.bounceThreshold` | 2 | `PxSceneDesc::bounceThresholdVelocity` |
| Scratch buffer | `World::SetScratchBufferChunkCount` | 4 | scratch memory passed to `PxScene::simulate` |
| Solver type | `CreateWorld.solverType` | 0 (PGS), 1 = TGS | `PxSceneDesc::solverType` |
| Broadphase | `CreateWorld.broadphaseType` | 2 (ABP) | `PxSceneDesc::broadPhaseType` |
| Friction type | `CreateWorld.frictionType` | 0 (patch) | `PxSceneDesc::frictionType` (verify in 5.x) |
| Contact pairs mode | `CreateWorld.contactPairsMode` | 0 | `PxPairFilteringMode` in `PxSceneDesc` |
| World bounds / subdivisions | `CreateWorld.worldMin/Max/worldSubdivisions` | +-256, 8 | `PxSceneDesc::sanityBounds`, MBP regions |
| Incremental static broadphase | `CreateWorld.enableIncrementalStaticBroadphase` | 1 | verify |
| Adaptive force | `CreateWorld.enableAdaptiveForce` | 0 | flag may be removed in PhysX 5; verify |
| Enhanced determinism | `CreateWorld.enableEnhancedDeterminism` | 0 | `PxSceneFlag::eENABLE_ENHANCED_DETERMINISM` |
| Fast motion threshold | `CreateWorld.fastMotionThreshold` | float max | CCD settings |
| Job worker count | `CreateWorld.jobWorkerCount` | 31 (main), 15 (import worker) | `PxDefaultCpuDispatcherCreate` |
| Default material | `SDK::GetDefaultMaterial`, `SetMaterialConfiguration` | friction 0.6/0.6, bounciness 0 | `PxMaterial` |
| Material combine modes | `SDK::SetMaterialCombineModes` | Average/Average | `PxMaterial::setFrictionCombineMode`, `setRestitutionCombineMode` |
| Improved patch friction | `SDK::SetImprovedPatchFrictionEnabledGlobal` | 0 in the test project | `PxSceneFlag::eIMPROVED_PATCH_FRICTION` (verify) |
| Default Contact Offset | `Shape::SetContactOffset` | 0.01 | `PxShape::setContactOffset` |
| Default Solver Iterations / Velocity Iterations | `Body::SetSolverIterationsOverride` | position=6, velocity=1 | `PxRigidDynamic::setSolverIterationCounts` |
| Sleep Threshold | `Body::SetSleepThresholdOverride` | 0.005 | `PxRigidDynamic::setSleepThreshold` |
| Default Max Depenetration Velocity | `Body::SetMaxDepenetrationVelocity` | 10 | `PxRigidBody::setMaxDepenetrationVelocity` |
| Rigidbody Mass | `Body::SetMass` | 1 | `PxRigidBody::setMass` |
| Rigidbody Drag / Angular Drag | `Body::SetDamping` | linear 0, angular 0.05 | `setLinearDamping`, `setAngularDamping` |

## Notes

- Per-body values (solver iterations, sleep threshold, max depenetration velocity, damping, mass) are sent for every `Rigidbody` creation, not only when they differ from a default.
- The target object of a command is passed as `sdkObject` (world for `World::`, body for `Body::`, shape for `Shape::`). `ctx.world` is the owning world; `ctx.body` is the owning body for `Shape::` commands.
- Adding a `Rigidbody` to a collider that already has a static shape triggers `Shape::DestroyShape` followed by `Shape::CreateShape`.
- `GetFilterData`, `SetFilterData` and `GetActorDescriptor` are called several times per shape and must be cheap.
- The test project uses Unity defaults. CLOiSim's own settings differ (TGS solver, enhanced determinism and improved patch friction enabled).
