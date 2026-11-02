# Third-Party Notices

## Unity Physics Backend Plugin API headers

- Location: `native/third_party/unity/`
- Source: Unity Editor `Data/PluginAPI/` (Unity 6000.6.7f1, see `native/third_party/unity/UNITY_VERSION`)
- Copyright (c) Unity Technologies ApS
- License: [Unity Companion License for Unity-dependent projects](http://www.unity3d.com/legal/licenses/Unity_Companion_License)

These headers are provided unmodified and are used only to build plugins that run inside Unity.

## NVIDIA PhysX 5.6.1

- Location: `native/third_party/PhysX5` (git submodule, tag `107.3-physx-5.6.1`), statically linked into `libPhysX5Backend.so`
- Copyright (c) NVIDIA Corporation
- License: BSD-3-Clause (see `native/third_party/PhysX5/LICENSE.md`)

## NVIDIA PhysX 4.1 (development only)

- Location: `native/third_party/PhysX4` (git submodule, branch `4.1`), linked only into the `PhysX 4 Ref` verification backend, which is not distributed
- Copyright (c) NVIDIA Corporation
- License: BSD-3-Clause (see `native/third_party/PhysX4/LICENSE.md`)
