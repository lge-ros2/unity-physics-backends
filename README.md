# unity-physics-backends

Native physics backends for Unity's GameObject physics, built on the Physics Backend Plugin API (`IUnityPhysicsBackendSystem`, Unity 6.6+). `Rigidbody`, `ArticulationBody`, and `Collider` stay as they are; only the engine underneath changes.

[![Unity](https://img.shields.io/badge/Unity-6.6-black.svg?style=flat&logo=unity)](https://unity.com/) [![C++17](https://img.shields.io/badge/C++-17-blue.svg?style=flat&logo=cplusplus)](https://isocpp.org/) [![C#](https://img.shields.io/badge/C%23-Unity%20package-239120.svg?style=flat&logo=csharp)](https://learn.microsoft.com/dotnet/csharp/) [![CMake](https://img.shields.io/badge/CMake-3.10+-064F8C.svg?style=flat&logo=cmake)](https://cmake.org/) [![PhysX](https://img.shields.io/badge/PhysX-5.6.1-76B900.svg?style=flat&logo=nvidia)](https://github.com/NVIDIA-Omniverse/PhysX) [![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows-lightgrey.svg?style=flat)](#install-unity-6000630f1-or-newer-linux--windows-x8664) [![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Build](https://github.com/lge-ros2/unity-physics-backends/actions/workflows/build.yml/badge.svg)](https://github.com/lge-ros2/unity-physics-backends/actions/workflows/build.yml)

Status: **scaffold**. The `PhysX 5` backend registers itself, initializes PhysX 5.6.1 (`PxFoundation`/`PxPhysics`) and answers every command with a stub, so Unity can select it, but nothing is simulated yet.

## Layout

| Path | Contents |
|---|---|
| `native/` | C++ backend (CMake). Output: `libPhysX5Backend.so` (Linux) / `PhysX5Backend.dll` (Windows) |
| `packages/com.lge-ros2.physics-backend.physx5/` | UPM package: runtime info API, editor installer, backend switch menu, build selector |
| `tests/BackendTestProject/` | Minimal Unity project that references the package via `file:` |

## Install (Unity 6000.6.3f1 or newer, Linux / Windows x86_64)

Add to `Packages/manifest.json`:

```json
"com.lge-ros2.physics-backend.physx5": "https://github.com/lge-ros2/unity-physics-backends.git?path=/packages/com.lge-ros2.physics-backend.physx5#upm/v0.1.0"
```

On first load the editor installer copies the native libraries into `Assets/Plugins/PhysX5Backend/linux/` and `.../windows/` (both, so either editor can build a player for the other OS) and asks for an editor restart. Add `Assets/Plugins/PhysX5Backend/` to your `.gitignore`.

A backend inside a UPM package is not selectable by Unity (known issue), which is why the installer copies it into `Assets/Plugins`.

## Switch backend

- Editor menu: `Tools/Physics Backend/PhysX 4 (Built-in)` or `PhysX 5` (writes `m_CurrentBackendId` in `DynamicsManager.asset`).
- Player build: `-physicsBackend physx4|physx5`.

## Build native library

PhysX is linked statically from a git submodule (CPU only). The first build compiles it and takes a few minutes.

```bash
git submodule update --init --depth 1 native/third_party/PhysX5
cmake -S native -B native/build
cmake --build native/build
cmake --install native/build   # -> packages/.../Plugins~/linux/
native/scripts/sync-unity-headers.sh --check   # compare vendored Unity headers with the installed editor
```

On Windows (Visual Studio 2022, static CRT; the PhysX 4.1 reference backend is Linux only):

```bat
git submodule update --init --depth 1 native/third_party/PhysX5
cmake -S native -B native/build -G "Visual Studio 17 2022" -A x64
cmake --build native/build --config Release
cmake --install native/build --config Release   # -> packages/.../Plugins~/windows/
```

`.github/workflows/build.yml` builds both flavors. The built libraries are not committed to `main`: a `v*` tag (matching `package.json` `version`, with a `## [x.y.z]` section in the package CHANGELOG.md that becomes the release notes) makes CI attach them to a GitHub release and add them to a commit tagged `upm/v*`, which is what the UPM git URL points to. Local `cmake --install` still copies the library into the package folder (ignored by git), so a `file:` reference works for development.

Set `-DPHYSX5_DEV_INSTALL_DIR=<project>/Assets/Plugins/PhysX5Backend/linux` to install straight into a Unity project. Restart the editor after replacing the library.

### PhysX 4.1 reference backend (development only)

`PhysX 4 Ref` builds the same backend code against PhysX 4.1 so the glue can be checked against Unity's built-in PhysX 4 before comparing with PhysX 5. It is not shipped in the package.

```bash
git submodule update --init --depth 1 --checkout native/third_party/PhysX4   # large (~800 MB)
cmake -S native -B native/build-ref -DPHYSX_SDK=4
cmake --build native/build-ref        # -> libPhysX4RefBackend.so
```

Copy it to a Unity project's `Assets/Plugins/` (Editor Linux x86_64, preload) and select `PhysX 4 Ref` under Project Settings > Physics > GameObject SDK. Trace with `PHYSX4REF_BACKEND_TRACE=1`.

## Trace

Start the editor (or player) with `PHYSX5_BACKEND_TRACE=1` to log every backend command with its name, target handles and, for known `Set*`/`Create*` commands, its input values. The name tables are generated from the vendored Unity headers by `native/scripts/gen-command-names.py`.

## License

MIT. Third-party notices: [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
