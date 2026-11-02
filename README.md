# unity-physics-backends

Native physics backends for Unity's GameObject physics, built on the Physics Backend Plugin API (`IUnityPhysicsBackendSystem`, Unity 6.6+). `Rigidbody`, `ArticulationBody`, and `Collider` stay as they are; only the engine underneath changes.

Status: **scaffold**. The `PhysX 5` backend registers itself, initializes PhysX 5.6.1 (`PxFoundation`/`PxPhysics`) and answers every command with a stub, so Unity can select it, but nothing is simulated yet.

## Layout

| Path | Contents |
|---|---|
| `native/` | C++ backend (CMake). Output: `libPhysX5Backend.so` |
| `packages/com.lge-ros2.physics-backend.physx5/` | UPM package: runtime info API, editor installer, backend switch menu, build selector |
| `tests/BackendTestProject/` | Minimal Unity project that references the package via `file:` |

## Install (Unity 6000.6.3f1 or newer, Linux x86_64)

Add to `Packages/manifest.json`:

```json
"com.lge-ros2.physics-backend.physx5": "https://github.com/lge-ros2/unity-physics-backends.git?path=/packages/com.lge-ros2.physics-backend.physx5"
```

On first load the editor installer copies the native library into `Assets/Plugins/PhysX5Backend/linux/` and asks for an editor restart. Add `Assets/Plugins/PhysX5Backend/` to your `.gitignore`.

A backend inside a UPM package is not selectable by Unity (known issue), which is why the installer copies it into `Assets/Plugins`.

## Switch backend

- Editor menu: `Tools/Physics Backend/PhysX 4 (Built-in)` or `PhysX 5` (writes `m_CurrentBackendId` in `DynamicsManager.asset`).
- Player build: `-physicsBackend physx4|physx5`.

## Build native library

PhysX 5.6.1 is a git submodule (CPU only, linked statically). The first build compiles it and takes a few minutes.

```bash
git submodule update --init --depth 1 native/third_party/PhysX
cmake -S native -B native/build
cmake --build native/build
cmake --install native/build   # -> packages/.../Plugins~/linux/
native/scripts/sync-unity-headers.sh --check   # compare vendored Unity headers with the installed editor
```

Set `-DPHYSX5_DEV_INSTALL_DIR=<project>/Assets/Plugins/PhysX5Backend/linux` to install straight into a Unity project. Restart the editor after replacing the library.

## Trace

Start the editor (or player) with `PHYSX5_BACKEND_TRACE=1` to log every backend command with its name, target handles and, for known `Set*`/`Create*` commands, its input values. The name tables are generated from the vendored Unity headers by `native/scripts/gen-command-names.py`.

## License

MIT. Third-party notices: [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
