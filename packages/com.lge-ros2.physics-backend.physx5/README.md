# PhysX 5 Physics Backend

PhysX 5 physics backend for the Unity 6 native physics integration API.

## Requirements

- Unity 6000.6.3f1 or newer
- Linux x86_64 only (Editor and Standalone)

## Installation

Add one line to `Packages/manifest.json` (replace the URL and revision as needed):

```json
"com.lge-ros2.physics-backend.physx5": "https://github.com/lge-ros2/unity-physics-backends.git?path=/packages/com.lge-ros2.physics-backend.physx5#main"
```

## First run

On the first editor start after installing, the package copies
`Plugins~/linux/libPhysX5Backend.so` to `Assets/Plugins/PhysX5Backend/linux/`
(plus a `VERSION` file holding the package version and the SHA-256 of the library)
and configures the plugin importer (Editor + StandaloneLinux64, x86_64, preloaded).

The editor must be restarted once so the native library is loaded. A dialog offers to restart.
The library is replaced via temp file + move, never overwritten in place, because
overwriting a loaded `.so` crashes the editor.

Add `Assets/Plugins/PhysX5Backend/` to your project `.gitignore`; the folder is generated.

## Menu (Tools > Physics Backend)

- `PhysX 4 (Built-in)` / `PhysX 5`: select the backend (`m_CurrentBackendId` in `ProjectSettings/DynamicsManager.asset`). The editor must be restarted afterwards. `PhysX 5` is disabled until the backend is registered with Unity.
- `Log Backend Info`: logs every registered integration and the current selection.
- `Reinstall Native Plugin`: forces a reinstall of the native library.

## Build selection

Pass `-physicsBackend physx4|physx5` to a batch build to select the backend for that build.
When the final selection is not PhysX 5, the native library is excluded from the Standalone Linux build and restored afterwards.

## Runtime API

`Lge.PhysicsBackend.PhysX5.PhysicsBackendInfo`: `IntegrationId`, `IsActive`, `Current`, `LogSummary()`.
