#pragma once

#include <cstdint>

namespace backend
{
	// Binds the PhysX-backed handlers over the stub fallbacks. Call after RegisterStubWorldHandlers().
	void RegisterPhysXWorldHandlers();

	// Called from SDK::InitializeSdk after PxPhysics exists.
	void PhysXWorldOnSdkInitialized(uint16_t engineObjectIdOffset);

	// Destroys every world, body, shape and material. Must run before PxPhysics is released.
	void ShutdownPhysXWorld();
}
