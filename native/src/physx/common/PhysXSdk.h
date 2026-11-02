#pragma once

namespace physx
{
	class PxFoundation;
	class PxPhysics;
}

namespace backend::physx_sdk
{
	// Creates PxFoundation and PxPhysics. Returns false on failure. Safe to call repeatedly.
	bool Initialize();
	void Shutdown();

	// Shared SDK objects (null before Initialize / after Shutdown).
	physx::PxFoundation* GetFoundation();
	physx::PxPhysics* GetPhysics();

	// Maps Physics::ErrorVerbosityLevel (0 = Disabled .. 4 = Debug) onto the PhysX error callback filter.
	void SetErrorVerbosity(int level);
}
