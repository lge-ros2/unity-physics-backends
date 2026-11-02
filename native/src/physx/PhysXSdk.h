#pragma once

namespace backend::physx_sdk
{
	// Creates PxFoundation and PxPhysics. Returns false on failure. Safe to call repeatedly.
	bool Initialize();
	void Shutdown();
}
