#include "physx/PhysXSdk.h"

#include "core/Dispatch.h"

#include <PxPhysicsAPI.h>

namespace
{
	class ErrorCallback : public physx::PxErrorCallback
	{
	public:
		void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line) override
		{
			BACKEND_ERRORF("PhysX error %d: %s (%s:%d)", static_cast<int>(code), message, file, line);
		}
	};

	ErrorCallback g_errorCallback;
	physx::PxDefaultAllocator g_allocator;
	physx::PxFoundation* g_foundation = nullptr;
	physx::PxPhysics* g_physics = nullptr;
}

namespace backend::physx_sdk
{
	bool Initialize()
	{
		if (g_physics != nullptr)
		{
			return true;
		}

		g_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, g_allocator, g_errorCallback);
		if (g_foundation == nullptr)
		{
			BACKEND_ERRORF("PxCreateFoundation failed");
			return false;
		}

		g_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *g_foundation, physx::PxTolerancesScale());
		if (g_physics == nullptr)
		{
			BACKEND_ERRORF("PxCreatePhysics failed");
			g_foundation->release();
			g_foundation = nullptr;
			return false;
		}

		BACKEND_LOGF("PhysX %d.%d.%d initialized (PxPhysics created)",
			PX_PHYSICS_VERSION_MAJOR, PX_PHYSICS_VERSION_MINOR, PX_PHYSICS_VERSION_BUGFIX);
		return true;
	}

	void Shutdown()
	{
		if (g_physics != nullptr)
		{
			g_physics->release();
			g_physics = nullptr;
		}
		if (g_foundation != nullptr)
		{
			g_foundation->release();
			g_foundation = nullptr;
		}
	}
}
