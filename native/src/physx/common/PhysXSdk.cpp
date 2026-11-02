#include "physx/common/PhysXSdk.h"

#include "core/Dispatch.h"
#include "PxCompat.h"

#include <PxPhysicsAPI.h>

#include <atomic>

namespace
{
	// Physics::ErrorVerbosityLevel: 0 Disabled, 1 Minimal, 2 Default, 3 Verbose, 4 Debug.
	std::atomic<int> g_verbosity{ 3 };

	class ErrorCallback : public physx::PxErrorCallback
	{
	public:
		void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line) override
		{
			const int level = g_verbosity.load(std::memory_order_relaxed);
			switch (code)
			{
			case physx::PxErrorCode::eINVALID_PARAMETER:
				if (level >= 2)
				{
					BACKEND_ERRORF("PhysX error %d: %s (%s:%d)", static_cast<int>(code), message, file, line);
				}
				break;
			case physx::PxErrorCode::eDEBUG_WARNING:
			case physx::PxErrorCode::ePERF_WARNING:
				if (level >= 3)
				{
					BACKEND_WARNF("PhysX warning %d: %s (%s:%d)", static_cast<int>(code), message, file, line);
				}
				break;
			case physx::PxErrorCode::eDEBUG_INFO:
				if (level >= 4)
				{
					BACKEND_LOGF("PhysX info: %s (%s:%d)", message, file, line);
				}
				break;
			case physx::PxErrorCode::eNO_ERROR:
				break;
			default:
				// invalid operation, out of memory, internal error, abort
				if (level >= 1)
				{
					BACKEND_ERRORF("PhysX error %d: %s (%s:%d)", static_cast<int>(code), message, file, line);
				}
				break;
			}
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

		BACKEND_LOGF("PhysX %d.%d.%d initialized (PxPhysics created, compat v%d)",
			PX_PHYSICS_VERSION_MAJOR, PX_PHYSICS_VERSION_MINOR, PX_PHYSICS_VERSION_BUGFIX, compat::kPhysXMajor);
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

	physx::PxFoundation* GetFoundation()
	{
		return g_foundation;
	}

	physx::PxPhysics* GetPhysics()
	{
		return g_physics;
	}

	void SetErrorVerbosity(int level)
	{
		g_verbosity.store(level, std::memory_order_relaxed);
	}
}
