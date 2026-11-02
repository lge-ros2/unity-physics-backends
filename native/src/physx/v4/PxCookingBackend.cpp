#include "physx/common/PxCookingBackend.h"

#include "physx/common/PhysXSdk.h"

#include <mutex>

namespace backend::cooking
{
	namespace
	{
		// PxCooking carries its parameters, so one shared object is reconfigured per call under a lock.
		std::mutex g_mutex;
		physx::PxCooking* g_cooking = nullptr;

		// Caller holds g_mutex.
		physx::PxCooking* Acquire(const Options& options)
		{
			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			physx::PxFoundation* foundation = physx_sdk::GetFoundation();
			if (physics == nullptr || foundation == nullptr)
			{
				return nullptr;
			}
			const physx::PxCookingParams params = MakeParams(physics->getTolerancesScale(), options);
			if (g_cooking == nullptr)
			{
				g_cooking = PxCreateCooking(PX_PHYSICS_VERSION, *foundation, params);
			}
			else
			{
				g_cooking->setParams(params);
			}
			return g_cooking;
		}
	}

	bool CookTriangleMesh(const physx::PxTriangleMeshDesc& desc, const Options& options, physx::PxOutputStream& stream)
	{
		std::lock_guard<std::mutex> guard(g_mutex);
		physx::PxCooking* cooking = Acquire(options);
		return cooking != nullptr && cooking->cookTriangleMesh(desc, stream);
	}

	bool CookConvexMesh(const physx::PxConvexMeshDesc& desc, const Options& options, physx::PxOutputStream& stream,
		bool* polygonLimitReached)
	{
		std::lock_guard<std::mutex> guard(g_mutex);
		physx::PxCooking* cooking = Acquire(options);
		if (cooking == nullptr)
		{
			return false;
		}
		physx::PxConvexMeshCookingResult::Enum result = physx::PxConvexMeshCookingResult::eSUCCESS;
		const bool ok = cooking->cookConvexMesh(desc, stream, &result);
		if (polygonLimitReached != nullptr)
		{
			*polygonLimitReached = result == physx::PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED;
		}
		return ok;
	}

	physx::PxHeightField* CreateHeightField(const physx::PxHeightFieldDesc& desc)
	{
		std::lock_guard<std::mutex> guard(g_mutex);
		physx::PxCooking* cooking = Acquire(Options());
		physx::PxPhysics* physics = physx_sdk::GetPhysics();
		return (cooking != nullptr && physics != nullptr) ? cooking->createHeightField(desc, physics->getPhysicsInsertionCallback()) : nullptr;
	}

	void Shutdown()
	{
		std::lock_guard<std::mutex> guard(g_mutex);
		if (g_cooking != nullptr)
		{
			g_cooking->release();
			g_cooking = nullptr;
		}
	}
}
