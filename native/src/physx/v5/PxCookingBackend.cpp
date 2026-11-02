#include "physx/common/PxCookingBackend.h"

#include "physx/common/PhysXSdk.h"

namespace backend::cooking
{
	bool CookTriangleMesh(const physx::PxTriangleMeshDesc& desc, const Options& options, physx::PxOutputStream& stream)
	{
		physx::PxPhysics* physics = physx_sdk::GetPhysics();
		if (physics == nullptr)
		{
			return false;
		}
		return PxCookTriangleMesh(MakeParams(physics->getTolerancesScale(), options), desc, stream);
	}

	bool CookConvexMesh(const physx::PxConvexMeshDesc& desc, const Options& options, physx::PxOutputStream& stream,
		bool* polygonLimitReached)
	{
		physx::PxPhysics* physics = physx_sdk::GetPhysics();
		if (physics == nullptr)
		{
			return false;
		}
		physx::PxConvexMeshCookingResult::Enum result = physx::PxConvexMeshCookingResult::eSUCCESS;
		const bool ok = PxCookConvexMesh(MakeParams(physics->getTolerancesScale(), options), desc, stream, &result);
		if (polygonLimitReached != nullptr)
		{
			*polygonLimitReached = result == physx::PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED;
		}
		return ok;
	}

	physx::PxHeightField* CreateHeightField(const physx::PxHeightFieldDesc& desc)
	{
		physx::PxPhysics* physics = physx_sdk::GetPhysics();
		return physics != nullptr ? PxCreateHeightField(desc, physics->getPhysicsInsertionCallback()) : nullptr;
	}

	void Shutdown()
	{
	}
}
