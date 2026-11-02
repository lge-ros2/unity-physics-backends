#pragma once

#include <PxPhysicsAPI.h>

// PhysX 5.x specific adaptations. Shared glue in physx/common includes this header,
// so API differences between 4.1 and 5.x are confined to physx/v4 and physx/v5.
namespace backend::compat
{
	constexpr int kPhysXMajor = 5;

	// PxSceneDesc::frictionType is deprecated in 5.x (patch friction only).
	// Returns false when the requested type cannot be honoured.
	// kind: 0 = patch, 1 = one directional, 2 = two directional.
	inline bool ApplyFrictionType(physx::PxSceneDesc&, int kind)
	{
		return kind == 0;
	}

	// eADAPTIVE_FORCE no longer exists in 5.x.
	inline void ApplyAdaptiveForce(physx::PxSceneDesc&, bool)
	{
	}

	// The improved patch friction flag is a PxMaterialFlag in 4.1 only.
	inline void ApplyImprovedPatchFriction(physx::PxMaterial&, bool)
	{
	}

	inline physx::PxBroadPhaseRegion MakeBroadPhaseRegion(const physx::PxBounds3& bounds)
	{
		physx::PxBroadPhaseRegion region;
		region.mBounds = bounds;
		region.mUserData = nullptr;
		return region;
	}

	inline physx::PxBounds3 ComputeGeometryBounds(const physx::PxGeometry& geom, const physx::PxTransform& pose)
	{
		physx::PxBounds3 bounds;
		physx::PxGeometryQuery::computeGeomBounds(bounds, geom, pose, 0.0f, 1.0f);
		return bounds;
	}

	// PxQueryFilterCallback::postFilter has a different signature per SDK; the base keeps it out of shared code.
	class QueryFilterCallbackBase : public physx::PxQueryFilterCallback
	{
	public:
		physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData&, const physx::PxQueryHit&,
			const physx::PxShape*, const physx::PxRigidActor*) override
		{
			return physx::PxQueryHitType::eBLOCK;
		}
	};
}
