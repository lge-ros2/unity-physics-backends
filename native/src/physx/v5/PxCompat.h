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

	// PxQueryFilterCallback::postFilter has a different signature per SDK; the base keeps it out of shared code
	// and forwards to OnPostFilter(hit, shape) (run only when PxQueryFlag::ePOSTFILTER is set).
	class QueryFilterCallbackBase : public physx::PxQueryFilterCallback
	{
	public:
		physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData&, const physx::PxQueryHit& hit,
			const physx::PxShape* shape, const physx::PxRigidActor*) override
		{
			return OnPostFilter(hit, shape);
		}

	protected:
		virtual physx::PxQueryHitType::Enum OnPostFilter(const physx::PxQueryHit&, const physx::PxShape*)
		{
			return physx::PxQueryHitType::eBLOCK;
		}
	};

	// Hit types of PxGeometryQuery::raycast / sweep (PxGeom* in 5.x, PxRaycastHit / PxSweepHit in 4.1).
	using GeomRaycastHit = physx::PxGeomRaycastHit;
	using GeomSweepHit = physx::PxGeomSweepHit;

	inline physx::PxGeometryType::Enum ShapeGeometryType(const physx::PxShape& shape)
	{
		return shape.getGeometry().getType();
	}

	// Copy of the geometry of a shape (getGeometry returns a holder in 4.1, a reference in 5.x).
	inline physx::PxGeometryHolder CopyShapeGeometry(const physx::PxShape& shape)
	{
		physx::PxGeometryHolder holder;
		holder.storeAny(shape.getGeometry());
		return holder;
	}
}
