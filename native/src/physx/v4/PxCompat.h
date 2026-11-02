#pragma once

#include <PxPhysicsAPI.h>

// PhysX 4.1 specific adaptations (reference backend). Shared glue in physx/common includes this header,
// so API differences between 4.1 and 5.x are confined to physx/v4 and physx/v5.
namespace backend::compat
{
	constexpr int kPhysXMajor = 4;

	// kind: 0 = patch, 1 = one directional, 2 = two directional.
	inline bool ApplyFrictionType(physx::PxSceneDesc& desc, int kind)
	{
		switch (kind)
		{
		case 1: desc.frictionType = physx::PxFrictionType::eONE_DIRECTIONAL; return true;
		case 2: desc.frictionType = physx::PxFrictionType::eTWO_DIRECTIONAL; return true;
		default: desc.frictionType = physx::PxFrictionType::ePATCH; return true;
		}
	}

	inline void ApplyAdaptiveForce(physx::PxSceneDesc& desc, bool enabled)
	{
		if (enabled)
		{
			desc.flags |= physx::PxSceneFlag::eADAPTIVE_FORCE;
		}
	}

	inline void ApplyImprovedPatchFriction(physx::PxMaterial& material, bool enabled)
	{
		material.setFlag(physx::PxMaterialFlag::eIMPROVED_PATCH_FRICTION, enabled);
	}

	inline physx::PxBroadPhaseRegion MakeBroadPhaseRegion(const physx::PxBounds3& bounds)
	{
		physx::PxBroadPhaseRegion region;
		region.bounds = bounds;
		region.userData = nullptr;
		return region;
	}

	inline physx::PxBounds3 ComputeGeometryBounds(const physx::PxGeometry& geom, const physx::PxTransform& pose)
	{
		return physx::PxGeometryQuery::getWorldBounds(geom, pose, 1.0f);
	}

	// PxQueryFilterCallback::postFilter has a different signature per SDK; the base keeps it out of shared code
	// and forwards to OnPostFilter(hit, shape) (run only when PxQueryFlag::ePOSTFILTER is set).
	class QueryFilterCallbackBase : public physx::PxQueryFilterCallback
	{
	public:
		physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData&, const physx::PxQueryHit& hit) override
		{
			return OnPostFilter(hit, hit.shape);
		}

	protected:
		virtual physx::PxQueryHitType::Enum OnPostFilter(const physx::PxQueryHit&, const physx::PxShape*)
		{
			return physx::PxQueryHitType::eBLOCK;
		}
	};

	// Hit types of PxGeometryQuery::raycast / sweep (PxGeom* in 5.x, PxRaycastHit / PxSweepHit in 4.1).
	using GeomRaycastHit = physx::PxRaycastHit;
	using GeomSweepHit = physx::PxSweepHit;

	inline physx::PxGeometryType::Enum ShapeGeometryType(const physx::PxShape& shape)
	{
		return shape.getGeometryType();
	}

	// Copy of the geometry of a shape (getGeometry returns a holder in 4.1, a reference in 5.x).
	inline physx::PxGeometryHolder CopyShapeGeometry(const physx::PxShape& shape)
	{
		return shape.getGeometry();
	}
}
