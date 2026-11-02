#pragma once

#include <PxPhysicsAPI.h>

// PhysX 5.x joint API adaptations (see v4/PxJointCompat.h for the 4.1 counterpart).
namespace backend::compat
{
	// Constraint projection was removed in 5.x.
	constexpr bool kJointProjectionSupported = false;

	inline void ApplyJointProjection(physx::PxJoint&, bool)
	{
	}

	inline void SetJointProjectionTolerance(physx::PxJoint&, float, float)
	{
	}

	// PxJointLimitParameters::contactDistance was removed in 5.x.
	inline void SetLimitContactDistance(physx::PxJointLimitParameters&, float)
	{
	}

	// Deprecated in 5.x but the only flag that makes drive limits forces.
	inline void ApplyDriveLimitsAreForces(physx::PxJoint& joint, bool enabled)
	{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		joint.setConstraintFlag(physx::PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, enabled);
#pragma GCC diagnostic pop
	}

	// Legacy angular drive model: eTWIST / eSWING / eSLERP (the default of 5.x).
	inline physx::PxD6Drive::Enum D6DriveSwing()
	{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		return physx::PxD6Drive::eSWING;
#pragma GCC diagnostic pop
	}
}
