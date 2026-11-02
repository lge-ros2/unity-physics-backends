#pragma once

#include <PxPhysicsAPI.h>

#include <cstring>

// PhysX 4.1 joint API adaptations (reference backend; see v5/PxJointCompat.h for the 5.x counterpart).
namespace backend::compat
{
	constexpr bool kJointProjectionSupported = true;

	inline void ApplyJointProjection(physx::PxJoint& joint, bool enabled)
	{
		joint.setConstraintFlag(physx::PxConstraintFlag::ePROJECTION, enabled);
	}

	// Linear tolerance applies to every joint type, angular only to the ones that can drift in rotation.
	inline void SetJointProjectionTolerance(physx::PxJoint& joint, float linear, float angular)
	{
		using namespace physx;
		auto is = [&joint](const char* name) { return std::strcmp(joint.getConcreteTypeName(), name) == 0; };
		if (is("PxD6Joint"))
		{
			auto& j = static_cast<PxD6Joint&>(joint);
			j.setProjectionLinearTolerance(linear);
			j.setProjectionAngularTolerance(angular);
		}
		else if (is("PxFixedJoint"))
		{
			auto& j = static_cast<PxFixedJoint&>(joint);
			j.setProjectionLinearTolerance(linear);
			j.setProjectionAngularTolerance(angular);
		}
		else if (is("PxRevoluteJoint"))
		{
			auto& j = static_cast<PxRevoluteJoint&>(joint);
			j.setProjectionLinearTolerance(linear);
			j.setProjectionAngularTolerance(angular);
		}
		else if (is("PxPrismaticJoint"))
		{
			auto& j = static_cast<PxPrismaticJoint&>(joint);
			j.setProjectionLinearTolerance(linear);
			j.setProjectionAngularTolerance(angular);
		}
		else if (is("PxSphericalJoint"))
		{
			static_cast<PxSphericalJoint&>(joint).setProjectionLinearTolerance(linear);
		}
	}

	// Values <= 0 keep the default the limit constructor computed.
	inline void SetLimitContactDistance(physx::PxJointLimitParameters& limit, float distance)
	{
		if (distance > 0.0f)
		{
			limit.contactDistance = distance;
		}
	}

	inline void ApplyDriveLimitsAreForces(physx::PxJoint& joint, bool enabled)
	{
		joint.setConstraintFlag(physx::PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, enabled);
	}

	inline physx::PxD6Drive::Enum D6DriveSwing()
	{
		return physx::PxD6Drive::eSWING;
	}
}
