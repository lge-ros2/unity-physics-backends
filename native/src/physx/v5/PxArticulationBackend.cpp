#include "physx/common/PxArticulationBackend.h"

#include <algorithm>

// A few calls used here (joint friction coefficient, gravity / Coriolis helpers returning plain joint forces)
// are deprecated in PhysX 5 but are the ones that match Unity's semantics.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace backend::artic
{
	namespace
	{
		physx::PxArticulationAxis::Enum ToPxAxis(int axis)
		{
			return static_cast<physx::PxArticulationAxis::Enum>(std::min(std::max(axis, 0), kAxisCount - 1));
		}

		physx::PxArticulationMotion::Enum ToPxMotion(Motion motion)
		{
			switch (motion)
			{
			case Motion::Limited: return physx::PxArticulationMotion::eLIMITED;
			case Motion::Free: return physx::PxArticulationMotion::eFREE;
			default: return physx::PxArticulationMotion::eLOCKED;
			}
		}

		physx::PxArticulationCacheFlags ToCacheFlags(JointData what)
		{
			switch (what)
			{
			case JointData::Position: return physx::PxArticulationCacheFlag::ePOSITION;
			case JointData::Velocity: return physx::PxArticulationCacheFlag::eVELOCITY;
			case JointData::Acceleration: return physx::PxArticulationCacheFlag::eACCELERATION;
			default: return physx::PxArticulationCacheFlag::eFORCE;
			}
		}

		// Stands in for the 4.1 "target" / "velocity" drive types, which treat the drive as (nearly) rigid.
		constexpr float kRigidGain = 1e25f;
	}

	Articulation* CreateArticulation(physx::PxPhysics& physics)
	{
		return physics.createArticulationReducedCoordinate();
	}

	void SetBaseFlags(Articulation& articulation, bool fixBase, bool driveLimitsAsForces)
	{
		articulation.setArticulationFlag(physx::PxArticulationFlag::eFIX_BASE, fixBase);
		articulation.setArticulationFlag(physx::PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES, driveLimitsAsForces);
	}

	void AddToScene(physx::PxScene& scene, Articulation& articulation)
	{
		scene.addArticulation(articulation);
	}

	void RemoveFromScene(physx::PxScene& scene, Articulation& articulation)
	{
		scene.removeArticulation(articulation);
	}

	void SetRootPose(Articulation& articulation, const physx::PxTransform& pose)
	{
		articulation.setRootGlobalPose(pose, true);
		articulation.updateKinematic(physx::PxArticulationKinematicFlag::ePOSITION);
	}

	void SetRootVelocity(Articulation& articulation, Cache&, const physx::PxVec3* linear, const physx::PxVec3* angular)
	{
		if (linear != nullptr)
		{
			articulation.setRootLinearVelocity(*linear, true);
		}
		if (angular != nullptr)
		{
			articulation.setRootAngularVelocity(*angular, true);
		}
		articulation.updateKinematic(physx::PxArticulationKinematicFlag::eVELOCITY);
	}

	bool SetJointType(Joint& joint, Physics::JointType type)
	{
		switch (type)
		{
		case Physics::JointType::Fixed:
			joint.setJointType(physx::PxArticulationJointType::eFIX);
			return true;
		case Physics::JointType::Prismatic:
			joint.setJointType(physx::PxArticulationJointType::ePRISMATIC);
			return true;
		case Physics::JointType::Hinge:
			// Unwrapped: continuous joints (wheels) must not wrap their position at 2 pi.
			joint.setJointType(physx::PxArticulationJointType::eREVOLUTE_UNWRAPPED);
			return true;
		case Physics::JointType::BallAndSocket:
			joint.setJointType(physx::PxArticulationJointType::eSPHERICAL);
			return true;
		default:
			joint.setJointType(physx::PxArticulationJointType::eFIX);
			return false;
		}
	}

	void SetMotion(Joint& joint, int axis, Motion motion)
	{
		joint.setMotion(ToPxAxis(axis), ToPxMotion(motion));
	}

	void SetLimit(Joint& joint, int axis, float low, float high)
	{
		joint.setLimitParams(ToPxAxis(axis), physx::PxArticulationLimit(low, high));
	}

	void SetDrive(Joint& joint, int axis, const DriveParams& drive)
	{
		float stiffness = drive.stiffness;
		float damping = drive.damping;
		physx::PxArticulationDriveType::Enum type = physx::PxArticulationDriveType::eFORCE;
		switch (drive.type)
		{
		case Physics::ArticulationMotorType::Acceleration:
			type = physx::PxArticulationDriveType::eACCELERATION;
			break;
		case Physics::ArticulationMotorType::Target:
			// Unity / PhysX 4.1 "target": drive straight to the position target, bounded by the force limit.
			if (stiffness > 0.0f || damping > 0.0f)
			{
				stiffness = kRigidGain;
				damping = 0.0f;
			}
			break;
		case Physics::ArticulationMotorType::Velocity:
			if (stiffness > 0.0f || damping > 0.0f)
			{
				stiffness = 0.0f;
				damping = kRigidGain;
			}
			break;
		default:
			break;
		}
		joint.setDriveParams(ToPxAxis(axis), physx::PxArticulationDrive(stiffness, damping, drive.forceLimit, type));
	}

	void SetDriveTarget(Joint& joint, int axis, float target, float targetVelocity)
	{
		joint.setDriveTarget(ToPxAxis(axis), target, true);
		joint.setDriveVelocity(ToPxAxis(axis), targetVelocity, true);
	}

	void SetFriction(Joint& joint, float coefficient)
	{
		joint.setFrictionCoefficient(coefficient);
	}

	void SetMaxVelocity(Joint& joint, float maxVelocity)
	{
		for (int axis = 0; axis < kAxisCount; ++axis)
		{
			joint.setMaxJointVelocity(ToPxAxis(axis), maxVelocity);
		}
	}

	Cache* CreateCache(Articulation& articulation)
	{
		return articulation.createCache();
	}

	void ReleaseCache(Articulation&, Cache* cache)
	{
		if (cache != nullptr)
		{
			cache->release();
		}
	}

	void ZeroCache(Articulation& articulation, Cache& cache)
	{
		articulation.zeroCache(cache);
	}

	void CopyState(Articulation& articulation, Cache& cache, JointData what)
	{
		articulation.copyInternalStateToCache(cache, ToCacheFlags(what));
	}

	void ApplyState(Articulation& articulation, Cache& cache, JointData what)
	{
		articulation.applyCache(cache, ToCacheFlags(what), true);
	}

	float* Data(Cache& cache, JointData what)
	{
		switch (what)
		{
		case JointData::Position: return cache.jointPosition;
		case JointData::Velocity: return cache.jointVelocity;
		case JointData::Acceleration: return cache.jointAcceleration;
		default: return cache.jointForce;
		}
	}

	void ComputeGravityForces(Articulation& articulation, Cache& cache)
	{
		articulation.commonInit();
		articulation.computeGeneralizedGravityForce(cache);
	}

	void ComputeCoriolisForces(Articulation& articulation, Cache& cache)
	{
		articulation.commonInit();
		articulation.computeCoriolisAndCentrifugalForce(cache);
	}

	void ComputeForcesForAcceleration(Articulation& articulation, Cache& cache)
	{
		articulation.commonInit();
		articulation.computeJointForce(cache);
	}

	const float* ComputeDenseJacobian(Articulation& articulation, Cache& cache, uint32_t& rows, uint32_t& cols)
	{
		physx::PxU32 r = 0;
		physx::PxU32 c = 0;
		articulation.commonInit();
		articulation.computeDenseJacobian(cache, r, c);
		rows = r;
		cols = c;
		return cache.denseJacobian;
	}
}

#pragma GCC diagnostic pop
