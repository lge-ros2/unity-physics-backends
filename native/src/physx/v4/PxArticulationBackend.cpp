#include "physx/common/PxArticulationBackend.h"

#include "core/Dispatch.h"

#include <algorithm>

// PhysX 4.1 reference backend. Joint state (positions, velocities, forces) is only reachable through
// PxArticulationCache here; the 5.x specific direct accessors do not exist.

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
			case JointData::Position: return physx::PxArticulationCache::ePOSITION;
			case JointData::Velocity: return physx::PxArticulationCache::eVELOCITY;
			case JointData::Acceleration: return physx::PxArticulationCache::eACCELERATION;
			default: return physx::PxArticulationCache::eFORCE;
			}
		}
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
		articulation.teleportRootLink(pose, true);
	}

	void SetRootVelocity(Articulation& articulation, Cache& cache, const physx::PxVec3* linear, const physx::PxVec3* angular)
	{
		if (cache.rootLinkData == nullptr)
		{
			return;
		}
		// 4.1 only applies the root velocity together with eVELOCITY (which also writes the joint velocities back).
		const physx::PxArticulationCacheFlags flags = physx::PxArticulationCache::eROOT | physx::PxArticulationCache::eVELOCITY;
		articulation.copyInternalStateToCache(cache, flags);
		if (linear != nullptr)
		{
			cache.rootLinkData->worldLinVel = *linear;
		}
		if (angular != nullptr)
		{
			cache.rootLinkData->worldAngVel = *angular;
		}
		articulation.applyCache(cache, flags, true);
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
			joint.setJointType(physx::PxArticulationJointType::eREVOLUTE);
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
		joint.setLimit(ToPxAxis(axis), low, high);
	}

	void SetDrive(Joint& joint, int axis, const DriveParams& drive)
	{
		physx::PxArticulationDriveType::Enum type = physx::PxArticulationDriveType::eFORCE;
		switch (drive.type)
		{
		case Physics::ArticulationMotorType::Acceleration: type = physx::PxArticulationDriveType::eACCELERATION; break;
		case Physics::ArticulationMotorType::Target: type = physx::PxArticulationDriveType::eTARGET; break;
		case Physics::ArticulationMotorType::Velocity: type = physx::PxArticulationDriveType::eVELOCITY; break;
		default: break;
		}
		joint.setDrive(ToPxAxis(axis), drive.stiffness, drive.damping, drive.forceLimit, type);
	}

	void SetDriveTarget(Joint& joint, int axis, float target, float targetVelocity)
	{
		joint.setDriveTarget(ToPxAxis(axis), target);
		joint.setDriveVelocity(ToPxAxis(axis), targetVelocity);
	}

	void SetFriction(Joint& joint, float coefficient)
	{
		joint.setFrictionCoefficient(coefficient);
	}

	void SetMaxVelocity(Joint& joint, float maxVelocity)
	{
		joint.setMaxJointVelocity(maxVelocity);
	}

	Cache* CreateCache(Articulation& articulation)
	{
		return articulation.createCache();
	}

	void ReleaseCache(Articulation& articulation, Cache* cache)
	{
		if (cache != nullptr)
		{
			articulation.releaseCache(*cache);
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

	namespace
	{
		// The 4.1 inverse dynamics entry points other than the gravity one (computeCoriolisAndCentrifugalForce,
		// computeJointForce) crash inside the SDK for caches created by createCache(): their scratch allocation
		// (allocateScratchSpatialData) comes back empty. They report zero forces instead.
		void ReportZeroForces(Articulation& articulation, Cache& cache, const char* what)
		{
			static bool s_warned = false;
			if (!s_warned)
			{
				s_warned = true;
				BACKEND_WARNF("%s is not available on PhysX 4.1; reporting zeros", what);
			}
			articulation.zeroCache(cache);
		}
	}

	void ComputeCoriolisForces(Articulation& articulation, Cache& cache)
	{
		ReportZeroForces(articulation, cache, "Coriolis / centrifugal joint forces");
	}

	void ComputeForcesForAcceleration(Articulation& articulation, Cache& cache)
	{
		ReportZeroForces(articulation, cache, "Joint forces for a given acceleration");
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
