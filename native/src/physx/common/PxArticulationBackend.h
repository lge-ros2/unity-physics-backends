#pragma once

// PhysX SDK specific part of the articulation module. The implementation lives in physx/v4 and
// physx/v5 (same interface); everything above it (PxArticulation.cpp) is SDK independent.

#include "PhysicsArticulationCommandData.h"
#include "PhysicsJointTypes.h"

#include <PxPhysicsAPI.h>

#include <cstdint>
#include <vector>

namespace backend::artic
{
	using Articulation = physx::PxArticulationReducedCoordinate;
	using Joint = physx::PxArticulationJointReducedCoordinate;
	using Cache = physx::PxArticulationCache;

	// Joint axis indices follow PxArticulationAxis: twist, swing1, swing2, x, y, z.
	constexpr int kAxisCount = 6;

	enum class Motion : uint8_t
	{
		Locked,
		Limited,
		Free
	};

	// Per-DOF arrays of an articulation cache (all of length getDofs()).
	enum class JointData
	{
		Position,
		Velocity,
		Acceleration,
		Force
	};

	struct DriveParams
	{
		float stiffness = 0.0f;
		float damping = 0.0f;
		float forceLimit = 0.0f;
		Physics::ArticulationMotorType type = Physics::ArticulationMotorType::Force;
	};

	// ---- articulation ----

	Articulation* CreateArticulation(physx::PxPhysics& physics);
	void SetBaseFlags(Articulation& articulation, bool fixBase, bool driveLimitsAsForces);
	void AddToScene(physx::PxScene& scene, Articulation& articulation);
	void RemoveFromScene(physx::PxScene& scene, Articulation& articulation);
	// Teleports the root link of an articulation that is in a scene.
	void SetRootPose(Articulation& articulation, const physx::PxTransform& pose);
	// Sets root link velocities (null = keep). Needs the articulation in a scene; cache is scratch space.
	void SetRootVelocity(Articulation& articulation, Cache& cache, const physx::PxVec3* linear, const physx::PxVec3* angular);

	// ---- joint (inbound joint of a link) ----

	// Returns false for types a reduced coordinate joint cannot express (the joint is made fixed then).
	bool SetJointType(Joint& joint, Physics::JointType type);
	void SetMotion(Joint& joint, int axis, Motion motion);
	void SetLimit(Joint& joint, int axis, float low, float high);
	void SetDrive(Joint& joint, int axis, const DriveParams& drive);
	void SetDriveTarget(Joint& joint, int axis, float target, float targetVelocity);
	void SetFriction(Joint& joint, float coefficient);
	void SetMaxVelocity(Joint& joint, float maxVelocity);

	// ---- cache based state access (articulation must be in a scene) ----

	Cache* CreateCache(Articulation& articulation);
	void ReleaseCache(Articulation& articulation, Cache* cache);
	void ZeroCache(Articulation& articulation, Cache& cache);
	void CopyState(Articulation& articulation, Cache& cache, JointData what);
	void ApplyState(Articulation& articulation, Cache& cache, JointData what);
	float* Data(Cache& cache, JointData what);

	// Total force / torque each link receives through its inbound joint (PhysX link index order, child joint frame, root = 0)
	// as of the last step. False when the SDK cannot report it (PhysX 4.1).
	bool ReadIncomingJointForces(Articulation& articulation, Cache& cache, std::vector<physx::PxVec3>& force, std::vector<physx::PxVec3>& torque);

	// Inverse dynamics helpers: the result is left in Data(cache, JointData::Force).
	void ComputeGravityForces(Articulation& articulation, Cache& cache);
	void ComputeCoriolisForces(Articulation& articulation, Cache& cache);
	void ComputeForcesForAcceleration(Articulation& articulation, Cache& cache);
	// Result in the returned array (rows * cols floats), owned by the cache.
	const float* ComputeDenseJacobian(Articulation& articulation, Cache& cache, uint32_t& rows, uint32_t& cols);
}
