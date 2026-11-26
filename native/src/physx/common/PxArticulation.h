#pragma once

// Objects behind articulation, link and link-joint handles. All of them live in the lock domain of the
// world the articulation was created in (WorldObj::state); the handle of each object is its own address.

#include "physx/common/PxArticulationBackend.h"
#include "physx/common/PxObjects.h"

#include "PhysicsArticulationCommandData.h"

#include <array>
#include <vector>

namespace backend::pxw
{
	// Joint state that is set before the articulation is in a scene (the cache API needs a scene);
	// applied when the articulation is added.
	struct PendingJointState
	{
		float values[3] = { 0.0f, 0.0f, 0.0f };
		bool set = false;
	};

	struct AxisConfig
	{
		artic::Motion motion = artic::Motion::Locked;
		float lowerLimit = 0.0f;
		float upperLimit = 0.0f;
		Physics::ArticulationMotorConfiguration motor;
	};

	// The "joint" handle returned by GetLinkJoint; resolves to its link.
	struct LinkJointObj
	{
		LinkObj* link = nullptr;
	};

	struct LinkObj
	{
		physx::PxArticulationLink* link = nullptr; // null once the PhysX link has been released
		artic::Joint* joint = nullptr;             // inbound joint, null for the root link
		ArticulationObj* art = nullptr;
		LinkObj* parent = nullptr;
		std::vector<LinkObj*> children;
		LinkJointObj* jointHandle = nullptr;
		void* userData = nullptr;
		std::vector<ShapeObj*> shapes;

		Physics::BodyFlags flags = Physics::BodyFlags::Gravity | Physics::BodyFlags::Simulated;
		Physics::CollisionDetectionMode ccdMode = Physics::CollisionDetectionMode::Discrete;
		float mass = 1.0f;
		float linearDamping = 0.0f;
		float angularDamping = 0.05f;
		float maxLinearVelocity = 100.0f;
		float maxAngularVelocity = 7.0f;
		float maxDepenetrationVelocity = 10.0f;

		// Link state that is only kept in PhysX; read back before the PhysX articulation is rebuilt (see RebuildScoped).
		bool dead = false;      // removed from the articulation; the handle stays valid until DestroyLink
		bool hasProps = false;  // body properties below were read from PhysX once
		physx::PxTransform pose;
		physx::PxVec3 inertia = physx::PxVec3(1.0f, 1.0f, 1.0f);
		physx::PxTransform centerOfMass;

		Physics::JointType jointType = Physics::JointType::Fixed;
		std::array<AxisConfig, artic::kAxisCount> axes;
		float friction = 0.0f;
		float maxJointVelocity = 100.0f;
		Physics::Pose parentFrame;
		Physics::Pose childFrame;

		// Continuous joint positions: PhysX wraps spherical joints (and the revolute joints of 4.1) into a fixed range, the
		// reported position is raw + offset, where the offset is advanced whenever the raw value jumps by a period.
		float unwrapOffset[3] = { 0.0f, 0.0f, 0.0f };
		float unwrapLast[3] = { 0.0f, 0.0f, 0.0f };
		bool unwrapInit = false;

		PendingJointState pendingPosition;
		PendingJointState pendingVelocity;
		PendingJointState pendingForce;
	};

	struct ArticulationObj
	{
		artic::Articulation* art = nullptr;
		WorldObj* world = nullptr;
		void* userData = nullptr;
		std::vector<LinkObj*> links;     // alive links in creation order (= PhysX link index order)
		std::vector<LinkObj*> graveyard; // links whose PhysX counterpart is gone but whose handle may still be used
		Physics::ArticulationFlags flags = Physics::ArticulationFlags::None;
		bool added = false;     // handed to AddActorsToScene
		bool simulated = true;  // ArticulationFlags::Simulated (cleared by SetFlags)
		bool inPxScene = false; // currently added to the PhysX scene
		float sleepThreshold = 0.0f;
		uint32_t positionIterations = 6;
		uint32_t velocityIterations = 1;
		// Shapes of all links share this group and never collide with each other (see EncodeShapeFilter).
		uint16_t collisionGroup = 0;
		artic::Cache* cache = nullptr; // lazily created, dropped on every structural change
	};
}
