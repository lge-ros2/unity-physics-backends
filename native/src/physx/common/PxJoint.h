#pragma once

// General (non articulation) joints: World::CreateJoint / DestroyJoint and the Joint:: command family.
// A joint handle is the address of a JointObj; it lives in the lock domain of its world.

#include "physx/common/PxObjects.h"

#include "PhysicsJointTypes.h"

namespace backend::pxw
{
	// Limit as sent by Set6DofLimit / Set6DofSpringLimit, kept so the PhysX limit can be rebuilt from both halves.
	struct LimitCache
	{
		bool set = false;
		float min = 0.0f;
		float max = 0.0f;
		float bounciness = 0.0f;
		float bounceMinVelocity = 0.0f;
		float contactDistance = 0.0f;
		float spring = 0.0f;
		float damping = 0.0f;
	};

	struct JointObj
	{
		physx::PxJoint* joint = nullptr; // null once an actor of the joint was destroyed
		WorldObj* world = nullptr;
		BodyObj* body[2] = { nullptr, nullptr }; // null = fixed to the world (or an articulation link, see link)
		// Articulation link handle used as actor instead of a body. A handle, not a pointer: it is validated with
		// Lookup(Kind::Link) on every use, so a link that was freed without telling the joint cannot dangle.
		Handle link[2] = { nullptr, nullptr };
		void* userData = nullptr;
		Physics::JointType type = Physics::JointType::Invalid;
		Physics::JointFlags flags = Physics::JointFlags::None;

		// ConstraintAxis-indexed limits (6DoF, prismatic, ball and socket) and the hinge limit.
		LimitCache limits[9];
		LimitCache hinge;
		bool hingeLimitEnabled = false;
		float hingeMaxForce = 0.0f;
		bool hingeFreeSpin = false;
		float hingeTargetVelocity = 0.0f;

		// Distance joint.
		float distanceMin = 0.0f;
		float distanceMax = PX_MAX_F32; // unbounded until SetDistanceLimit
		float distanceSpring = 0.0f;
		float distanceDamping = 0.0f;
		float distanceTolerance = 0.025f;

		// 6DoF drive targets.
		physx::PxVec3 drivePosition = physx::PxVec3(0.0f);
		physx::PxQuat driveRotation = physx::PxQuat(physx::PxIdentity);
		physx::PxVec3 driveLinearVelocity = physx::PxVec3(0.0f);
		physx::PxVec3 driveAngularVelocity = physx::PxVec3(0.0f);
	};

	// For the event module: our joint handle from the PhysX objects reported in onConstraintBreak
	// (PxConstraintInfo::constraint / externalReference). Null when the object is not one of our live joints.
	Handle JointHandleFromJoint(const physx::PxJoint* joint);
	Handle JointHandleFromConstraint(const physx::PxConstraint* constraint);

	// Caller holds the world domain.
	// Releases the PhysX joints attached to a body that is about to be destroyed; the handles stay valid but report an invalid constraint.
	void DetachJointsFromBodyLocked(BodyObj* body);
	// Releases the PhysX joints attached to an articulation link whose wrapper is about to be freed (FreeLinkWrapper).
	void DetachJointsFromLinkLocked(LinkObj* link);
	// After the PhysX articulation was rebuilt (links got new PxArticulationLink objects), before it is added to the scene again:
	// points the joints that use its links at the new PhysX links. Joints of links that no longer exist are released.
	void RebindJointsOfArticulationLocked(ArticulationObj& art);
	// Destroys every joint of a world that is about to be destroyed.
	void DestroyJointsOfWorldLocked(WorldObj* world);

	void RegisterJointHandlers();
}
