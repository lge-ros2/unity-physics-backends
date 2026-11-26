#include "physx/common/PxJoint.h"

#include "core/Dispatch.h"
#include "physx/common/PhysXSdk.h"
#include "physx/common/PxArticulation.h"

#include "PxJointCompat.h"

#include "PhysicsJointCommandData.h"
#include "PhysicsWorldCommandData.h"

#include <algorithm>
#include <cmath>

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace jd = PhysicsCommands::JointData;
	namespace wd = PhysicsCommands::WorldData;

	using Physics::ConstraintAxis;
	using Physics::JointDofLock;
	using Physics::JointFlags;
	using Physics::JointType;
	using Physics::PoseTarget;

	namespace
	{
		constexpr float kPi = 3.14159265358979f;
		constexpr float kMaxAngle = kPi - 1e-3f;
		constexpr float kMinConeAngle = 1e-3f;
		constexpr float kMaxLinear = 1.0e9f;

		float Sane(float v, float fallback)
		{
			return physx::PxIsFinite(v) ? v : fallback;
		}

		// Force / torque / distance style values where infinity means "unbounded".
		float Unbounded(float v)
		{
			return (!physx::PxIsFinite(v) || v > PX_MAX_F32) ? PX_MAX_F32 : std::max(v, 0.0f);
		}

		float NonNegative(float v)
		{
			return std::max(Sane(v, 0.0f), 0.0f);
		}

		bool Has(JointFlags flags, JointFlags bit)
		{
			return (flags & bit) != JointFlags::None;
		}

		physx::PxD6Joint* AsD6(JointObj& j)
		{
			return (j.joint != nullptr && j.type == JointType::Dof6) ? static_cast<physx::PxD6Joint*>(j.joint) : nullptr;
		}

		physx::PxRevoluteJoint* AsRevolute(JointObj& j)
		{
			return (j.joint != nullptr && j.type == JointType::Hinge) ? static_cast<physx::PxRevoluteJoint*>(j.joint) : nullptr;
		}

		physx::PxDistanceJoint* AsDistance(JointObj& j)
		{
			return (j.joint != nullptr && j.type == JointType::Distance) ? static_cast<physx::PxDistanceJoint*>(j.joint) : nullptr;
		}

		physx::PxPrismaticJoint* AsPrismatic(JointObj& j)
		{
			return (j.joint != nullptr && j.type == JointType::Prismatic) ? static_cast<physx::PxPrismaticJoint*>(j.joint) : nullptr;
		}

		physx::PxSphericalJoint* AsSpherical(JointObj& j)
		{
			return (j.joint != nullptr && j.type == JointType::BallAndSocket) ? static_cast<physx::PxSphericalJoint*>(j.joint) : nullptr;
		}

		int ActorIndex(PoseTarget target)
		{
			return target == PoseTarget::Child ? 1 : 0;
		}

		physx::PxJointActorIndex::Enum PxActorIndex(PoseTarget target)
		{
			return target == PoseTarget::Child ? physx::PxJointActorIndex::eACTOR1 : physx::PxJointActorIndex::eACTOR0;
		}

		// ---- flags ----

		void ApplyFlags(JointObj& j)
		{
			if (j.joint == nullptr)
			{
				return;
			}
			j.joint->setConstraintFlag(physx::PxConstraintFlag::eDISABLE_PREPROCESSING, !Has(j.flags, JointFlags::Preprocessing));
			j.joint->setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, Has(j.flags, JointFlags::CollisionReporting));
			compat::ApplyJointProjection(*j.joint, Has(j.flags, JointFlags::Projection));
			compat::ApplyDriveLimitsAreForces(*j.joint, Has(j.flags, JointFlags::DriveLimitsAreForces));
		}

		// ---- limits ----

		void FillLimitParameters(physx::PxJointLimitParameters& p, const LimitCache& c)
		{
			p.restitution = std::min(NonNegative(c.bounciness), 1.0f);
			p.bounceThreshold = NonNegative(c.bounceMinVelocity);
			p.stiffness = NonNegative(c.spring);
			p.damping = NonNegative(c.damping);
			compat::SetLimitContactDistance(p, Sane(c.contactDistance, 0.0f));
		}

		void OrderedRange(const LimitCache& c, float bound, float& lower, float& upper)
		{
			lower = std::min(std::max(Sane(c.min, -bound), -bound), bound);
			upper = std::min(std::max(Sane(c.max, bound), -bound), bound);
			if (upper < lower)
			{
				std::swap(lower, upper);
			}
		}

		physx::PxJointLinearLimitPair MakeLinearLimit(const LimitCache& c)
		{
			float lower, upper;
			OrderedRange(c, kMaxLinear, lower, upper);
			physx::PxJointLinearLimitPair limit(lower, upper, physx::PxSpring(0.0f, 0.0f));
			FillLimitParameters(limit, c);
			return limit;
		}

		physx::PxJointAngularLimitPair MakeAngularLimit(const LimitCache& c)
		{
			float lower, upper;
			OrderedRange(c, kMaxAngle, lower, upper);
			physx::PxJointAngularLimitPair limit(lower, upper, physx::PxSpring(0.0f, 0.0f));
			FillLimitParameters(limit, c);
			return limit;
		}

		float ConeAngle(const LimitCache& c)
		{
			const float a = std::max(std::fabs(Sane(c.min, 0.0f)), std::fabs(Sane(c.max, 0.0f)));
			return std::min(std::max(a, kMinConeAngle), kMaxAngle);
		}

		physx::PxJointLimitCone MakeConeLimit(float yAngle, float zAngle, const LimitCache& params)
		{
			physx::PxJointLimitCone limit(yAngle, zAngle);
			FillLimitParameters(limit, params);
			return limit;
		}

		bool Symmetric(const LimitCache& c)
		{
			return std::fabs(Sane(c.min, 0.0f) + Sane(c.max, 0.0f)) < 1e-4f;
		}

		void ApplySwingLimit(JointObj& j, physx::PxD6Joint& d6)
		{
			const LimitCache& y = j.limits[static_cast<int>(ConstraintAxis::AngularY)];
			const LimitCache& z = j.limits[static_cast<int>(ConstraintAxis::AngularZ)];
			const LimitCache& yz = j.limits[static_cast<int>(ConstraintAxis::AngularYZ)];

			if (!y.set && !z.set)
			{
				if (yz.set)
				{
					const float a = ConeAngle(yz);
					d6.setSwingLimit(MakeConeLimit(a, a, yz));
				}
				return;
			}

			// Per axis limits: a cone when both are symmetric, otherwise a pyramid.
			const LimitCache& params = y.set ? y : z;
			if ((!y.set || Symmetric(y)) && (!z.set || Symmetric(z)))
			{
				d6.setSwingLimit(MakeConeLimit(y.set ? ConeAngle(y) : kMaxAngle, z.set ? ConeAngle(z) : kMaxAngle, params));
				return;
			}

			float yLower = -kMaxAngle, yUpper = kMaxAngle, zLower = -kMaxAngle, zUpper = kMaxAngle;
			if (y.set)
			{
				OrderedRange(y, kMaxAngle, yLower, yUpper);
			}
			if (z.set)
			{
				OrderedRange(z, kMaxAngle, zLower, zUpper);
			}
			physx::PxJointLimitPyramid pyramid(yLower, yUpper, zLower, zUpper);
			FillLimitParameters(pyramid, params);
			d6.setPyramidSwingLimit(pyramid);
		}

		physx::PxD6Axis::Enum LinearAxis(ConstraintAxis axis)
		{
			switch (axis)
			{
			case ConstraintAxis::LinearY: return physx::PxD6Axis::eY;
			case ConstraintAxis::LinearZ: return physx::PxD6Axis::eZ;
			default: return physx::PxD6Axis::eX;
			}
		}

		void ApplyLimit(JointObj& j, ConstraintAxis axis)
		{
			const int index = static_cast<int>(axis);
			if (j.joint == nullptr || index < 0 || index >= 9 || !j.limits[index].set)
			{
				return;
			}
			const LimitCache& c = j.limits[index];

			switch (j.type)
			{
			case JointType::Dof6:
			{
				physx::PxD6Joint* d6 = AsD6(j);
				switch (axis)
				{
				case ConstraintAxis::LinearX:
				case ConstraintAxis::LinearY:
				case ConstraintAxis::LinearZ:
					d6->setLinearLimit(LinearAxis(axis), MakeLinearLimit(c));
					break;
				case ConstraintAxis::Distance:
				{
					physx::PxJointLinearLimit limit(Unbounded(c.max), physx::PxSpring(0.0f, 0.0f));
					FillLimitParameters(limit, c);
					d6->setDistanceLimit(limit);
					break;
				}
				case ConstraintAxis::AngularX:
					d6->setTwistLimit(MakeAngularLimit(c));
					break;
				case ConstraintAxis::AngularY:
				case ConstraintAxis::AngularZ:
				case ConstraintAxis::AngularYZ:
					ApplySwingLimit(j, *d6);
					break;
				default:
					break;
				}
				break;
			}
			case JointType::Prismatic:
				if (axis == ConstraintAxis::LinearX)
				{
					physx::PxPrismaticJoint* prismatic = AsPrismatic(j);
					prismatic->setLimit(MakeLinearLimit(c));
					prismatic->setPrismaticJointFlag(physx::PxPrismaticJointFlag::eLIMIT_ENABLED, true);
				}
				break;
			case JointType::BallAndSocket:
				if (axis == ConstraintAxis::AngularYZ)
				{
					physx::PxSphericalJoint* spherical = AsSpherical(j);
					const float a = ConeAngle(c);
					spherical->setLimitCone(MakeConeLimit(a, a, c));
					spherical->setSphericalJointFlag(physx::PxSphericalJointFlag::eLIMIT_ENABLED, true);
				}
				break;
			default:
				break;
			}
		}

		// ---- 6DoF motion / drives ----

		physx::PxD6Motion::Enum ToPxMotion(JointDofLock value)
		{
			switch (value)
			{
			case JointDofLock::Limited: return physx::PxD6Motion::eLIMITED;
			case JointDofLock::Free: return physx::PxD6Motion::eFREE;
			default: return physx::PxD6Motion::eLOCKED;
			}
		}

		// Returns false for axes that have no drive.
		bool DriveIndex(ConstraintAxis axis, physx::PxD6Drive::Enum& index)
		{
			switch (axis)
			{
			case ConstraintAxis::LinearX: index = physx::PxD6Drive::eX; return true;
			case ConstraintAxis::LinearY: index = physx::PxD6Drive::eY; return true;
			case ConstraintAxis::LinearZ: index = physx::PxD6Drive::eZ; return true;
			case ConstraintAxis::AngularX: index = physx::PxD6Drive::eTWIST; return true;
			case ConstraintAxis::AngularY:
			case ConstraintAxis::AngularZ:
			case ConstraintAxis::AngularYZ: index = compat::D6DriveSwing(); return true;
			case ConstraintAxis::Slerp: index = physx::PxD6Drive::eSLERP; return true;
			default: return false;
			}
		}

		void ApplyDrivePosition(JointObj& j)
		{
			if (physx::PxD6Joint* d6 = AsD6(j))
			{
				d6->setDrivePosition(physx::PxTransform(j.drivePosition, j.driveRotation));
			}
		}

		void ApplyDriveVelocity(JointObj& j)
		{
			if (physx::PxD6Joint* d6 = AsD6(j))
			{
				d6->setDriveVelocity(j.driveLinearVelocity, j.driveAngularVelocity);
			}
		}

		// ---- hinge ----

		void ApplyHingeLimit(JointObj& j)
		{
			physx::PxRevoluteJoint* hinge = AsRevolute(j);
			if (hinge != nullptr && j.hinge.set)
			{
				hinge->setLimit(MakeAngularLimit(j.hinge));
			}
		}

		void ApplyHingeMotor(JointObj& j)
		{
			physx::PxRevoluteJoint* hinge = AsRevolute(j);
			if (hinge == nullptr)
			{
				return;
			}
			hinge->setDriveForceLimit(Unbounded(j.hingeMaxForce));
			hinge->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eDRIVE_FREESPIN, j.hingeFreeSpin);
			hinge->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eDRIVE_ENABLED, j.hingeMaxForce > 0.0f);
			hinge->setDriveVelocity(Sane(j.hingeTargetVelocity, 0.0f));
		}

		// ---- distance ----

		void ApplyDistance(JointObj& j)
		{
			physx::PxDistanceJoint* distance = AsDistance(j);
			if (distance == nullptr)
			{
				return;
			}
			const float minDistance = NonNegative(j.distanceMin);
			// An unbounded max stays enabled with a huge value: PhysX 4.1 pins the distance when both limits are off.
			const bool bounded = physx::PxIsFinite(j.distanceMax) && j.distanceMax < kMaxLinear;
			const float maxDistance = bounded ? std::max(j.distanceMax, minDistance) : kMaxLinear;

			distance->setMinDistance(minDistance);
			distance->setMaxDistance(maxDistance);
			distance->setDistanceJointFlag(physx::PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, minDistance > 0.0f);
			distance->setDistanceJointFlag(physx::PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
			distance->setStiffness(NonNegative(j.distanceSpring));
			distance->setDamping(NonNegative(j.distanceDamping));
			distance->setDistanceJointFlag(physx::PxDistanceJointFlag::eSPRING_ENABLED,
				j.distanceSpring > 0.0f || j.distanceDamping > 0.0f);
			distance->setTolerance(std::max(NonNegative(j.distanceTolerance), 1e-4f));
		}

		// ---- bookkeeping ----

		void ReleaseNative(JointObj& j)
		{
			if (j.joint != nullptr)
			{
				j.joint->userData = nullptr;
				j.joint->release();
				j.joint = nullptr;
			}
		}

		void Unlink(JointObj& j)
		{
			for (BodyObj*& b : j.body)
			{
				if (b != nullptr)
				{
					b->joints.erase(std::remove(b->joints.begin(), b->joints.end(), &j), b->joints.end());
					b = nullptr;
				}
			}
			j.link[0] = nullptr;
			j.link[1] = nullptr;
		}

		// One side of a joint: nothing (world anchor), a body or an articulation link.
		struct Anchor
		{
			BodyObj* body = nullptr;
			LinkObj* link = nullptr;

			physx::PxRigidActor* actor() const
			{
				if (body != nullptr)
				{
					return body->actor;
				}
				return link != nullptr ? link->link : nullptr;
			}

			bool dynamic() const
			{
				return link != nullptr || (body != nullptr && !body->isStatic);
			}

			bool same(const Anchor& other) const
			{
				return (body != nullptr || link != nullptr) && body == other.body && link == other.link;
			}
		};

		LinkObj* LiveLink(const JointObj& j, int i)
		{
			return j.link[i] != nullptr ? static_cast<LinkObj*>(Lookup(j.link[i], Kind::Link)) : nullptr;
		}

		void Link(JointObj& j, const Anchor& a0, const Anchor& a1)
		{
			const Anchor* anchors[2] = { &a0, &a1 };
			for (int i = 0; i < 2; ++i)
			{
				j.body[i] = anchors[i]->body;
				j.link[i] = anchors[i]->link;  // LinkObj handle = its address
				if (j.body[i] != nullptr)
				{
					j.body[i]->joints.push_back(&j);
				}
			}
		}

		void DestroyJointLocked(Handle h, JointObj* j)
		{
			ReleaseNative(*j);
			Unlink(*j);
			if (j->world != nullptr)
			{
				auto& list = j->world->joints;
				list.erase(std::remove(list.begin(), list.end(), j), list.end());
			}
			Unregister(h);
			delete j;
		}

		// Resolves a body or articulation link handle of the given world. Null is allowed (world anchor); false when the
		// handle is foreign, a dead link or not an actor.
		bool ResolveActor(Handle h, const WorldObj* world, Anchor& out)
		{
			out = Anchor();
			if (h == nullptr)
			{
				return true;
			}
			if (auto* body = static_cast<BodyObj*>(Lookup(h, Kind::Body)))
			{
				if (body->world != world)
				{
					return false;
				}
				out.body = body;
				return true;
			}
			if (auto* link = static_cast<LinkObj*>(Lookup(h, Kind::Link)))
			{
				if (link->link == nullptr || link->dead || link->art == nullptr || link->art->world != world)
				{
					return false;
				}
				out.link = link;
				return true;
			}
			return false;
		}

		// PhysX needs two different actors, at least one of them dynamic.
		bool ValidActors(const Anchor& a0, const Anchor& a1)
		{
			return !a0.same(a1) && (a0.dynamic() || a1.dynamic());
		}

		physx::PxJoint* CreateNative(physx::PxPhysics& physics, JointType type, physx::PxRigidActor* a0, physx::PxRigidActor* a1)
		{
			const physx::PxTransform frame(physx::PxIdentity);
			switch (type)
			{
			case JointType::Fixed: return physx::PxFixedJointCreate(physics, a0, frame, a1, frame);
			case JointType::Prismatic: return physx::PxPrismaticJointCreate(physics, a0, frame, a1, frame);
			case JointType::Hinge: return physx::PxRevoluteJointCreate(physics, a0, frame, a1, frame);
			case JointType::BallAndSocket: return physx::PxSphericalJointCreate(physics, a0, frame, a1, frame);
			case JointType::Distance: return physx::PxDistanceJointCreate(physics, a0, frame, a1, frame);
			case JointType::Dof6: return physx::PxD6JointCreate(physics, a0, frame, a1, frame);
			default: return nullptr;
			}
		}

		// ---- world commands ----

		void CreateJoint(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::CreateJoint&>(c);
			cmd.outJoint = nullptr;

			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock)
			{
				return;
			}
			WorldObj* world = lock.get<WorldObj>();

			Anchor b0, b1;
			if (!ResolveActor(cmd.body0, world, b0) || !ResolveActor(cmd.body1, world, b1) || !ValidActors(b0, b1))
			{
				BACKEND_WARNF("World::CreateJoint: the joint needs two different bodies or articulation links of this world, at least one of them non-static");
				return;
			}

			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (physics == nullptr)
			{
				return;
			}
			physx::PxJoint* native = CreateNative(*physics, cmd.type, b0.actor(), b1.actor());
			if (native == nullptr)
			{
				BACKEND_WARNF("World::CreateJoint: unsupported joint type %d", static_cast<int>(cmd.type));
				return;
			}

			auto* joint = new JointObj();
			joint->joint = native;
			joint->world = world;
			joint->userData = cmd.userData;
			joint->type = cmd.type;
			joint->flags = cmd.flags;
			native->userData = joint;
			Link(*joint, b0, b1);
			world->joints.push_back(joint);

			ApplyFlags(*joint);
			ApplyDistance(*joint); // unconstrained until SetDistanceLimit (PhysX would pin the actors at distance 0)

			Register(joint, Kind::Joint, joint, world->state);
			cmd.outJoint = joint;
		}

		void DestroyJoint(Handle, const Context&, Command& c)
		{
			Lock lock(Target{ Kind::Joint, static_cast<wd::DestroyJoint&>(c).value, nullptr });
			if (lock)
			{
				DestroyJointLocked(lock.handle(), lock.get<JointObj>());
			}
		}

		// ---- joint commands ----

		// Locks the joint's world and hands the object to Fn; commands on dead handles produce zeroed output.
		template <typename Cmd, void (*Fn)(JointObj&, Cmd&)>
		void JointCmd(Handle sdkObject, const Context&, Command& c)
		{
			Lock lock(Target{ Kind::Joint, sdkObject, nullptr });
			if (!lock)
			{
				ZeroCommand<Cmd>(c);
				return;
			}
			Fn(*lock.get<JointObj>(), static_cast<Cmd&>(c));
		}

		void SetFlags(JointObj& j, jd::SetFlags& cmd)
		{
			using type_t = std::underlying_type_t<JointFlags>;
			const auto bits = static_cast<type_t>(cmd.value);
			const auto cur = static_cast<type_t>(j.flags);
			j.flags = static_cast<JointFlags>(cmd.enabled ? (cur | bits) : (cur & ~bits));
			ApplyFlags(j);
		}

		void GetFlags(JointObj& j, jd::GetFlags& cmd)
		{
			cmd.value = j.flags;
		}

		void GetUserData(JointObj& j, jd::GetUserData& cmd)
		{
			cmd.value = j.userData;
		}

		void SetActorLocalPose(JointObj& j, jd::SetActorLocalPose& cmd)
		{
			if (j.joint != nullptr)
			{
				j.joint->setLocalPose(PxActorIndex(cmd.target), ToPx(cmd.value));
			}
		}

		void GetActorLocalPose(JointObj& j, jd::GetActorLocalPose& cmd)
		{
			cmd.value = j.joint != nullptr ? FromPx(j.joint->getLocalPose(PxActorIndex(cmd.target))) : Physics::Pose();
		}

		void SetBreakForce(JointObj& j, jd::SetBreakForce& cmd)
		{
			if (j.joint != nullptr)
			{
				j.joint->setBreakForce(Unbounded(cmd.force), Unbounded(cmd.torque));
			}
		}

		void GetIsConstraintValid(JointObj& j, jd::GetIsConstraintValid& cmd)
		{
			physx::PxConstraint* constraint = j.joint != nullptr ? j.joint->getConstraint() : nullptr;
			cmd.value = constraint != nullptr && constraint->isValid()
				&& !(j.joint->getConstraintFlags() & physx::PxConstraintFlag::eBROKEN);
		}

		void GetForces(JointObj& j, jd::GetForces& cmd)
		{
			cmd.linear = Physics::Vec3();
			cmd.angular = Physics::Vec3();
			physx::PxConstraint* constraint = j.joint != nullptr ? j.joint->getConstraint() : nullptr;
			if (constraint != nullptr)
			{
				physx::PxVec3 linear(0.0f), angular(0.0f);
				constraint->getForce(linear, angular);
				cmd.linear = FromPx(linear);
				cmd.angular = FromPx(angular);
			}
		}

		void SetInvMassAndInertiaScale(JointObj& j, jd::SetInvMassAndInertiaScale& cmd)
		{
			if (j.joint == nullptr)
			{
				return;
			}
			const float scale = NonNegative(cmd.value);
			if (ActorIndex(cmd.target) == 0)
			{
				j.joint->setInvMassScale0(scale);
				j.joint->setInvInertiaScale0(scale);
			}
			else
			{
				j.joint->setInvMassScale1(scale);
				j.joint->setInvInertiaScale1(scale);
			}
		}

		void SetActors(JointObj& j, jd::SetActors& cmd)
		{
			Anchor b[2];
			if (j.joint == nullptr || !ResolveActor(cmd.actors[0].actor, j.world, b[0])
				|| !ResolveActor(cmd.actors[1].actor, j.world, b[1]) || !ValidActors(b[0], b[1]))
			{
				BACKEND_WARNF("Joint::SetActors: invalid actors");
				return;
			}
			j.joint->setActors(b[0].actor(), b[1].actor());
			Unlink(j);
			Link(j, b[0], b[1]);
		}

		void GetActors(JointObj& j, jd::GetActors& cmd)
		{
			for (int i = 0; i < 2; ++i)
			{
				const BodyObj* b = j.body[i];
				if (j.link[i] != nullptr)
				{
					const LinkObj* l = LiveLink(j, i);
					cmd.actors[i].actor = l != nullptr ? j.link[i] : nullptr;
					cmd.actors[i].userData = l != nullptr ? l->userData : nullptr;
					cmd.actors[i].type = l != nullptr ? Physics::ActorType::Link : Physics::ActorType::Invalid;
					continue;
				}
				cmd.actors[i].actor = const_cast<BodyObj*>(b);
				cmd.actors[i].userData = b != nullptr ? b->userData : nullptr;
				cmd.actors[i].type = b == nullptr ? Physics::ActorType::Invalid
					: (b->isStatic ? Physics::ActorType::Static : Physics::ActorType::Dynamic);
			}
		}

		void WakeUpActors(JointObj& j, jd::WakeUpActors&)
		{
			for (const BodyObj* b : j.body)
			{
				if (b != nullptr && b->dynamic != nullptr && b->inScene
					&& !(b->dynamic->getRigidBodyFlags() & physx::PxRigidBodyFlag::eKINEMATIC))
				{
					b->dynamic->wakeUp();
				}
			}
			for (int i = 0; i < 2; ++i)
			{
				const LinkObj* l = LiveLink(j, i);
				if (l != nullptr && l->link != nullptr && l->art != nullptr && l->art->inPxScene)
				{
					l->link->getArticulation().wakeUp();
				}
			}
		}

		void Set6DofLimit(JointObj& j, jd::Set6DofLimit& cmd)
		{
			const int index = static_cast<int>(cmd.axis);
			if (index < 0 || index >= 9)
			{
				return;
			}
			LimitCache& c = j.limits[index];
			c.set = true;
			c.min = cmd.min;
			c.max = cmd.max;
			c.bounciness = cmd.bounciness;
			c.bounceMinVelocity = cmd.bounceMinVelocity;
			c.contactDistance = cmd.contactDistance;
			ApplyLimit(j, cmd.axis);
		}

		void Get6DofLimit(JointObj& j, jd::Get6DofLimit& cmd)
		{
			const int index = static_cast<int>(cmd.axis);
			const LimitCache empty;
			const LimitCache& c = (index >= 0 && index < 9) ? j.limits[index] : empty;
			cmd.min = c.min;
			cmd.max = c.max;
			cmd.bounciness = c.bounciness;
			cmd.bounceMinVelocity = c.bounceMinVelocity;
			cmd.contactDistance = c.contactDistance;
		}

		void Set6DofSpringLimit(JointObj& j, jd::Set6DofSpringLimit& cmd)
		{
			const int index = static_cast<int>(cmd.axis);
			if (index < 0 || index >= 9)
			{
				return;
			}
			j.limits[index].spring = cmd.spring;
			j.limits[index].damping = cmd.damping;
			ApplyLimit(j, cmd.axis);
		}

		void Set6DofAxisLock(JointObj& j, jd::Set6DofAxisLock& cmd)
		{
			physx::PxD6Joint* d6 = AsD6(j);
			if (d6 == nullptr)
			{
				return;
			}
			const physx::PxD6Motion::Enum motion = ToPxMotion(cmd.value);
			switch (cmd.axis)
			{
			case ConstraintAxis::LinearX: d6->setMotion(physx::PxD6Axis::eX, motion); break;
			case ConstraintAxis::LinearY: d6->setMotion(physx::PxD6Axis::eY, motion); break;
			case ConstraintAxis::LinearZ: d6->setMotion(physx::PxD6Axis::eZ, motion); break;
			case ConstraintAxis::AngularX: d6->setMotion(physx::PxD6Axis::eTWIST, motion); break;
			case ConstraintAxis::AngularY: d6->setMotion(physx::PxD6Axis::eSWING1, motion); break;
			case ConstraintAxis::AngularZ: d6->setMotion(physx::PxD6Axis::eSWING2, motion); break;
			case ConstraintAxis::AngularYZ:
				d6->setMotion(physx::PxD6Axis::eSWING1, motion);
				d6->setMotion(physx::PxD6Axis::eSWING2, motion);
				break;
			default: break;
			}
		}

		void Set6DofAxisMotorConfiguration(JointObj& j, jd::Set6DofAxisMotorConfiguration& cmd)
		{
			physx::PxD6Joint* d6 = AsD6(j);
			physx::PxD6Drive::Enum index;
			if (d6 == nullptr || !DriveIndex(cmd.axis, index))
			{
				return;
			}
			d6->setDrive(index, physx::PxD6JointDrive(NonNegative(cmd.positionSpring), NonNegative(cmd.positionDamper),
				Unbounded(cmd.maximumForce), cmd.usesAcceleration != 0));
		}

		void Set6DofMotorTargetPosition(JointObj& j, jd::Set6DofMotorTargetPosition& cmd)
		{
			j.drivePosition = ToPx(cmd.value);
			ApplyDrivePosition(j);
		}

		void Set6DofMotorTargetRotation(JointObj& j, jd::Set6DofMotorTargetRotation& cmd)
		{
			physx::PxQuat q(cmd.value.x, cmd.value.y, cmd.value.z, cmd.value.w);
			j.driveRotation = (q.isFinite() && q.magnitude() > 1e-6f) ? q.getNormalized() : physx::PxQuat(physx::PxIdentity);
			ApplyDrivePosition(j);
		}

		void Set6DofMotorTargetLinearVelocity(JointObj& j, jd::Set6DofMotorTargetLinearVelocity& cmd)
		{
			j.driveLinearVelocity = ToPx(cmd.value);
			ApplyDriveVelocity(j);
		}

		void Set6DofMotorTargetAngularVelocity(JointObj& j, jd::Set6DofMotorTargetAngularVelocity& cmd)
		{
			j.driveAngularVelocity = ToPx(cmd.value);
			ApplyDriveVelocity(j);
		}

		void Set6DofProjectionTolerance(JointObj& j, jd::Set6DofProjectionTolerance& cmd)
		{
			if (j.joint != nullptr && compat::kJointProjectionSupported)
			{
				compat::SetJointProjectionTolerance(*j.joint, std::max(NonNegative(cmd.linear), 1e-4f), std::max(NonNegative(cmd.angular), 1e-4f));
			}
		}

		void SetDistanceLimit(JointObj& j, jd::SetDistanceLimit& cmd)
		{
			j.distanceMin = cmd.min;
			j.distanceMax = cmd.max;
			ApplyDistance(j);
		}

		void SetDistanceSpringLimit(JointObj& j, jd::SetDistanceSpringLimit& cmd)
		{
			j.distanceSpring = cmd.spring;
			j.distanceDamping = cmd.damping;
			ApplyDistance(j);
		}

		void SetDistanceErrorTolerance(JointObj& j, jd::SetDistanceErrorTolerance& cmd)
		{
			j.distanceTolerance = cmd.value;
			ApplyDistance(j);
		}

		void SetHingeLimitEnabled(JointObj& j, jd::SetHingeLimitEnabled& cmd)
		{
			j.hingeLimitEnabled = cmd.value;
			if (physx::PxRevoluteJoint* hinge = AsRevolute(j))
			{
				hinge->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED, cmd.value);
			}
		}

		void SetHingeMotorConfiguration(JointObj& j, jd::SetHingeMotorConfiguration& cmd)
		{
			j.hingeMaxForce = cmd.maxForce;
			j.hingeFreeSpin = cmd.freeSpin;
			ApplyHingeMotor(j);
		}

		void SetHingeMotorTargetVelocity(JointObj& j, jd::SetHingeMotorTargetVelocity& cmd)
		{
			j.hingeTargetVelocity = cmd.value;
			if (physx::PxRevoluteJoint* hinge = AsRevolute(j))
			{
				hinge->setDriveVelocity(Sane(cmd.value, 0.0f));
			}
		}

		void SetHingeLimit(JointObj& j, jd::SetHingeLimit& cmd)
		{
			LimitCache& c = j.hinge;
			c.set = true;
			c.min = cmd.min;
			c.max = cmd.max;
			c.bounciness = cmd.bounciness;
			c.bounceMinVelocity = cmd.bounceMinVelocity;
			c.contactDistance = cmd.contactDistance;
			ApplyHingeLimit(j);
		}
	}

	// ---- shared helpers ----

	Handle JointHandleFromJoint(const physx::PxJoint* joint)
	{
		Handle h = joint != nullptr ? joint->userData : nullptr;
		return IsKind(h, Kind::Joint) ? h : nullptr;
	}

	Handle JointHandleFromConstraint(const physx::PxConstraint* constraint)
	{
		if (constraint == nullptr)
		{
			return nullptr;
		}
		physx::PxU32 typeId = 0;
		void* external = const_cast<physx::PxConstraint*>(constraint)->getExternalReference(typeId);
		if (external == nullptr || typeId != physx::PxConstraintExtIDs::eJOINT)
		{
			return nullptr;
		}
		return JointHandleFromJoint(static_cast<const physx::PxJoint*>(external));
	}

	void DetachJointsFromBodyLocked(BodyObj* body)
	{
		const std::vector<JointObj*> joints = body->joints;
		for (JointObj* j : joints)
		{
			ReleaseNative(*j);
			Unlink(*j);
		}
		body->joints.clear();
	}

	void DetachJointsFromLinkLocked(LinkObj* link)
	{
		if (link == nullptr || link->art == nullptr || link->art->world == nullptr)
		{
			return;
		}
		for (JointObj* j : link->art->world->joints)
		{
			if (j->link[0] == link || j->link[1] == link)
			{
				ReleaseNative(*j);
				Unlink(*j);
			}
		}
	}

	void RebindJointsOfArticulationLocked(ArticulationObj& art)
	{
		if (art.world == nullptr)
		{
			return;
		}
		// PhysX nulls the actor of a joint when that actor is released, so the PxJoint (frames, limits, drives) survives the
		// rebuild of the articulation; it only has to be handed the new PxArticulationLink objects.
		for (JointObj* j : art.world->joints)
		{
			bool touches = false;
			for (int i = 0; i < 2; ++i)
			{
				const LinkObj* l = LiveLink(*j, i);
				touches = touches || (l != nullptr && l->art == &art);
			}
			if (!touches || j->joint == nullptr)
			{
				continue;
			}
			physx::PxRigidActor* actors[2] = { nullptr, nullptr };
			bool alive = true;
			for (int i = 0; i < 2; ++i)
			{
				if (j->link[i] != nullptr)
				{
					const LinkObj* l = LiveLink(*j, i);
					actors[i] = l != nullptr && !l->dead ? l->link : nullptr;
					alive = alive && actors[i] != nullptr;
				}
				else if (j->body[i] != nullptr)
				{
					actors[i] = j->body[i]->actor;
				}
			}
			if (alive)
			{
				j->joint->setActors(actors[0], actors[1]);
			}
			else
			{
				ReleaseNative(*j);
				Unlink(*j);
			}
		}
	}

	void DestroyJointsOfWorldLocked(WorldObj* world)
	{
		const std::vector<JointObj*> joints = world->joints;
		for (JointObj* j : joints)
		{
			DestroyJointLocked(j, j);
		}
		world->joints.clear();
	}

	void RegisterJointHandlers()
	{
		using W = pc::World;
		using J = pc::Joint;
		Bind(W::CreateJoint, &CreateJoint);
		Bind(W::DestroyJoint, &DestroyJoint);

		Bind(J::SetFlags, &JointCmd<jd::SetFlags, &SetFlags>);
		Bind(J::GetFlags, &JointCmd<jd::GetFlags, &GetFlags>);
		Bind(J::GetUserData, &JointCmd<jd::GetUserData, &GetUserData>);
		Bind(J::SetActorLocalPose, &JointCmd<jd::SetActorLocalPose, &SetActorLocalPose>);
		Bind(J::GetActorLocalPose, &JointCmd<jd::GetActorLocalPose, &GetActorLocalPose>);
		Bind(J::SetBreakForce, &JointCmd<jd::SetBreakForce, &SetBreakForce>);
		Bind(J::GetIsConstraintValid, &JointCmd<jd::GetIsConstraintValid, &GetIsConstraintValid>);
		Bind(J::GetForces, &JointCmd<jd::GetForces, &GetForces>);
		Bind(J::SetInvMassAndInertiaScale, &JointCmd<jd::SetInvMassAndInertiaScale, &SetInvMassAndInertiaScale>);
		Bind(J::SetActors, &JointCmd<jd::SetActors, &SetActors>);
		Bind(J::GetActors, &JointCmd<jd::GetActors, &GetActors>);
		Bind(J::WakeUpActors, &JointCmd<jd::WakeUpActors, &WakeUpActors>);
		Bind(J::Set6DofLimit, &JointCmd<jd::Set6DofLimit, &Set6DofLimit>);
		Bind(J::Get6DofLimit, &JointCmd<jd::Get6DofLimit, &Get6DofLimit>);
		Bind(J::Set6DofSpringLimit, &JointCmd<jd::Set6DofSpringLimit, &Set6DofSpringLimit>);
		Bind(J::Set6DofAxisLock, &JointCmd<jd::Set6DofAxisLock, &Set6DofAxisLock>);
		Bind(J::Set6DofAxisMotorConfiguration, &JointCmd<jd::Set6DofAxisMotorConfiguration, &Set6DofAxisMotorConfiguration>);
		Bind(J::Set6DofMotorTargetPosition, &JointCmd<jd::Set6DofMotorTargetPosition, &Set6DofMotorTargetPosition>);
		Bind(J::Set6DofMotorTargetRotation, &JointCmd<jd::Set6DofMotorTargetRotation, &Set6DofMotorTargetRotation>);
		Bind(J::Set6DofMotorTargetLinearVelocity, &JointCmd<jd::Set6DofMotorTargetLinearVelocity, &Set6DofMotorTargetLinearVelocity>);
		Bind(J::Set6DofMotorTargetAngularVelocity, &JointCmd<jd::Set6DofMotorTargetAngularVelocity, &Set6DofMotorTargetAngularVelocity>);
		Bind(J::Set6DofProjectionTolerance, &JointCmd<jd::Set6DofProjectionTolerance, &Set6DofProjectionTolerance>);
		Bind(J::SetDistanceLimit, &JointCmd<jd::SetDistanceLimit, &SetDistanceLimit>);
		Bind(J::SetDistanceSpringLimit, &JointCmd<jd::SetDistanceSpringLimit, &SetDistanceSpringLimit>);
		Bind(J::SetDistanceErrorTolerance, &JointCmd<jd::SetDistanceErrorTolerance, &SetDistanceErrorTolerance>);
		Bind(J::SetHingeMotorConfiguration, &JointCmd<jd::SetHingeMotorConfiguration, &SetHingeMotorConfiguration>);
		Bind(J::SetHingeMotorTargetVelocity, &JointCmd<jd::SetHingeMotorTargetVelocity, &SetHingeMotorTargetVelocity>);
		Bind(J::SetHingeLimit, &JointCmd<jd::SetHingeLimit, &SetHingeLimit>);
		Bind(J::SetHingeLimitEnabled, &JointCmd<jd::SetHingeLimitEnabled, &SetHingeLimitEnabled>);
	}
}
