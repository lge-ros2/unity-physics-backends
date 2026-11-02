#include "core/Dispatch.h"
#include "physx/common/PxObjects.h"

#include "PhysicsBodyCommandData.h"

#include <algorithm>
#include <cmath>

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace bd = PhysicsCommands::BodyData;

	using Physics::BodyFlags;
	using Physics::CollisionDetectionMode;

	namespace
	{
		// Locks the world that owns the body addressed by a Body:: command.
		class BodyScope
		{
		public:
			BodyScope(Handle sdkObject, const Context& ctx)
				: m_lock(Target{ Kind::Body, sdkObject, ctx.body })
			{
			}

			explicit operator bool() const { return static_cast<bool>(m_lock); }
			BodyObj* operator->() const { return m_lock.get<BodyObj>(); }
			BodyObj& operator*() const { return *m_lock.get<BodyObj>(); }

		private:
			Lock m_lock;
		};

		bool IsKinematic(const BodyObj& b)
		{
			return b.dynamic != nullptr && (b.dynamic->getRigidBodyFlags() & physx::PxRigidBodyFlag::eKINEMATIC);
		}

		void ApplyCcd(BodyObj& b)
		{
			if (b.dynamic == nullptr)
			{
				return;
			}
			const bool kinematic = IsKinematic(b);
			const bool ccd = !kinematic
				&& (b.ccdMode == CollisionDetectionMode::Continuous || b.ccdMode == CollisionDetectionMode::ContinuousDynamic);
			const bool speculative = !kinematic && b.ccdMode == CollisionDetectionMode::ContinuousSpeculative;

			const physx::PxRigidBodyFlags flags = b.dynamic->getRigidBodyFlags();
			const bool hasCcd = flags & physx::PxRigidBodyFlag::eENABLE_CCD;
			const bool hasSpeculative = flags & physx::PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD;

			// Clear first: PhysX rejects both CCD flavours being raised at once.
			if (hasCcd && !ccd)
			{
				b.dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_CCD, false);
			}
			if (hasSpeculative && !speculative)
			{
				b.dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, false);
			}
			if (ccd && !hasCcd)
			{
				b.dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_CCD, true);
			}
			if (speculative && !hasSpeculative)
			{
				b.dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);
			}
		}

		physx::PxForceMode::Enum ToPxForceMode(Physics::ForceMode mode)
		{
			switch (mode)
			{
			case Physics::ForceMode::Impulse: return physx::PxForceMode::eIMPULSE;
			case Physics::ForceMode::VelocityChange: return physx::PxForceMode::eVELOCITY_CHANGE;
			case Physics::ForceMode::Acceleration: return physx::PxForceMode::eACCELERATION;
			default: return physx::PxForceMode::eFORCE;
			}
		}

		// ---- flags / mass / damping / pose ----

		void SetFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::SetFlags&>(c);
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				const BodyFlags next = cmd.enabled ? (b->flags | cmd.value) : (b->flags & ~cmd.value);
				ApplyBodyFlags(*b, next);
			}
		}

		void GetFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetFlags&>(c).value = b ? b->flags : BodyFlags::None;
		}

		void SetMass(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = static_cast<bd::SetMass&>(c).value;
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->mass = value;
				if (b->dynamic != nullptr && value > 0.0f)
				{
					b->dynamic->setMass(value);
				}
			}
		}

		void GetMass(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetMass&>(c).value = !b ? 0.0f : (b->dynamic != nullptr ? b->dynamic->getMass() : b->mass);
		}

		void SetDamping(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::SetDamping&>(c);
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->linearDamping = cmd.linear;
				b->angularDamping = cmd.angular;
				if (b->dynamic != nullptr)
				{
					b->dynamic->setLinearDamping(std::max(cmd.linear, 0.0f));
					b->dynamic->setAngularDamping(std::max(cmd.angular, 0.0f));
				}
			}
		}

		void GetDamping(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::GetDamping&>(c);
			BodyScope b(sdkObject, ctx);
			cmd.linear = !b ? 0.0f : (b->dynamic != nullptr ? b->dynamic->getLinearDamping() : b->linearDamping);
			cmd.angular = !b ? 0.0f : (b->dynamic != nullptr ? b->dynamic->getAngularDamping() : b->angularDamping);
		}

		void SetPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::SetPose&>(c);
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->actor->setGlobalPose(ToPx(cmd.value), cmd.awake);
			}
		}

		void GetPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetPose&>(c).value = b ? FromPx(b->actor->getGlobalPose()) : Physics::Pose();
		}

		// ---- shapes ----

		void AttachShape(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::AttachShape&>(c);
			Lock lock(Target{ Kind::Body, sdkObject, ctx.body }, Target{ Kind::Shape, cmd.shape });
			if (!lock)
			{
				return;
			}
			auto* body = lock.get<BodyObj>(0);
			auto* shape = lock.get<ShapeObj>(1);

			// The pose is used as the shape's pose relative to the actor regardless of body type (forceLocalPose).
			shape->shape->setLocalPose(ToPx(cmd.pose));
			if (shape->owner == body)
			{
				return;
			}
			if (shape->owner != nullptr)
			{
				BodyObj* old = shape->owner;
				old->actor->detachShape(*shape->shape);
				old->shapes.erase(std::remove(old->shapes.begin(), old->shapes.end(), shape), old->shapes.end());
				shape->owner = nullptr;
			}

			if (!body->actor->attachShape(*shape->shape))
			{
				BACKEND_ERRORF("PxRigidActor::attachShape failed");
				Rebind(shape, LooseState());
				return;
			}
			body->shapes.push_back(shape);
			shape->owner = body;
			Rebind(shape, body->world->state);
		}

		void DetachShape(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::DetachShape&>(c);
			Lock lock(Target{ Kind::Body, sdkObject, ctx.body }, Target{ Kind::Shape, cmd.shape });
			if (!lock)
			{
				return;
			}
			auto* body = lock.get<BodyObj>(0);
			auto* shape = lock.get<ShapeObj>(1);
			if (shape->owner != body)
			{
				return;
			}
			body->actor->detachShape(*shape->shape);
			body->shapes.erase(std::remove(body->shapes.begin(), body->shapes.end(), shape), body->shapes.end());
			shape->owner = nullptr;
			Rebind(shape, LooseState());
		}

		void GetShapeCount(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetShapeCount&>(c).value = b ? b->shapes.size() : 0;
		}

		void GetShapes(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::GetShapes&>(c);
			cmd.written = 0;
			BodyScope b(sdkObject, ctx);
			if (b && cmd.buffer != nullptr)
			{
				for (ShapeObj* shape : b->shapes)
				{
					if (cmd.written >= cmd.bufferSize)
					{
						break;
					}
					cmd.buffer[cmd.written++] = shape;
				}
			}
		}

		void GetShapesUserData(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::GetShapesUserData&>(c);
			cmd.written = 0;
			BodyScope b(sdkObject, ctx);
			if (b && cmd.buffer != nullptr)
			{
				for (const ShapeObj* shape : b->shapes)
				{
					if (cmd.written >= cmd.bufferSize)
					{
						break;
					}
					cmd.buffer[cmd.written++] = shape->userData;
				}
			}
		}

		void GetUserData(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetUserData&>(c).value = b ? b->userData : nullptr;
		}

		void SetCollisionDetectionMode(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->ccdMode = static_cast<bd::SetCollisionDetectionMode&>(c).value;
				ApplyCcd(*b);
			}
		}

		// ---- forces / velocities ----

		void AddForce(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::AddForce&>(c);
			BodyScope b(sdkObject, ctx);
			// PhysX only accepts forces on non-kinematic bodies that live in a scene.
			if (b && b->dynamic != nullptr && b->inScene && !IsKinematic(*b))
			{
				b->dynamic->addForce(ToPx(cmd.value), ToPxForceMode(cmd.mode), cmd.awake);
			}
		}

		void AddTorque(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::AddTorque&>(c);
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr && b->inScene && !IsKinematic(*b))
			{
				b->dynamic->addTorque(ToPx(cmd.value), ToPxForceMode(cmd.mode), cmd.awake);
			}
		}

		void SetLinearVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::SetLinearVelocity&>(c);
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr && !IsKinematic(*b))
			{
				b->dynamic->setLinearVelocity(ToPx(cmd.value), cmd.awake);
			}
		}

		void GetLinearVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetLinearVelocity&>(c).value =
				(b && b->dynamic != nullptr) ? FromPx(b->dynamic->getLinearVelocity()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void SetAngularVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::SetAngularVelocity&>(c);
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr && !IsKinematic(*b))
			{
				b->dynamic->setAngularVelocity(ToPx(cmd.value), cmd.awake);
			}
		}

		void GetAngularVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetAngularVelocity&>(c).value =
				(b && b->dynamic != nullptr) ? FromPx(b->dynamic->getAngularVelocity()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void SetMaxLinearVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = std::max(static_cast<bd::SetMaxLinearVelocity&>(c).value, 0.0f);
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->maxLinearVelocity = value;
				if (b->dynamic != nullptr)
				{
					b->dynamic->setMaxLinearVelocity(value);
				}
			}
		}

		void GetMaxLinearVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetMaxLinearVelocity&>(c).value =
				!b ? 0.0f : (b->dynamic != nullptr ? b->dynamic->getMaxLinearVelocity() : b->maxLinearVelocity);
		}

		void SetMaxAngularVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = std::max(static_cast<bd::SetMaxAngularVelocity&>(c).value, 0.0f);
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->maxAngularVelocity = value;
				if (b->dynamic != nullptr)
				{
					b->dynamic->setMaxAngularVelocity(value);
				}
			}
		}

		void GetMaxAngularVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetMaxAngularVelocity&>(c).value =
				!b ? 0.0f : (b->dynamic != nullptr ? b->dynamic->getMaxAngularVelocity() : b->maxAngularVelocity);
		}

		void SetMaxDepenetrationVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = std::max(static_cast<bd::SetMaxDepenetrationVelocity&>(c).value, 0.0f);
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->maxDepenetrationVelocity = value;
				if (b->dynamic != nullptr)
				{
					b->dynamic->setMaxDepenetrationVelocity(value);
				}
			}
		}

		void GetMaxDepenetrationVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetMaxDepenetrationVelocity&>(c).value =
				!b ? 0.0f : (b->dynamic != nullptr ? b->dynamic->getMaxDepenetrationVelocity() : b->maxDepenetrationVelocity);
		}

		// ---- mass properties ----

		void SetInertiaTensor(Handle sdkObject, const Context& ctx, Command& c)
		{
			const physx::PxVec3 v = ToPx(static_cast<bd::SetInertiaTensor&>(c).value);
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr)
			{
				b->dynamic->setMassSpaceInertiaTensor(physx::PxVec3(std::max(v.x, 0.0f), std::max(v.y, 0.0f), std::max(v.z, 0.0f)));
			}
		}

		void GetInertiaTensor(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetInertiaTensor&>(c).value =
				(b && b->dynamic != nullptr) ? FromPx(b->dynamic->getMassSpaceInertiaTensor()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void SetInertiaTensorRotation(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Physics::Vec4 q = static_cast<bd::SetInertiaTensorRotation&>(c).value;
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr)
			{
				physx::PxTransform t = b->dynamic->getCMassLocalPose();
				t.q = ToPx(Physics::Pose(0, 0, 0, q.x, q.y, q.z, q.w)).q;
				b->dynamic->setCMassLocalPose(t);
			}
		}

		void GetInertiaTensorRotation(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			auto& out = static_cast<bd::GetInertiaTensorRotation&>(c).value;
			if (b && b->dynamic != nullptr)
			{
				const physx::PxQuat q = b->dynamic->getCMassLocalPose().q;
				out = Physics::Vec4(q.x, q.y, q.z, q.w);
			}
			else
			{
				out = Physics::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
			}
		}

		void SetLocalCenterOfMass(Handle sdkObject, const Context& ctx, Command& c)
		{
			const physx::PxVec3 v = ToPx(static_cast<bd::SetLocalCenterOfMass&>(c).value);
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr)
			{
				physx::PxTransform t = b->dynamic->getCMassLocalPose();
				t.p = v;
				b->dynamic->setCMassLocalPose(t);
			}
		}

		void GetLocalCenterOfMass(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetLocalCenterOfMass&>(c).value =
				(b && b->dynamic != nullptr) ? FromPx(b->dynamic->getCMassLocalPose().p) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void GetWorldInertiaTensorMatrix(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::GetWorldInertiaTensorMatrix&>(c);
			std::fill(std::begin(cmd.value), std::end(cmd.value), 0.0f);
			BodyScope b(sdkObject, ctx);
			if (!b || b->dynamic == nullptr)
			{
				return;
			}
			const physx::PxTransform cm = b->dynamic->getGlobalPose() * b->dynamic->getCMassLocalPose();
			const physx::PxMat33 r(cm.q);
			const physx::PxMat33 w = r * physx::PxMat33::createDiagonal(b->dynamic->getMassSpaceInertiaTensor()) * r.getTranspose();
			for (int row = 0; row < 3; ++row)
			{
				for (int col = 0; col < 3; ++col)
				{
					cmd.value[row * 3 + col] = w[col][row];
				}
			}
		}

		void RecomputeMassProperties(Handle sdkObject, const Context& ctx, Command& c)
		{
			const auto& overrides = static_cast<bd::RecomputeMassProperties&>(c).overrides;
			BodyScope b(sdkObject, ctx);
			if (!b || b->dynamic == nullptr)
			{
				return;
			}

			size_t simShapes = 0;
			for (const ShapeObj* shape : b->shapes)
			{
				if ((shape->flags & Physics::ShapeFlags::Trigger) == Physics::ShapeFlags::None)
				{
					++simShapes;
				}
			}
			if (simShapes > 0 && b->mass > 0.0f)
			{
				// Keeps the requested mass and derives the inertia from the attached shapes (uniform density).
				physx::PxRigidBodyExt::setMassAndUpdateInertia(*b->dynamic, b->mass);
			}

			physx::PxTransform cm = b->dynamic->getCMassLocalPose();
			bool changed = false;
			if (overrides.overrideCenterOfMass)
			{
				cm.p = ToPx(overrides.centerOfMass);
				changed = true;
			}
			if (overrides.overrideInertiaTensor)
			{
				cm.q = ToPx(Physics::Pose(0, 0, 0, overrides.inertiaTensorRotation.x, overrides.inertiaTensorRotation.y,
					overrides.inertiaTensorRotation.z, overrides.inertiaTensorRotation.w)).q;
				changed = true;
			}
			if (changed)
			{
				b->dynamic->setCMassLocalPose(cm);
			}
			if (overrides.overrideInertiaTensor)
			{
				const physx::PxVec3 v = ToPx(overrides.inertiaTensor);
				b->dynamic->setMassSpaceInertiaTensor(physx::PxVec3(std::max(v.x, 0.0f), std::max(v.y, 0.0f), std::max(v.z, 0.0f)));
			}
		}

		void GetLocalPointVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::GetLocalPointVelocity&>(c);
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr)
			{
				const physx::PxVec3 world = b->dynamic->getGlobalPose().transform(ToPx(cmd.localPoint));
				cmd.value = FromPx(physx::PxRigidBodyExt::getVelocityAtPos(*b->dynamic, world));
			}
			else
			{
				cmd.value = Physics::Vec3(0.0f, 0.0f, 0.0f);
			}
		}

		void GetWorldPointVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::GetWorldPointVelocity&>(c);
			BodyScope b(sdkObject, ctx);
			cmd.value = (b && b->dynamic != nullptr)
				? FromPx(physx::PxRigidBodyExt::getVelocityAtPos(*b->dynamic, ToPx(cmd.worldPoint)))
				: Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void GetClosestWorldPointOnBounds(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::GetClosestWorldPointOnBounds&>(c);
			cmd.outClosestWorldPositionOnBounds = cmd.worldPosition;
			cmd.outSqrDistance = 0.0f;
			BodyScope b(sdkObject, ctx);
			if (!b || b->shapes.empty())
			{
				return;
			}
			const physx::PxBounds3 bounds = b->actor->getWorldBounds(1.0f);
			if (bounds.isEmpty())
			{
				return;
			}
			const physx::PxVec3 p = ToPx(cmd.worldPosition);
			const physx::PxVec3 closest = bounds.minimum.maximum(p.minimum(bounds.maximum));
			cmd.outClosestWorldPositionOnBounds = FromPx(closest);
			cmd.outSqrDistance = (closest - p).magnitudeSquared();
		}

		// ---- kinematic / sleep / solver ----

		void SetKinematicTarget(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Physics::Pose& pose = static_cast<bd::SetKinematicTarget&>(c).value;
			BodyScope b(sdkObject, ctx);
			if (!b || b->dynamic == nullptr || !IsKinematic(*b))
			{
				return;
			}
			b->kinematicTarget = pose;
			if (b->inScene)
			{
				b->dynamic->setKinematicTarget(ToPx(pose));
			}
			else
			{
				b->dynamic->setGlobalPose(ToPx(pose));
			}
		}

		void GetKinematicTarget(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			auto& out = static_cast<bd::GetKinematicTarget&>(c).value;
			if (!b)
			{
				out = Physics::Pose();
				return;
			}
			physx::PxTransform target;
			if (b->dynamic != nullptr && b->inScene && IsKinematic(*b) && b->dynamic->getKinematicTarget(target))
			{
				out = FromPx(target);
			}
			else
			{
				out = FromPx(b->actor->getGlobalPose());
			}
		}

		void GetIsSleeping(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetIsSleeping&>(c).value =
				b && b->dynamic != nullptr && b->inScene && !IsKinematic(*b) && b->dynamic->isSleeping();
		}

		void Sleep(Handle sdkObject, const Context& ctx, Command&)
		{
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr && b->inScene && !IsKinematic(*b))
			{
				b->dynamic->putToSleep();
			}
		}

		void WakeUp(Handle sdkObject, const Context& ctx, Command&)
		{
			BodyScope b(sdkObject, ctx);
			if (b && b->dynamic != nullptr && b->inScene && !IsKinematic(*b))
			{
				b->dynamic->wakeUp();
			}
		}

		void SetSleepThresholdOverride(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = std::max(static_cast<bd::SetSleepThresholdOverride&>(c).value, 0.0f);
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->sleepThreshold = value;
				if (b->dynamic != nullptr)
				{
					b->dynamic->setSleepThreshold(value);
				}
			}
		}

		void GetSleepThresholdOverride(Handle sdkObject, const Context& ctx, Command& c)
		{
			BodyScope b(sdkObject, ctx);
			static_cast<bd::GetSleepThresholdOverride&>(c).value =
				!b ? 0.0f : (b->dynamic != nullptr ? b->dynamic->getSleepThreshold() : b->sleepThreshold);
		}

		void SetSolverIterationsOverride(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::SetSolverIterationsOverride&>(c);
			BodyScope b(sdkObject, ctx);
			if (b)
			{
				b->positionIterations = std::min(std::max(cmd.positionIterations, 1u), 255u);
				b->velocityIterations = std::min(std::max(cmd.velocityIterations, 1u), 255u);
				if (b->dynamic != nullptr)
				{
					b->dynamic->setSolverIterationCounts(b->positionIterations, b->velocityIterations);
				}
			}
		}

		void GetSolverIterationsOverride(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<bd::GetSolverIterationsOverride&>(c);
			cmd.positionIterations = 0;
			cmd.velocityIterations = 0;
			BodyScope b(sdkObject, ctx);
			if (!b)
			{
				return;
			}
			if (b->dynamic != nullptr)
			{
				physx::PxU32 pos = 0;
				physx::PxU32 vel = 0;
				b->dynamic->getSolverIterationCounts(pos, vel);
				cmd.positionIterations = pos;
				cmd.velocityIterations = vel;
			}
			else
			{
				cmd.positionIterations = b->positionIterations;
				cmd.velocityIterations = b->velocityIterations;
			}
		}
	}

	void ApplyBodyFlags(BodyObj& b, BodyFlags flags)
	{
		b.flags = flags;
		b.actor->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, (flags & BodyFlags::Gravity) == BodyFlags::None);
		if (b.dynamic == nullptr)
		{
			// Static actors always take part in the simulation.
			return;
		}
		b.actor->setActorFlag(physx::PxActorFlag::eDISABLE_SIMULATION, (flags & BodyFlags::Simulated) == BodyFlags::None);

		const bool wantKinematic = (flags & BodyFlags::Kinematic) != BodyFlags::None;
		if (wantKinematic != IsKinematic(b))
		{
			b.dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, wantKinematic);
		}
		ApplyCcd(b);
	}

	void RegisterBodyHandlers()
	{
		using B = pc::Body;
		Bind(B::SetFlags, &SetFlags);
		Bind(B::GetFlags, &GetFlags);
		Bind(B::SetMass, &SetMass);
		Bind(B::GetMass, &GetMass);
		Bind(B::SetDamping, &SetDamping);
		Bind(B::GetDamping, &GetDamping);
		Bind(B::SetPose, &SetPose);
		Bind(B::GetPose, &GetPose);
		Bind(B::AttachShape, &AttachShape);
		Bind(B::DetachShape, &DetachShape);
		Bind(B::GetShapeCount, &GetShapeCount);
		Bind(B::GetShapes, &GetShapes);
		Bind(B::GetShapesUserData, &GetShapesUserData);
		Bind(B::GetUserData, &GetUserData);
		Bind(B::SetCollisionDetectionMode, &SetCollisionDetectionMode);
		Bind(B::AddForce, &AddForce);
		Bind(B::AddTorque, &AddTorque);
		Bind(B::SetLinearVelocity, &SetLinearVelocity);
		Bind(B::GetLinearVelocity, &GetLinearVelocity);
		Bind(B::SetAngularVelocity, &SetAngularVelocity);
		Bind(B::GetAngularVelocity, &GetAngularVelocity);
		Bind(B::SetMaxLinearVelocity, &SetMaxLinearVelocity);
		Bind(B::GetMaxLinearVelocity, &GetMaxLinearVelocity);
		Bind(B::SetMaxAngularVelocity, &SetMaxAngularVelocity);
		Bind(B::GetMaxAngularVelocity, &GetMaxAngularVelocity);
		Bind(B::SetMaxDepenetrationVelocity, &SetMaxDepenetrationVelocity);
		Bind(B::GetMaxDepenetrationVelocity, &GetMaxDepenetrationVelocity);
		Bind(B::SetInertiaTensor, &SetInertiaTensor);
		Bind(B::GetInertiaTensor, &GetInertiaTensor);
		Bind(B::SetInertiaTensorRotation, &SetInertiaTensorRotation);
		Bind(B::GetInertiaTensorRotation, &GetInertiaTensorRotation);
		Bind(B::SetLocalCenterOfMass, &SetLocalCenterOfMass);
		Bind(B::GetLocalCenterOfMass, &GetLocalCenterOfMass);
		Bind(B::GetWorldInertiaTensorMatrix, &GetWorldInertiaTensorMatrix);
		Bind(B::RecomputeMassProperties, &RecomputeMassProperties);
		Bind(B::GetLocalPointVelocity, &GetLocalPointVelocity);
		Bind(B::GetWorldPointVelocity, &GetWorldPointVelocity);
		Bind(B::GetClosestWorldPointOnBounds, &GetClosestWorldPointOnBounds);
		Bind(B::SetKinematicTarget, &SetKinematicTarget);
		Bind(B::GetKinematicTarget, &GetKinematicTarget);
		Bind(B::GetIsSleeping, &GetIsSleeping);
		Bind(B::Sleep, &Sleep);
		Bind(B::WakeUp, &WakeUp);
		Bind(B::SetSleepThresholdOverride, &SetSleepThresholdOverride);
		Bind(B::GetSleepThresholdOverride, &GetSleepThresholdOverride);
		Bind(B::SetSolverIterationsOverride, &SetSolverIterationsOverride);
		Bind(B::GetSolverIterationsOverride, &GetSolverIterationsOverride);
	}
}
