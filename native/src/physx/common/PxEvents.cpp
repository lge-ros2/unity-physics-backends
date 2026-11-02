#include "physx/common/PxEvents.h"

#include "core/Dispatch.h"
#include "physx/common/PxFilter.h"
#include "physx/common/PxArticulation.h"
#include "physx/common/PxJoint.h"

#include "PhysicsEventTypes.h"
#include "PhysicsShapeCommandData.h"
#include "PhysicsWorldCommandData.h"

#include <algorithm>
#include <cstring>
#include <unordered_map>
#include <utility>
#include <vector>

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace wd = PhysicsCommands::WorldData;
	namespace sd = PhysicsCommands::ShapeData;

	// ShapePairEventFlags / ShapePairFlags / BodyPairFlags mirror PxPairFlag / PxContactPairFlag / PxContactPairHeaderFlag bit for bit.
	static_assert(static_cast<uint16_t>(Physics::ShapePairEventFlags::SolveContact) == physx::PxPairFlag::eSOLVE_CONTACT, "pair flag layout");
	static_assert(static_cast<uint16_t>(Physics::ShapePairEventFlags::NotifyTouchFound) == physx::PxPairFlag::eNOTIFY_TOUCH_FOUND, "pair flag layout");
	static_assert(static_cast<uint16_t>(Physics::ShapePairEventFlags::NotifyTouchLost) == physx::PxPairFlag::eNOTIFY_TOUCH_LOST, "pair flag layout");
	static_assert(static_cast<uint16_t>(Physics::ShapePairEventFlags::NotifyContactPoints) == physx::PxPairFlag::eNOTIFY_CONTACT_POINTS, "pair flag layout");
	static_assert(static_cast<uint16_t>(Physics::ShapePairEventFlags::PostSolvedVelocity) == physx::PxPairFlag::ePOST_SOLVER_VELOCITY, "pair flag layout");
	static_assert(static_cast<uint16_t>(Physics::ShapePairFlags::ActorPairLostTouch) == physx::PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH, "contact pair flag layout");
	static_assert(static_cast<uint16_t>(Physics::ShapePairFlags::InternalContactAreFlipped) == physx::PxContactPairFlag::eINTERNAL_CONTACTS_ARE_FLIPPED, "contact pair flag layout");
	static_assert(static_cast<uint16_t>(Physics::BodyPairFlags::RemovedOtherActor) == physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_1, "header flag layout");

	namespace
	{
		constexpr size_t kMaxPendingBodyPairs = 1u << 20;
		constexpr size_t kMaxPendingTriggers = 1u << 20;

		Physics::EngineObjectId ReadId(const void* userData)
		{
			Physics::EngineObjectId id;
			if (userData != nullptr)
			{
				std::memcpy(&id, static_cast<const uint8_t*>(userData) + EngineObjectIdOffset(), sizeof(id));
			}
			return id;
		}

		struct Ref
		{
			Handle handle = nullptr;
			Physics::EngineObjectId id;
		};

		// Result arrays of one step. Pointers inside the structs are linked once the vectors are final.
		struct Buffers
		{
			std::vector<Physics::BodyPair> bodyPairs;
			std::vector<size_t> bodyFirst; // index of the first shape pair of each body pair
			std::vector<Physics::ShapePair> shapePairs;
			std::vector<size_t> shapeFirst; // index of the first contact point of each shape pair
			std::vector<Physics::ContactPoint> points;
			std::vector<Physics::TriggerEvent> triggers;
			std::vector<Physics::JointBreakEvent> jointBreaks;

			bool Empty() const
			{
				return bodyPairs.empty() && triggers.empty() && jointBreaks.empty();
			}

			void Link()
			{
				for (size_t i = 0; i < bodyPairs.size(); ++i)
				{
					bodyPairs[i].startPtr = bodyPairs[i].nbPairs > 0 ? shapePairs.data() + bodyFirst[i] : nullptr;
				}
				for (size_t i = 0; i < shapePairs.size(); ++i)
				{
					shapePairs[i].startPtr = shapePairs[i].nbPoints > 0 ? points.data() + shapeFirst[i] : nullptr;
				}
			}
		};

		Physics::Vec3 ToVec3(const physx::PxVec3& v)
		{
			return Physics::Vec3(v.x, v.y, v.z);
		}
	}

	struct WorldEvents final : public physx::PxSimulationEventCallback
	{
		// Objects removed since the last fetchResults, keyed by the PhysX pointer that PhysX reports for them.
		std::unordered_map<const void*, Ref> removedShapes;
		std::unordered_map<const void*, Ref> removedActors;

		Buffers pending;	// collected by the callbacks, handed out by the next GetLastSimulationStepResults
		Buffers published;	// storage behind the pointers of the last GetLastSimulationStepResults
		bool warnedOverflow = false;

		// A removed object is only identified through the removal tables; a live one through its userData.
		Ref ResolveShape(const physx::PxShape* shape, bool removed) const
		{
			Ref ref;
			if (shape == nullptr)
			{
				return ref;
			}
			if (removed)
			{
				const auto it = removedShapes.find(shape);
				if (it != removedShapes.end())
				{
					ref = it->second;
				}
				return ref;
			}
			void* obj = shape->userData;
			if (obj != nullptr && Lookup(obj, Kind::Shape) != nullptr)
			{
				ref.handle = obj;
				ref.id = ReadId(static_cast<const ShapeObj*>(obj)->userData);
			}
			return ref;
		}

		Ref ResolveActor(const physx::PxActor* actor, bool removed) const
		{
			Ref ref;
			if (actor == nullptr)
			{
				return ref;
			}
			if (removed)
			{
				const auto it = removedActors.find(actor);
				if (it != removedActors.end())
				{
					ref = it->second;
				}
				return ref;
			}
			void* obj = actor->userData;
			if (obj != nullptr && Lookup(obj, Kind::Body) != nullptr)
			{
				ref.handle = obj;
				ref.id = ReadId(static_cast<const BodyObj*>(obj)->userData);
			}
			else if (obj != nullptr && IsKind(obj, Kind::Link))
			{
				ref.handle = obj;
				ref.id = ReadId(LinkUserData(obj));
			}
			return ref;
		}

		void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override
		{
			for (physx::PxU32 i = 0; i < count; ++i)
			{
				const Handle handle = JointHandleFromConstraint(constraints[i].constraint);
				if (handle == nullptr)
				{
					continue;
				}
				Physics::JointBreakEvent event;
				event.jointID = ReadId(static_cast<const JointObj*>(handle)->userData);
				physx::PxVec3 force(0.0f), torque(0.0f);
				constraints[i].constraint->getForce(force, torque);
				event.linearMagnitude = force.magnitude();
				event.angularMagnitude = torque.magnitude();
				pending.jointBreaks.push_back(event);
			}
		}

		void onWake(physx::PxActor**, physx::PxU32) override {}
		void onSleep(physx::PxActor**, physx::PxU32) override {}
		void onAdvance(const physx::PxRigidBody* const*, const physx::PxTransform*, const physx::PxU32) override {}

		void onContact(const physx::PxContactPairHeader& header, const physx::PxContactPair* pairs, physx::PxU32 nbPairs) override
		{
			if (pending.bodyPairs.size() >= kMaxPendingBodyPairs)
			{
				if (!warnedOverflow)
				{
					warnedOverflow = true;
					BACKEND_WARNF("Contact events are not fetched (GetLastSimulationStepResults); dropping new ones");
				}
				return;
			}

			const bool removed0 = (static_cast<uint16_t>(header.flags) & physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_0) != 0;
			const bool removed1 = (static_cast<uint16_t>(header.flags) & physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_1) != 0;

			Physics::BodyPair body{};
			body.thisBodyID = ResolveActor(header.actors[0], removed0).id;
			body.otherBodyID = ResolveActor(header.actors[1], removed1).id;
			body.flags = static_cast<Physics::BodyPairFlags>(static_cast<uint16_t>(header.flags));

			// Pre-solver velocities come with ePRE_SOLVER_VELOCITY; without them fall back to the current state.
			bool haveVelocity = false;
			if (header.extraDataStream != nullptr && header.extraDataStreamSize > 0)
			{
				physx::PxContactPairExtraDataIterator it(header.extraDataStream, header.extraDataStreamSize);
				while (it.nextItemSet())
				{
					if (it.preSolverVelocity != nullptr)
					{
						body.thisBodyLinearVelocity = ToVec3(it.preSolverVelocity->linearVelocity[0]);
						body.thisBodyAngularVelocity = ToVec3(it.preSolverVelocity->angularVelocity[0]);
						body.otherBodyLinearVelocity = ToVec3(it.preSolverVelocity->linearVelocity[1]);
						body.otherBodyAngularVelocity = ToVec3(it.preSolverVelocity->angularVelocity[1]);
						haveVelocity = true;
						break;
					}
				}
			}
			if (!haveVelocity)
			{
				const physx::PxActor* actors[2] = { header.actors[0], header.actors[1] };
				const bool removed[2] = { removed0, removed1 };
				Physics::Vec3* lin[2] = { &body.thisBodyLinearVelocity, &body.otherBodyLinearVelocity };
				Physics::Vec3* ang[2] = { &body.thisBodyAngularVelocity, &body.otherBodyAngularVelocity };
				for (int i = 0; i < 2; ++i)
				{
					const auto* rb = (!removed[i] && actors[i] != nullptr) ? actors[i]->is<physx::PxRigidBody>() : nullptr;
					*lin[i] = rb != nullptr ? ToVec3(rb->getLinearVelocity()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
					*ang[i] = rb != nullptr ? ToVec3(rb->getAngularVelocity()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
				}
			}

			body.nbPairs = 0;
			pending.bodyFirst.push_back(pending.shapePairs.size());

			std::vector<physx::PxContactPairPoint> scratch;
			for (physx::PxU32 i = 0; i < nbPairs; ++i)
			{
				const physx::PxContactPair& pair = pairs[i];
				const uint16_t pairFlags = static_cast<uint16_t>(pair.flags);
				const bool shapeRemoved0 = (pairFlags & physx::PxContactPairFlag::eREMOVED_SHAPE_0) != 0;
				const bool shapeRemoved1 = (pairFlags & physx::PxContactPairFlag::eREMOVED_SHAPE_1) != 0;

				Physics::ShapePair shape{};
				shape.thisColliderID = ResolveShape(pair.shapes[0], shapeRemoved0).id;
				shape.otherColliderID = ResolveShape(pair.shapes[1], shapeRemoved1).id;
				shape.flags = static_cast<Physics::ShapePairFlags>(pairFlags);
				shape.events = static_cast<Physics::ShapePairEventFlags>(static_cast<uint16_t>(pair.events));
				shape.impulseSum = Physics::Vec3(0.0f, 0.0f, 0.0f);

				pending.shapeFirst.push_back(pending.points.size());
				if (pair.contactCount > 0)
				{
					scratch.resize(pair.contactCount);
					const physx::PxU32 got = pair.extractContacts(scratch.data(), pair.contactCount);
					for (physx::PxU32 p = 0; p < got; ++p)
					{
						const physx::PxContactPairPoint& src = scratch[p];
						Physics::ContactPoint point{};
						point.position = ToVec3(src.position);
						point.separation = src.separation;
						point.normal = ToVec3(src.normal);
						point.faceIndex0 = src.internalFaceIndex0;
						point.impulse = ToVec3(src.impulse);
						point.faceIndex1 = src.internalFaceIndex1;
						shape.impulseSum.x += src.impulse.x;
						shape.impulseSum.y += src.impulse.y;
						shape.impulseSum.z += src.impulse.z;
						pending.points.push_back(point);
					}
					shape.nbPoints = got;
				}
				pending.shapePairs.push_back(shape);
				++body.nbPairs;
			}
			pending.bodyPairs.push_back(body);
		}

		void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override
		{
			for (physx::PxU32 i = 0; i < count; ++i)
			{
				const physx::PxTriggerPair& pair = pairs[i];
				const uint8_t flags = static_cast<uint8_t>(pair.flags);
				const bool triggerRemoved = (flags & physx::PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER) != 0;
				const bool otherRemoved = (flags & physx::PxTriggerPairFlag::eREMOVED_SHAPE_OTHER) != 0;

				// Trigger vs trigger does not produce events.
				if (!otherRemoved && pair.otherShape != nullptr && (pair.otherShape->getFlags() & physx::PxShapeFlag::eTRIGGER_SHAPE))
				{
					continue;
				}

				// A pair that ends because a shape was removed is reported as Exit so the engine never keeps it forever.
				Physics::TriggerEventType type;
				if (triggerRemoved || otherRemoved || pair.status == physx::PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					type = Physics::TriggerEventType::Exit;
				}
				else if (pair.status == physx::PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					type = Physics::TriggerEventType::Enter;
				}
				else
				{
					continue;
				}

				if (pending.triggers.size() >= kMaxPendingTriggers)
				{
					return;
				}
				const Ref trigger = ResolveShape(pair.triggerShape, triggerRemoved);
				const Ref other = ResolveShape(pair.otherShape, otherRemoved);
				pending.triggers.emplace_back(type, trigger.handle, other.handle, trigger.id, other.id);
			}
		}
	};

	// ---- world / removal hooks ----

	void AttachWorldEvents(WorldObj& world)
	{
		if (world.events == nullptr && world.scene != nullptr)
		{
			world.events = new WorldEvents();
			world.scene->setSimulationEventCallback(world.events);
		}
	}

	void DetachWorldEvents(WorldObj& world)
	{
		if (world.events == nullptr)
		{
			return;
		}
		if (world.scene != nullptr)
		{
			world.scene->setSimulationEventCallback(nullptr);
		}
		delete world.events;
		world.events = nullptr;
	}

	void FinishStepEvents(WorldObj& world)
	{
		if (world.events != nullptr)
		{
			world.events->removedShapes.clear();
			world.events->removedActors.clear();
		}
	}

	namespace
	{
		void CaptureShape(WorldEvents& events, const ShapeObj& shape)
		{
			events.removedShapes[shape.shape] = Ref{ const_cast<ShapeObj*>(&shape), ReadId(shape.userData) };
		}
	}

	void NotifyShapeRemoved(const ShapeObj& shape)
	{
		if (shape.ownerLink != nullptr)
		{
			const ArticulationObj* art = shape.ownerLink->art;
			if (art != nullptr && art->added && art->world != nullptr && art->world->events != nullptr)
			{
				CaptureShape(*art->world->events, shape);
			}
			return;
		}
		const BodyObj* owner = shape.owner;
		if (owner == nullptr || !owner->inScene || owner->world == nullptr || owner->world->events == nullptr)
		{
			return;
		}
		CaptureShape(*owner->world->events, shape);
	}

	void NotifyBodyRemoved(const BodyObj& body)
	{
		if (!body.inScene || body.world == nullptr || body.world->events == nullptr)
		{
			return;
		}
		WorldEvents& events = *body.world->events;
		events.removedActors[body.actor] = Ref{ const_cast<BodyObj*>(&body), ReadId(body.userData) };
		for (const ShapeObj* shape : body.shapes)
		{
			CaptureShape(events, *shape);
		}
	}

	void NotifyPxShapeRemoved(WorldObj& world, const physx::PxShape* shape, Handle handle, const void* engineUserData)
	{
		if (world.events != nullptr && shape != nullptr)
		{
			world.events->removedShapes[shape] = Ref{ handle, ReadId(engineUserData) };
		}
	}

	void NotifyPxActorRemoved(WorldObj& world, const physx::PxActor* actor, Handle handle, const void* engineUserData)
	{
		if (world.events != nullptr && actor != nullptr)
		{
			world.events->removedActors[actor] = Ref{ handle, ReadId(engineUserData) };
		}
	}

	void ReportJointBreak(WorldObj& world, const Physics::JointBreakEvent& event)
	{
		if (world.events != nullptr)
		{
			world.events->pending.jointBreaks.push_back(event);
		}
	}

	// ---- handlers ----

	namespace
	{
		void GetLastSimulationStepResults(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetLastSimulationStepResults&>(c);
			cmd.bodyContactPairs = nullptr;
			cmd.bodyContactPairsCount = 0;
			cmd.triggerEvents = nullptr;
			cmd.triggerEventsCount = 0;
			cmd.jointBreakEvents = nullptr;
			cmd.jointBreakEventsCount = 0;

			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock)
			{
				return;
			}
			WorldEvents* events = lock.get<WorldObj>()->events;
			if (events == nullptr)
			{
				return;
			}

			// Results not released by the caller are replaced; their pointers become invalid here.
			events->published = std::move(events->pending);
			events->pending = Buffers();
			Buffers& out = events->published;
			out.Link();

			cmd.bodyContactPairs = out.bodyPairs.empty() ? nullptr : out.bodyPairs.data();
			cmd.bodyContactPairsCount = out.bodyPairs.size();
			cmd.triggerEvents = out.triggers.empty() ? nullptr : out.triggers.data();
			cmd.triggerEventsCount = out.triggers.size();
			cmd.jointBreakEvents = out.jointBreaks.empty() ? nullptr : out.jointBreaks.data();
			cmd.jointBreakEventsCount = out.jointBreaks.size();
		}

		void ReleaseLastSimulationStepData(Handle sdkObject, const Context& ctx, Command&)
		{
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (lock && lock.get<WorldObj>()->events != nullptr)
			{
				lock.get<WorldObj>()->events->published = Buffers();
			}
		}

		void GetLastSimulationStats(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetLastSimulationStepStats&>(c);
			cmd.value = Physics::SimulationStepStats();
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock)
			{
				return;
			}
			physx::PxSimulationStatistics stats;
			lock.get<WorldObj>()->scene->getSimulationStatistics(stats);

			// The pair matrices are symmetric: sum each geometry combination once.
			auto sumPairs = [&stats](physx::PxSimulationStatistics::RbPairStatsType type) {
				uint32_t total = 0;
				for (int g0 = 0; g0 < physx::PxGeometryType::eGEOMETRY_COUNT; ++g0)
				{
					for (int g1 = g0; g1 < physx::PxGeometryType::eGEOMETRY_COUNT; ++g1)
					{
						total += stats.getRbPairStats(type, static_cast<physx::PxGeometryType::Enum>(g0), static_cast<physx::PxGeometryType::Enum>(g1));
					}
				}
				return total;
			};

			Physics::SimulationStepStats& out = cmd.value;
			out.dynamicBodies = stats.nbDynamicBodies;
			out.activeDynamicBodies = stats.nbActiveDynamicBodies;
			out.kinematicBodies = stats.nbKinematicBodies;
			out.activeKinematicBodies = stats.nbActiveKinematicBodies;
			out.staticBodies = stats.nbStaticBodies;
			out.articulations = stats.nbArticulations;
			out.constraints = stats.nbActiveConstraints;
			out.broadPhaseAdds = stats.getNbBroadPhaseAdds();
			out.broadPhaseRemoves = stats.getNbBroadPhaseRemoves();
			out.newTouches = stats.nbNewTouches;
			out.lostTouches = stats.nbLostTouches;
			out.contactPairs = stats.nbDiscreteContactPairsTotal;
			out.CCDContactPairs = sumPairs(physx::PxSimulationStatistics::eCCD_PAIRS);
			out.modifiableContactPairs = sumPairs(physx::PxSimulationStatistics::eMODIFIED_CONTACT_PAIRS);
			out.triggerPairs = sumPairs(physx::PxSimulationStatistics::eTRIGGER_PAIRS);
		}

		void SetSupportedMessages(Handle sdkObject, const Context& ctx, Command& c)
		{
			const auto& cmd = static_cast<sd::SetSupportedMessages&>(c);
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (!lock)
			{
				return;
			}
			ShapeObj* shape = lock.get<ShapeObj>();
			const uint16_t flags = static_cast<uint16_t>(cmd.flags);
			if (cmd.layer >= 0 && cmd.layer < 32)
			{
				shape->supportedByLayer[cmd.layer] = flags;
			}
			else
			{
				std::fill(std::begin(shape->supportedByLayer), std::end(shape->supportedByLayer), flags);
			}
			UpdateShapeSimulationFilter(*shape);
		}

		void SetModifiableContacts(Handle, const Context&, Command&)
		{
			// Contact modification is not supported; the flag is accepted and ignored.
		}
	}

	void RegisterEventHandlers()
	{
		Bind(pc::World::GetLastSimulationStepResults, &GetLastSimulationStepResults);
		Bind(pc::World::ReleaseLastSimulationStepData, &ReleaseLastSimulationStepData);
		Bind(pc::World::GetLastSimulationStats, &GetLastSimulationStats);
		Bind(pc::Shape::SetSupportedMessages, &SetSupportedMessages);
		Bind(pc::Shape::SetModifiableContacts, &SetModifiableContacts);
	}
}
