#include "physx/common/PxWorld.h"

#include "core/Dispatch.h"
#include "physx/common/PhysXSdk.h"
#include "physx/common/PxCookingBackend.h"
#include "physx/common/PxDiag.h"
#include "physx/common/PxEvents.h"
#include "physx/common/PxMesh.h"
#include "physx/common/PxFilter.h"
#include "physx/common/PxJoint.h"
#include "physx/common/PxObjects.h"

#include "PhysicsWorldCommandData.h"

#include <algorithm>
#include <thread>

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace wd = PhysicsCommands::WorldData;

	namespace
	{
		uint32_t DispatcherThreadCount(uint32_t requested)
		{
			const uint32_t hw = std::max(1u, std::thread::hardware_concurrency());
			const uint32_t cap = std::min(hw, 8u);
			return std::min(std::max(requested, 1u), cap);
		}

		void CreateWorld(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<wd::CreateWorld&>(c);
			cmd.outWorld = nullptr;

			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (physics == nullptr)
			{
				BACKEND_ERRORF("World::CreateWorld called before the SDK was initialized");
				return;
			}

			physx::PxSceneDesc desc(physics->getTolerancesScale());
			desc.gravity = ToPx(cmd.gravity);
			desc.bounceThresholdVelocity = cmd.bounceThreshold;
			desc.filterShader = &SimulationFilterShader;
			desc.flags |= physx::PxSceneFlag::eENABLE_CCD;
			if (cmd.enableEnhancedDeterminism)
			{
				desc.flags |= physx::PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
			}
			if (cmd.fastMotionThreshold > 0.0f)
			{
				desc.ccdThreshold = cmd.fastMotionThreshold;
			}

			desc.solverType = cmd.solverType == Physics::SolverType::TemporalGaussSeidelSolver
				? physx::PxSolverType::eTGS : physx::PxSolverType::ePGS;

			switch (cmd.broadphaseType)
			{
			case Physics::BroadphaseType::SweepAndPruneBroadphase: desc.broadPhaseType = physx::PxBroadPhaseType::eSAP; break;
			case Physics::BroadphaseType::MultiBoxPruning: desc.broadPhaseType = physx::PxBroadPhaseType::eMBP; break;
			default: desc.broadPhaseType = physx::PxBroadPhaseType::eABP; break;
			}

			const int frictionKind = static_cast<int>(cmd.frictionType);
			if (!compat::ApplyFrictionType(desc, frictionKind))
			{
				BACKEND_WARNF("PhysX %d supports patch friction only; frictionType=%d ignored", compat::kPhysXMajor, frictionKind);
			}
			compat::ApplyAdaptiveForce(desc, cmd.enableAdaptiveForce);

			using Mode = Physics::ContactPairsMode;
			if ((cmd.contactPairsMode & Mode::EnableKinematicKinematicPairs) != Mode::DefaultContactPairs)
			{
				desc.kineKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
			}
			if ((cmd.contactPairsMode & Mode::EnableKinematicStaticPairs) != Mode::DefaultContactPairs)
			{
				desc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
			}

			auto* world = new WorldObj();
			world->dispatcher = physx::PxDefaultCpuDispatcherCreate(DispatcherThreadCount(cmd.jobWorkerCount));
			desc.cpuDispatcher = world->dispatcher;

			if (!desc.isValid())
			{
				BACKEND_ERRORF("World::CreateWorld produced an invalid PxSceneDesc");
				world->dispatcher->release();
				delete world;
				return;
			}

			world->scene = physics->createScene(desc);
			if (world->scene == nullptr)
			{
				BACKEND_ERRORF("PxPhysics::createScene failed");
				world->dispatcher->release();
				delete world;
				return;
			}

			if (desc.broadPhaseType == physx::PxBroadPhaseType::eMBP)
			{
				const uint32_t subdivisions = std::min(std::max(cmd.worldSubdivisions, 1u), 64u);
				std::vector<physx::PxBounds3> regions(static_cast<size_t>(subdivisions) * subdivisions);
				const physx::PxBounds3 bounds(ToPx(cmd.worldMin), ToPx(cmd.worldMax));
				const physx::PxU32 count = physx::PxBroadPhaseExt::createRegionsFromWorldBounds(regions.data(), bounds, subdivisions);
				for (physx::PxU32 i = 0; i < count; ++i)
				{
					world->scene->addBroadPhaseRegion(compat::MakeBroadPhaseRegion(regions[i]));
				}
			}

			world->state = std::make_shared<WorldState>();
			world->userData = cmd.userData;
			world->scene->userData = world;
			AttachWorldEvents(*world);
			Register(world, Kind::World, world, world->state);
			cmd.outWorld = world;
		}

		void DestroyWorld(Handle sdkObject, const Context& ctx, Command&)
		{
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock)
			{
				return;
			}
			DestroyWorldLocked(lock.handle(), lock.get<WorldObj>());
		}

		void SetGravity(Handle sdkObject, const Context& ctx, Command& c)
		{
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (lock)
			{
				lock.get<WorldObj>()->scene->setGravity(ToPx(static_cast<wd::SetGravity&>(c).value));
			}
		}

		void GetGravity(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetGravity&>(c);
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			cmd.value = lock ? FromPx(lock.get<WorldObj>()->scene->getGravity()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void SetBounceThreshold(Handle sdkObject, const Context& ctx, Command& c)
		{
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (lock)
			{
				lock.get<WorldObj>()->scene->setBounceThresholdVelocity(static_cast<wd::SetBounceThreshold&>(c).value);
			}
		}

		void GetBounceThreshold(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetBounceThreshold&>(c);
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			cmd.value = lock ? lock.get<WorldObj>()->scene->getBounceThresholdVelocity() : 0.0f;
		}

		void SetScratchBufferChunkCount(Handle sdkObject, const Context& ctx, Command& c)
		{
			// Stored only: Simulate() runs without a caller-provided scratch block.
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (lock)
			{
				lock.get<WorldObj>()->scratchChunkCount = static_cast<wd::SetScratchBufferChunkCount&>(c).value;
			}
		}

		void GetUserData(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetUserData&>(c);
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			cmd.userData = lock ? lock.get<WorldObj>()->userData : nullptr;
		}

		void GetBasicStats(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetBasicStats&>(c);
			cmd.value = Physics::WorldStats();
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (lock)
			{
				// Joints are still served by the stub, so only bodies and articulations are counted here.
				cmd.value.bodies = static_cast<uint32_t>(lock.get<WorldObj>()->bodies.size());
				cmd.value.articulations = static_cast<uint32_t>(lock.get<WorldObj>()->articulations.size());
			}
		}

		void Simulate(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::Simulate&>(c);
			cmd.ranSimulation = false;

			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock || !(cmd.deltaTime > 0.0f))
			{
				return;
			}

			physx::PxScene* scene = lock.get<WorldObj>()->scene;
			if (cmd.ignoreEmptyScene)
			{
				const physx::PxU32 actors = scene->getNbActors(
					physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC);
				if (actors == 0 && scene->getNbArticulations() == 0)
				{
					return;
				}
			}

			// jobGroupdId and expectedMaxDeltaTime are not needed: the step runs synchronously on the caller.
			scene->simulate(cmd.deltaTime);
			scene->fetchResults(true);
			FinishStepEvents(*lock.get<WorldObj>());
			DiagStep(*lock.get<WorldObj>());
			cmd.ranSimulation = true;
		}

		void CreateBody(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::CreateBody&>(c);
			cmd.outBody = nullptr;

			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (!lock || physics == nullptr)
			{
				return;
			}

			WorldObj* world = lock.get<WorldObj>();
			const physx::PxTransform pose = ToPx(cmd.pose);

			auto* body = new BodyObj();
			body->world = world;
			body->userData = cmd.userData;
			body->isStatic = cmd.isStatic;
			body->mass = cmd.mass;
			body->linearDamping = cmd.linearDamping;
			body->angularDamping = cmd.angularDamping;
			body->kinematicTarget = cmd.pose;

			if (cmd.isStatic)
			{
				body->actor = physics->createRigidStatic(pose);
			}
			else
			{
				body->dynamic = physics->createRigidDynamic(pose);
				body->actor = body->dynamic;
			}

			if (body->actor == nullptr)
			{
				BACKEND_ERRORF("PxPhysics::createRigid%s failed", cmd.isStatic ? "Static" : "Dynamic");
				delete body;
				return;
			}
			body->actor->userData = body;

			if (body->dynamic != nullptr)
			{
				body->dynamic->setLinearDamping(cmd.linearDamping);
				body->dynamic->setAngularDamping(cmd.angularDamping);
				if (cmd.mass > 0.0f)
				{
					body->dynamic->setMass(cmd.mass);
				}
				body->dynamic->setMaxLinearVelocity(body->maxLinearVelocity);
				body->dynamic->setMaxAngularVelocity(body->maxAngularVelocity);
				body->dynamic->setMaxDepenetrationVelocity(body->maxDepenetrationVelocity);
				body->dynamic->setSleepThreshold(body->sleepThreshold);
				body->dynamic->setSolverIterationCounts(body->positionIterations, body->velocityIterations);
			}
			ApplyBodyFlags(*body, cmd.flags);

			world->bodies.push_back(body);
			Register(body, Kind::Body, body, world->state);
			// Unity 6000.6.7f1 never sends AddActorsToScene for plain bodies, so they join the scene here.
			world->scene->addActor(*body->actor);
			body->inScene = true;
			cmd.outBody = body;
		}

		void AddActorsToScene(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::AddActorsToScene&>(c);
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock || cmd.actors == nullptr || cmd.bufferSize <= 0)
			{
				return;
			}
			WorldObj* world = lock.get<WorldObj>();

			auto add = [world](Handle h) {
				auto* body = static_cast<BodyObj*>(Lookup(h, Kind::Body));
				if (body == nullptr)
				{
					AddArticulationToSceneLocked(world, h);
					return;
				}
				if (body->world != world || body->inScene)
				{
					return;
				}
				world->scene->addActor(*body->actor);
				body->inScene = true;
			};

			// The header describes an array of handles; accept a single handle passed directly as well.
			if (IsKind(cmd.actors, Kind::Body) || IsKind(cmd.actors, Kind::Articulation))
			{
				add(cmd.actors);
				return;
			}
			const auto* handles = static_cast<const Handle*>(cmd.actors);
			for (int i = 0; i < cmd.bufferSize; ++i)
			{
				add(handles[i]);
			}
		}

		void DestroyBody(Handle sdkObject, const Context& ctx, Command& c)
		{
			Handle target = static_cast<wd::DestroyBody&>(c).value;
			Lock lock(Target{ Kind::Body, target, nullptr });
			if (lock)
			{
				DestroyBodyLocked(lock.handle(), lock.get<BodyObj>());
			}
			(void)sdkObject;
			(void)ctx;
		}

		void GetAllShapesByLayerMask(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetAllShapesByLayerMask&>(c);
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock || cmd.callback == nullptr)
			{
				return;
			}
			std::vector<void*> userData;
			for (const BodyObj* body : lock.get<WorldObj>()->bodies)
			{
				for (const ShapeObj* shape : body->shapes)
				{
					if ((shape->filter.belongsTo & cmd.layerMask) != 0)
					{
						userData.push_back(shape->userData);
					}
				}
			}
			CollectLinkShapesUserData(*lock.get<WorldObj>(), cmd.layerMask, userData);
			cmd.callback(userData.data(), userData.size());
		}
	}

	void RegisterWorldHandlers()
	{
		using W = pc::World;
		Bind(W::CreateWorld, &CreateWorld);
		Bind(W::DestroyWorld, &DestroyWorld);
		Bind(W::SetGravity, &SetGravity);
		Bind(W::GetGravity, &GetGravity);
		Bind(W::SetBounceThreshold, &SetBounceThreshold);
		Bind(W::GetBounceThreshold, &GetBounceThreshold);
		Bind(W::GetBasicStats, &GetBasicStats);
		Bind(W::SetScratchBufferChunkCount, &SetScratchBufferChunkCount);
		Bind(W::Simulate, &Simulate);
		Bind(W::GetUserData, &GetUserData);
		Bind(W::CreateBody, &CreateBody);
		Bind(W::AddActorsToScene, &AddActorsToScene);
		Bind(W::DestroyBody, &DestroyBody);
		Bind(W::GetAllShapesByLayerMask, &GetAllShapesByLayerMask);
	}
}

namespace backend
{
	void RegisterPhysXWorldHandlers()
	{
		pxw::RegisterSdkHandlers();
		pxw::RegisterWorldHandlers();
		pxw::RegisterBodyHandlers();
		pxw::RegisterShapeHandlers();
		pxw::RegisterQueryHandlers();
		pxw::RegisterEventHandlers();
		pxw::RegisterMeshHandlers();
		pxw::RegisterArticulationHandlers();
		pxw::RegisterJointHandlers();
	}

	void PhysXWorldOnSdkInitialized(uint16_t engineObjectIdOffset)
	{
		pxw::SetEngineObjectIdOffset(engineObjectIdOffset);
	}

	void ShutdownPhysXWorld()
	{
		using namespace pxw;
		for (Handle h : AllHandles(Kind::World))
		{
			Lock lock(Target{ Kind::World, h, nullptr });
			if (lock)
			{
				DestroyWorldLocked(lock.handle(), lock.get<WorldObj>());
			}
		}
		for (Handle h : AllHandles(Kind::Shape))
		{
			Lock lock(Target{ Kind::Shape, h, nullptr });
			if (lock)
			{
				DestroyShapeLocked(lock.handle(), lock.get<ShapeObj>());
			}
		}
		ReleaseAllMeshes();
		cooking::Shutdown();
		ReleaseAllMaterials();
		ClearIgnoredPairs();
	}
}
