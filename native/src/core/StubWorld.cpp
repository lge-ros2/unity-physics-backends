#include "StubWorld.h"
#include "Dispatch.h"

#include "PhysicsGeneralCommandData.h"
#include "PhysicsMaterialCommandData.h"
#include "PhysicsWorldCommandData.h"
#include "PhysicsBodyCommandData.h"
#include "PhysicsShapeCommandData.h"
#include "PhysicsQueryCommandData.h"
#include "PhysicsJointCommandData.h"
#include "PhysicsArticulationCommandData.h"

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <mutex>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace backend
{
	namespace pc = PhysicsCommands;
	namespace sdkd = PhysicsCommands::SdkData;
	namespace matd = PhysicsCommands::MaterialData;
	namespace wd = PhysicsCommands::WorldData;
	namespace bd = PhysicsCommands::BodyData;
	namespace sd = PhysicsCommands::ShapeData;
	namespace qd = PhysicsCommands::QueryData;
	namespace jd = PhysicsCommands::JointData;
	namespace ajd = PhysicsCommands::ArticulationJointData;
	namespace ald = PhysicsCommands::ArticulationLinkData;
	namespace ad = PhysicsCommands::ArticulationData;

	using Handle = Physics::SDKObjectHandle;
	using Context = pc::Context;
	using Command = pc::Command;

	namespace
	{
		enum class Kind : uint8_t
		{
			World,
			Body,
			Articulation,
			Link,
			LinkJoint,
			Joint,
			Shape,
			Material,
			Mesh,
			HeightField
		};

		// Bookkeeping for one dummy handle. Handles are never dereferenced by the stub;
		// the record only keeps what is needed to answer simple read-back commands.
		struct Record
		{
			Kind kind = Kind::Body;
			void* userData = nullptr;
			Handle world = nullptr;
			Handle owner = nullptr;  // shape -> body/link, link -> articulation
			Handle parent = nullptr; // link -> parent link
			Handle joint = nullptr;  // link -> its link joint
			Handle actors[2] = { nullptr, nullptr };
			Handle material = nullptr;

			Physics::Pose pose;
			Physics::Pose localPose;
			float mass = 1.0f;
			float linearDamping = 0.0f;
			float angularDamping = 0.05f;
			bool isStatic = false;

			Physics::BodyFlags bodyFlags = Physics::BodyFlags::None;
			Physics::ArticulationFlags articulationFlags = Physics::ArticulationFlags::None;
			Physics::JointFlags jointFlags = Physics::JointFlags::None;
			Physics::ShapeFlags shapeFlags = Physics::ShapeFlags::None;
			Physics::FilterData filter;
			Physics::ShapeGeometry geometry = {};

			Physics::Vec3 gravity = { 0.0f, -9.81f, 0.0f };
			float bounceThreshold = 1.0f;

			std::vector<Handle> shapes;   // body/link
			std::vector<Handle> children; // link -> child links
			std::vector<Handle> links;    // articulation -> links (creation order)
		};

		std::mutex g_mutex;
		std::unordered_map<Handle, Record> g_records;
		uintptr_t g_nextId = 0x10000;
		Handle g_defaultMaterial = nullptr;
		std::set<std::pair<uint16_t, uint16_t>> g_ignoredPairs;

		// ---- handle table helpers (g_mutex must be held) ----

		Record* Allocate(Kind kind, void* userData, Handle world, Handle& outHandle)
		{
			g_nextId += 16;
			outHandle = reinterpret_cast<Handle>(g_nextId);
			Record& r = g_records[outHandle];
			r.kind = kind;
			r.userData = userData;
			r.world = world;
			return &r;
		}

		Record* Find(Handle h, Kind kind)
		{
			if (h == nullptr)
			{
				return nullptr;
			}
			const auto it = g_records.find(h);
			if (it == g_records.end() || it->second.kind != kind)
			{
				return nullptr;
			}
			return &it->second;
		}

		// The target handle of a command may arrive as ctx.body, sdkObject or ctx.world
		// depending on the category; accept the first one that matches the expected kind.
		Record* Resolve(const Context& ctx, Handle sdkObject, Kind kind, Handle* outHandle = nullptr)
		{
			for (Handle h : { ctx.body, sdkObject, ctx.world })
			{
				if (Record* r = Find(h, kind))
				{
					if (outHandle != nullptr)
					{
						*outHandle = h;
					}
					return r;
				}
			}
			return nullptr;
		}

		void RemoveFrom(std::vector<Handle>& list, Handle h)
		{
			list.erase(std::remove(list.begin(), list.end(), h), list.end());
		}

		void Release(Handle h, Kind kind)
		{
			Record* r = Find(h, kind);
			if (r == nullptr)
			{
				return;
			}

			if (kind == Kind::Body || kind == Kind::Link)
			{
				for (Handle s : r->shapes)
				{
					if (Record* shape = Find(s, Kind::Shape))
					{
						shape->owner = nullptr;
					}
				}
			}
			else if (kind == Kind::Shape)
			{
				if (Record* owner = Find(r->owner, Kind::Body))
				{
					RemoveFrom(owner->shapes, h);
				}
				else if (Record* ownerLink = Find(r->owner, Kind::Link))
				{
					RemoveFrom(ownerLink->shapes, h);
				}
			}

			if (kind == Kind::Link)
			{
				if (Record* art = Find(r->owner, Kind::Articulation))
				{
					RemoveFrom(art->links, h);
				}
				if (Record* parent = Find(r->parent, Kind::Link))
				{
					RemoveFrom(parent->children, h);
				}
				if (r->joint != nullptr)
				{
					g_records.erase(r->joint);
				}
			}

			g_records.erase(h);
		}

		void ReleaseResolved(const Context& ctx, Handle sdkObject, Kind kind)
		{
			Handle h = nullptr;
			if (Resolve(ctx, sdkObject, kind, &h) != nullptr)
			{
				Release(h, kind);
			}
		}

		// ---- generic handler templates ----

		template <typename T>
		struct MemberClass;
		template <typename C, typename M>
		struct MemberClass<M C::*>
		{
			using Class = C;
			using Type = M;
		};

		template <typename T>
		void ZeroOut(Handle, const Context&, Command& c)
		{
			static_cast<T&>(c) = T{};
		}

		// Copies cmd.*CmdField into the record of the resolved handle.
		template <Kind K, auto CmdField, auto RecField>
		void StoreField(Handle sdkObject, const Context& ctx, Command& c)
		{
			using Cmd = typename MemberClass<decltype(CmdField)>::Class;
			auto& cmd = static_cast<Cmd&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* r = Resolve(ctx, sdkObject, K))
			{
				r->*RecField = cmd.*CmdField;
			}
		}

		// Copies the record field into cmd.*CmdField, or a value-initialized default when unknown.
		template <Kind K, auto CmdField, auto RecField>
		void LoadField(Handle sdkObject, const Context& ctx, Command& c)
		{
			using Cmd = typename MemberClass<decltype(CmdField)>::Class;
			using Value = typename MemberClass<decltype(CmdField)>::Type;
			auto& cmd = static_cast<Cmd&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* r = Resolve(ctx, sdkObject, K))
			{
				cmd.*CmdField = r->*RecField;
			}
			else
			{
				cmd.*CmdField = Value{};
			}
		}

		template <Kind K, typename T>
		void SetBodyFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<T&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* r = Resolve(ctx, sdkObject, K))
			{
				r->bodyFlags = cmd.enabled ? (r->bodyFlags | cmd.value) : (r->bodyFlags & ~cmd.value);
			}
		}

		template <Kind K, typename T>
		void SetDampingH(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<T&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* r = Resolve(ctx, sdkObject, K))
			{
				r->linearDamping = cmd.linear;
				r->angularDamping = cmd.angular;
			}
		}

		template <Kind K, typename T>
		void GetDampingH(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<T&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* r = Resolve(ctx, sdkObject, K))
			{
				cmd.linear = r->linearDamping;
				cmd.angular = r->angularDamping;
			}
			else
			{
				cmd.linear = 0.0f;
				cmd.angular = 0.0f;
			}
		}

		template <Kind K, typename T>
		void AttachShapeH(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<T&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle self = nullptr;
			Record* r = Resolve(ctx, sdkObject, K, &self);
			Record* shape = Find(cmd.shape, Kind::Shape);
			if (r == nullptr || shape == nullptr)
			{
				return;
			}
			if (std::find(r->shapes.begin(), r->shapes.end(), cmd.shape) == r->shapes.end())
			{
				r->shapes.push_back(cmd.shape);
			}
			shape->owner = self;
			shape->localPose = cmd.pose;
		}

		template <Kind K, typename T>
		void DetachShapeH(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<T&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* r = Resolve(ctx, sdkObject, K))
			{
				RemoveFrom(r->shapes, cmd.shape);
			}
			if (Record* shape = Find(cmd.shape, Kind::Shape))
			{
				shape->owner = nullptr;
			}
		}

		template <Kind K, typename T>
		void GetShapeCountH(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<T&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Record* r = Resolve(ctx, sdkObject, K);
			cmd.value = r != nullptr ? r->shapes.size() : 0;
		}

		template <Kind K, typename T>
		void GetShapesH(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<T&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			cmd.written = 0;
			if (Record* r = Resolve(ctx, sdkObject, K))
			{
				for (Handle s : r->shapes)
				{
					if (cmd.buffer == nullptr || cmd.written >= cmd.bufferSize)
					{
						break;
					}
					cmd.buffer[cmd.written++] = s;
				}
			}
		}

		template <Kind K, typename T>
		void GetShapesUserDataH(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<T&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			cmd.written = 0;
			if (Record* r = Resolve(ctx, sdkObject, K))
			{
				for (Handle s : r->shapes)
				{
					if (cmd.buffer == nullptr || cmd.written >= cmd.bufferSize)
					{
						break;
					}
					const Record* shape = Find(s, Kind::Shape);
					cmd.buffer[cmd.written++] = shape != nullptr ? shape->userData : nullptr;
				}
			}
		}

		template <typename T, int Mode>
		void ConstantFloat(Handle, const Context&, Command& c)
		{
			// 0: max linear velocity, 1: max angular velocity, 2: max depenetration velocity
			static_cast<T&>(c).value = Mode == 0 ? 100.0f : (Mode == 1 ? 7.0f : 10.0f);
		}

		template <typename T>
		void IdentityQuat(Handle, const Context&, Command& c)
		{
			static_cast<T&>(c).value = Physics::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
		}

		template <typename E>
		void BindNoops(std::initializer_list<E> ids)
		{
			for (E id : ids)
			{
				Bind(id, &NoopHandler);
			}
		}

		// ---- SDK ----

		void ShutdownSdkHandler(Handle, const Context&, Command&)
		{
			ResetStubWorld();
		}

		void CreateMaterialHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<matd::CreateMaterial&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle h = nullptr;
			Allocate(Kind::Material, cmd.userData, nullptr, h);
			cmd.outMaterial = h;
		}

		void ReleaseMaterialHandler(Handle sdkObject, const Context& ctx, Command&)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle h = nullptr;
			if (Resolve(ctx, sdkObject, Kind::Material, &h) != nullptr && h != g_defaultMaterial)
			{
				Release(h, Kind::Material);
			}
		}

		void GetDefaultMaterialHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<matd::GetDefaultMaterial&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (g_defaultMaterial == nullptr)
			{
				Allocate(Kind::Material, nullptr, nullptr, g_defaultMaterial);
			}
			cmd.value = g_defaultMaterial;
		}

		void GetMaterialConfigurationHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<matd::GetMaterialConfiguration&>(c);
			cmd.dynamicFriction = 0.6f;
			cmd.staticFriction = 0.6f;
			cmd.bounciness = 0.0f;
		}

		void GetMaterialCombineModesHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<matd::GetMaterialCombineModes&>(c);
			cmd.friction = Physics::MaterialCombineMode::Average;
			cmd.bounciness = Physics::MaterialCombineMode::Average;
		}

		// ---- World ----

		void CreateWorldHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<wd::CreateWorld&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle h = nullptr;
			Record* r = Allocate(Kind::World, cmd.userData, nullptr, h);
			r->world = h;
			r->gravity = cmd.gravity;
			r->bounceThreshold = cmd.bounceThreshold;
			cmd.outWorld = h;
		}

		void DestroyWorldHandler(Handle sdkObject, const Context& ctx, Command&)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle world = nullptr;
			if (Resolve(ctx, sdkObject, Kind::World, &world) == nullptr)
			{
				return;
			}
			for (auto it = g_records.begin(); it != g_records.end();)
			{
				if (it->second.world == world)
				{
					it = g_records.erase(it);
				}
				else
				{
					++it;
				}
			}
		}

		void GetBasicStatsHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetBasicStats&>(c);
			cmd.value = Physics::WorldStats();
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle world = nullptr;
			if (Resolve(ctx, sdkObject, Kind::World, &world) == nullptr)
			{
				return;
			}
			for (const auto& kv : g_records)
			{
				if (kv.second.world != world)
				{
					continue;
				}
				switch (kv.second.kind)
				{
				case Kind::Body: ++cmd.value.bodies; break;
				case Kind::Articulation: ++cmd.value.articulations; break;
				case Kind::Joint: ++cmd.value.constraints; break;
				default: break;
				}
			}
		}

		void SimulateHandler(Handle, const Context&, Command& c)
		{
			// Nothing is simulated, so report that the step was skipped.
			static_cast<wd::Simulate&>(c).ranSimulation = false;
		}

		void GetLastSimulationStepResultsHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<wd::GetLastSimulationStepResults&>(c);
			cmd.bodyContactPairs = nullptr;
			cmd.bodyContactPairsCount = 0;
			cmd.triggerEvents = nullptr;
			cmd.triggerEventsCount = 0;
			cmd.jointBreakEvents = nullptr;
			cmd.jointBreakEventsCount = 0;
		}

		void CreateBodyHandler(Handle, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::CreateBody&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle h = nullptr;
			Record* r = Allocate(Kind::Body, cmd.userData, ctx.world, h);
			r->pose = cmd.pose;
			r->mass = cmd.mass;
			r->linearDamping = cmd.linearDamping;
			r->angularDamping = cmd.angularDamping;
			r->bodyFlags = cmd.flags;
			r->isStatic = cmd.isStatic;
			cmd.outBody = h;
		}

		void DestroyBodyHandler(Handle, const Context&, Command& c)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			Release(static_cast<wd::DestroyBody&>(c).value, Kind::Body);
		}

		void CreateArticulationHandler(Handle, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::CreateArticulation&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle h = nullptr;
			Record* r = Allocate(Kind::Articulation, cmd.userData, ctx.world, h);
			r->articulationFlags = cmd.flags;
			cmd.outArticulation = h;
		}

		void DestroyArticulationHandler(Handle, const Context&, Command& c)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			const Handle h = static_cast<wd::DestoryArticulation&>(c).value;
			if (Record* r = Find(h, Kind::Articulation))
			{
				const std::vector<Handle> links = r->links;
				for (Handle link : links)
				{
					Release(link, Kind::Link);
				}
				g_records.erase(h);
			}
		}

		void CreateJointHandler(Handle, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::CreateJoint&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle h = nullptr;
			Record* r = Allocate(Kind::Joint, cmd.userData, ctx.world, h);
			r->actors[0] = cmd.body0;
			r->actors[1] = cmd.body1;
			r->jointFlags = cmd.flags;
			cmd.outJoint = h;
		}

		void DestroyJointHandler(Handle, const Context&, Command& c)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			Release(static_cast<wd::DestroyJoint&>(c).value, Kind::Joint);
		}

		void GetArticulationCountHandler(Handle, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetArticulationCount&>(c);
			cmd.value = 0;
			std::lock_guard<std::mutex> lock(g_mutex);
			for (const auto& kv : g_records)
			{
				if (kv.second.kind == Kind::Articulation && (ctx.world == nullptr || kv.second.world == ctx.world))
				{
					++cmd.value;
				}
			}
		}

		void GetArticulationsHandler(Handle, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetArticulations&>(c);
			cmd.written = 0;
			std::lock_guard<std::mutex> lock(g_mutex);
			for (const auto& kv : g_records)
			{
				if (cmd.buffer == nullptr || cmd.written >= cmd.bufferSize)
				{
					break;
				}
				if (kv.second.kind == Kind::Articulation && (ctx.world == nullptr || kv.second.world == ctx.world))
				{
					cmd.buffer[cmd.written++] = kv.first;
				}
			}
		}

		// ---- Shape ----

		void CreateShapeHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CreateShape&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle h = nullptr;
			Record* r = Allocate(Kind::Shape, cmd.userData, nullptr, h);
			r->geometry = cmd.geom;
			r->material = cmd.materialPtr;
			r->shapeFlags = cmd.flags;
			cmd.value = h;
		}

		void DestroyShapeHandler(Handle sdkObject, const Context& ctx, Command&)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			ReleaseResolved(ctx, sdkObject, Kind::Shape);
		}

		void GetActorDescriptorHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<sd::GetActorDescriptor&>(c);
			cmd.value = Physics::ActorDescriptor();
			std::lock_guard<std::mutex> lock(g_mutex);
			Record* shape = Resolve(ctx, sdkObject, Kind::Shape);
			if (shape == nullptr || shape->owner == nullptr)
			{
				return;
			}
			if (const Record* owner = Find(shape->owner, Kind::Body))
			{
				cmd.value.actor = shape->owner;
				cmd.value.userData = owner->userData;
				cmd.value.type = owner->isStatic ? Physics::ActorType::Static : Physics::ActorType::Dynamic;
			}
			else if (const Record* link = Find(shape->owner, Kind::Link))
			{
				cmd.value.actor = shape->owner;
				cmd.value.userData = link->userData;
				cmd.value.type = Physics::ActorType::Link;
			}
		}

		void GetBoundsAtPoseHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::GetBoundsAtPose&>(c);
			cmd.extents = Physics::Vec3(0.0f, 0.0f, 0.0f);
			cmd.center = cmd.pose.pos;
		}

		void ShouldIgnoreCollisionHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::ShouldIgnoreCollision&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			const auto key = std::minmax(cmd.ignoreId0, cmd.ignoreId1);
			cmd.value = g_ignoredPairs.count(std::make_pair(key.first, key.second)) != 0;
		}

		void IgnoreCollisionHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::IgnoreCollision&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			const auto key = std::minmax(cmd.ignoreId0, cmd.ignoreId1);
			const auto pair = std::make_pair(key.first, key.second);
			if (cmd.value)
			{
				g_ignoredPairs.insert(pair);
			}
			else
			{
				g_ignoredPairs.erase(pair);
			}
		}

		void CleanupIgnoredCollidersHandler(Handle, const Context&, Command& c)
		{
			const uint16_t id = static_cast<sd::CleanupIgnoredColliders&>(c).ignoreID;
			std::lock_guard<std::mutex> lock(g_mutex);
			for (auto it = g_ignoredPairs.begin(); it != g_ignoredPairs.end();)
			{
				if (it->first == id || it->second == id)
				{
					it = g_ignoredPairs.erase(it);
				}
				else
				{
					++it;
				}
			}
		}

		Handle NewMeshHandle(Kind kind)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle h = nullptr;
			Allocate(kind, nullptr, nullptr, h);
			return h;
		}

		void CookCollisionMeshHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CookCollisionMesh&>(c);
			if (cmd.onMeshReadyCallback != nullptr)
			{
				cmd.onMeshReadyCallback(Physics::CookingError::Success, nullptr, cmd.dataContext, NewMeshHandle(Kind::Mesh));
			}
		}

		void CookCollisionMeshStreamHandler(Handle, const Context&, Command& c)
		{
			static uint8_t s_dummyStream[8] = { 'S', 'T', 'U', 'B', 0, 0, 0, 0 };
			auto& cmd = static_cast<sd::CookCollisionMeshStream&>(c);
			if (cmd.onStreamReadyCallback != nullptr)
			{
				cmd.onStreamReadyCallback(Physics::CookingError::Success, nullptr, cmd.dataContext, s_dummyStream, sizeof(s_dummyStream));
			}
		}

		void CreateCollisionMeshFromStreamHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CreateCollisionMeshFromStream&>(c);
			cmd.outCollisionMesh = NewMeshHandle(Kind::Mesh);
		}

		void CreateHeightFieldHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CreateHeightField&>(c);
			cmd.heightField = NewMeshHandle(Kind::HeightField);
		}

		void DestroyHeightFieldHandler(Handle, const Context&, Command& c)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			Release(static_cast<sd::DestroyHeightField&>(c).heightField, Kind::HeightField);
		}

		void DestroyCollisionMeshHandler(Handle, const Context&, Command& c)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			Release(static_cast<sd::DestroyCollisionMesh&>(c).collisionMesh, Kind::Mesh);
		}

		void ExtractCollisionMeshDataHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::ExtractCollisionMeshData&>(c);
			if (cmd.callback != nullptr)
			{
				Physics::SDKCollisionMeshData data = {};
				cmd.callback(cmd.ctx, data);
			}
		}

		void RemapTriangleIndexHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::RemapTriangleIndexToSourceIndex&>(c);
			cmd.outSourceMeshTriangleIndex = cmd.physicsMeshTriangleIndex;
		}

		void GetGeometryTypeHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<sd::GetGeometryType&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Record* r = Resolve(ctx, sdkObject, Kind::Shape);
			cmd.value = r != nullptr ? r->geometry.type : Physics::GeometryType::Invalid;
		}

		// ---- Articulation links ----

		void CreateLinkHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::CreateLink&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle articulation = nullptr;
			Record* art = Resolve(ctx, sdkObject, Kind::Articulation, &articulation);

			Handle h = nullptr;
			Record* link = Allocate(Kind::Link, cmd.userData, ctx.world, h);
			link->pose = cmd.pose;
			link->owner = articulation;
			link->parent = cmd.parentLink;

			Handle jointHandle = nullptr;
			Record* joint = Allocate(Kind::LinkJoint, nullptr, ctx.world, jointHandle);
			joint->owner = h;
			// Allocate() may rehash but never moves nodes; re-fetch anyway for clarity.
			link = Find(h, Kind::Link);
			link->joint = jointHandle;

			if (art != nullptr)
			{
				art->links.push_back(h);
			}
			if (Record* parent = Find(cmd.parentLink, Kind::Link))
			{
				parent->children.push_back(h);
			}
			cmd.outNewLink = h;
		}

		void DestroyLinkHandler(Handle, const Context&, Command& c)
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			Release(static_cast<ad::DestroyLink&>(c).value, Kind::Link);
		}

		void GetLinkIndexHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetLinkIndex&>(c);
			cmd.value = 0;
			std::lock_guard<std::mutex> lock(g_mutex);
			Handle self = nullptr;
			Record* link = Resolve(ctx, sdkObject, Kind::Link, &self);
			if (link == nullptr)
			{
				return;
			}
			if (const Record* art = Find(link->owner, Kind::Articulation))
			{
				const auto it = std::find(art->links.begin(), art->links.end(), self);
				if (it != art->links.end())
				{
					cmd.value = static_cast<int>(it - art->links.begin());
				}
			}
		}

		void GetLinkJointHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetLinkJoint&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Record* link = Resolve(ctx, sdkObject, Kind::Link);
			cmd.value = link != nullptr ? link->joint : nullptr;
		}

		void GetLinkCountHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkCount&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			Record* art = Resolve(ctx, sdkObject, Kind::Articulation);
			cmd.value = art != nullptr ? art->links.size() : 0;
		}

		void GetLinksUserDataHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinksUserData&>(c);
			cmd.written = 0;
			std::lock_guard<std::mutex> lock(g_mutex);
			Record* art = Resolve(ctx, sdkObject, Kind::Articulation);
			if (art == nullptr)
			{
				return;
			}
			for (Handle l : art->links)
			{
				if (cmd.buffer == nullptr || cmd.written >= cmd.bufferSize)
				{
					break;
				}
				const Record* link = Find(l, Kind::Link);
				cmd.buffer[cmd.written++] = link != nullptr ? link->userData : nullptr;
			}
		}

		void GetLinkParentHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkParent&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			const Record* link = Find(cmd.link, Kind::Link);
			cmd.value = link != nullptr ? link->parent : nullptr;
		}

		void GetLinkParentUserDataHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkParentUserData&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			const Record* link = Find(cmd.link, Kind::Link);
			const Record* parent = link != nullptr ? Find(link->parent, Kind::Link) : nullptr;
			cmd.value = parent != nullptr ? parent->userData : nullptr;
		}

		void GetLinkChildCountHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkChildCount&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			const Record* link = Find(cmd.link, Kind::Link);
			cmd.value = link != nullptr ? link->children.size() : 0;
		}

		void GetLinkChildrenUserDataHandler(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkChildrenUserData&>(c);
			cmd.written = 0;
			std::lock_guard<std::mutex> lock(g_mutex);
			const Record* link = Find(cmd.link, Kind::Link);
			if (link == nullptr)
			{
				return;
			}
			for (Handle ch : link->children)
			{
				if (cmd.buffer == nullptr || cmd.written >= cmd.bufferSize)
				{
					break;
				}
				const Record* child = Find(ch, Kind::Link);
				cmd.buffer[cmd.written++] = child != nullptr ? child->userData : nullptr;
			}
		}

		void SetArticulationFlagsHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::SetFlags&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* r = Resolve(ctx, sdkObject, Kind::Articulation))
			{
				using type_t = std::underlying_type_t<Physics::ArticulationFlags>;
				const type_t cur = static_cast<type_t>(r->articulationFlags);
				const type_t bits = static_cast<type_t>(cmd.value);
				r->articulationFlags = static_cast<Physics::ArticulationFlags>(cmd.enabled ? (cur | bits) : (cur & ~bits));
			}
		}

		// ---- Joint ----

		void SetJointFlagsHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<jd::SetFlags&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* r = Resolve(ctx, sdkObject, Kind::Joint))
			{
				using type_t = std::underlying_type_t<Physics::JointFlags>;
				const type_t cur = static_cast<type_t>(r->jointFlags);
				const type_t bits = static_cast<type_t>(cmd.value);
				r->jointFlags = static_cast<Physics::JointFlags>(cmd.enabled ? (cur | bits) : (cur & ~bits));
			}
		}

		void GetJointActorsHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<jd::GetActors&>(c);
			cmd.actors[0] = Physics::ActorDescriptor();
			cmd.actors[1] = Physics::ActorDescriptor();
			std::lock_guard<std::mutex> lock(g_mutex);
			const Record* joint = Resolve(ctx, sdkObject, Kind::Joint);
			if (joint == nullptr)
			{
				return;
			}
			for (int i = 0; i < 2; ++i)
			{
				if (const Record* body = Find(joint->actors[i], Kind::Body))
				{
					cmd.actors[i].actor = joint->actors[i];
					cmd.actors[i].userData = body->userData;
					cmd.actors[i].type = body->isStatic ? Physics::ActorType::Static : Physics::ActorType::Dynamic;
				}
				else if (const Record* link = Find(joint->actors[i], Kind::Link))
				{
					cmd.actors[i].actor = joint->actors[i];
					cmd.actors[i].userData = link->userData;
					cmd.actors[i].type = Physics::ActorType::Link;
				}
			}
		}

		void SetJointActorsHandler(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<jd::SetActors&>(c);
			std::lock_guard<std::mutex> lock(g_mutex);
			if (Record* joint = Resolve(ctx, sdkObject, Kind::Joint))
			{
				joint->actors[0] = cmd.actors[0].actor;
				joint->actors[1] = cmd.actors[1].actor;
			}
		}

		// ---- registration pieces ----

		void RegisterSdk()
		{
			Bind(pc::SDK::ShutdownSdk, &ShutdownSdkHandler);
			BindNoops({ pc::SDK::SetErrorVerbosityLevel, pc::SDK::DisconnectVisualDebugger, pc::SDK::SetVisualDebuggerViewport,
				pc::SDK::SetImprovedPatchFrictionEnabledGlobal, pc::SDK::SetMaterialConfiguration, pc::SDK::SetMaterialCombineModes });
			Bind(pc::SDK::ConnectVisualDebugger, [](Handle, const Context&, Command& c) {
				static_cast<sdkd::ConnectVisualDebugger&>(c).result = false;
			});
			Bind(pc::SDK::GetIsVisualDebuggerConnected, [](Handle, const Context&, Command& c) {
				static_cast<sdkd::GetIsVisualDebuggerConnected&>(c).value = false;
			});
			Bind(pc::SDK::CreateMaterial, &CreateMaterialHandler);
			Bind(pc::SDK::ReleaseMaterial, &ReleaseMaterialHandler);
			Bind(pc::SDK::GetDefaultMaterial, &GetDefaultMaterialHandler);
			Bind(pc::SDK::GetMaterialConfiguration, &GetMaterialConfigurationHandler);
			Bind(pc::SDK::GetMaterialCombineModes, &GetMaterialCombineModesHandler);
		}

		void RegisterWorld()
		{
			Bind(pc::World::CreateWorld, &CreateWorldHandler);
			Bind(pc::World::DestroyWorld, &DestroyWorldHandler);
			Bind(pc::World::SetGravity, &StoreField<Kind::World, &wd::SetGravity::value, &Record::gravity>);
			Bind(pc::World::GetGravity, &LoadField<Kind::World, &wd::GetGravity::value, &Record::gravity>);
			Bind(pc::World::SetBounceThreshold, &StoreField<Kind::World, &wd::SetBounceThreshold::value, &Record::bounceThreshold>);
			Bind(pc::World::GetBounceThreshold, &LoadField<Kind::World, &wd::GetBounceThreshold::value, &Record::bounceThreshold>);
			Bind(pc::World::GetBasicStats, &GetBasicStatsHandler);
			Bind(pc::World::Simulate, &SimulateHandler);
			Bind(pc::World::GetLastSimulationStepResults, &GetLastSimulationStepResultsHandler);
			Bind(pc::World::GetLastSimulationStats, &ZeroOut<wd::GetLastSimulationStepStats>);
			Bind(pc::World::GetUserData, &LoadField<Kind::World, &wd::GetUserData::userData, &Record::userData>);
			Bind(pc::World::CreateBody, &CreateBodyHandler);
			Bind(pc::World::DestroyBody, &DestroyBodyHandler);
			Bind(pc::World::CreateArticulation, &CreateArticulationHandler);
			Bind(pc::World::DestroyArticulation, &DestroyArticulationHandler);
			Bind(pc::World::CreateJoint, &CreateJointHandler);
			Bind(pc::World::DestroyJoint, &DestroyJointHandler);
			Bind(pc::World::GetArticulationCount, &GetArticulationCountHandler);
			Bind(pc::World::GetArticulations, &GetArticulationsHandler);
			BindNoops({ pc::World::SetScratchBufferChunkCount, pc::World::ReleaseLastSimulationStepData,
				pc::World::AddActorsToScene, pc::World::GetAllShapesByLayerMask });
		}

		void RegisterBody()
		{
			using B = pc::Body;
			Bind(B::SetFlags, &SetBodyFlags<Kind::Body, bd::SetFlags>);
			Bind(B::GetFlags, &LoadField<Kind::Body, &bd::GetFlags::value, &Record::bodyFlags>);
			Bind(B::SetMass, &StoreField<Kind::Body, &bd::SetMass::value, &Record::mass>);
			Bind(B::GetMass, &LoadField<Kind::Body, &bd::GetMass::value, &Record::mass>);
			Bind(B::SetDamping, &SetDampingH<Kind::Body, bd::SetDamping>);
			Bind(B::GetDamping, &GetDampingH<Kind::Body, bd::GetDamping>);
			Bind(B::SetPose, &StoreField<Kind::Body, &bd::SetPose::value, &Record::pose>);
			Bind(B::GetPose, &LoadField<Kind::Body, &bd::GetPose::value, &Record::pose>);
			Bind(B::AttachShape, &AttachShapeH<Kind::Body, bd::AttachShape>);
			Bind(B::DetachShape, &DetachShapeH<Kind::Body, bd::DetachShape>);
			Bind(B::GetShapeCount, &GetShapeCountH<Kind::Body, bd::GetShapeCount>);
			Bind(B::GetShapes, &GetShapesH<Kind::Body, bd::GetShapes>);
			Bind(B::GetShapesUserData, &GetShapesUserDataH<Kind::Body, bd::GetShapesUserData>);
			Bind(B::GetUserData, &LoadField<Kind::Body, &bd::GetUserData::value, &Record::userData>);

			Bind(B::GetLinearVelocity, &ZeroOut<bd::GetLinearVelocity>);
			Bind(B::GetAngularVelocity, &ZeroOut<bd::GetAngularVelocity>);
			Bind(B::GetMaxLinearVelocity, &ConstantFloat<bd::GetMaxLinearVelocity, 0>);
			Bind(B::GetMaxAngularVelocity, &ConstantFloat<bd::GetMaxAngularVelocity, 1>);
			Bind(B::GetMaxDepenetrationVelocity, &ConstantFloat<bd::GetMaxDepenetrationVelocity, 2>);
			Bind(B::GetInertiaTensor, &ZeroOut<bd::GetInertiaTensor>);
			Bind(B::GetInertiaTensorRotation, &IdentityQuat<bd::GetInertiaTensorRotation>);
			Bind(B::GetLocalCenterOfMass, &ZeroOut<bd::GetLocalCenterOfMass>);
			Bind(B::GetWorldInertiaTensorMatrix, &ZeroOut<bd::GetWorldInertiaTensorMatrix>);
			Bind(B::GetAccumulatedForce, &ZeroOut<bd::GetAccumulatedForce>);
			Bind(B::GetAccumulatedTorque, &ZeroOut<bd::GetAccumulatedTorque>);
			Bind(B::GetLocalPointVelocity, &ZeroOut<bd::GetLocalPointVelocity>);
			Bind(B::GetWorldPointVelocity, &ZeroOut<bd::GetWorldPointVelocity>);
			Bind(B::GetClosestWorldPointOnBounds, &ZeroOut<bd::GetClosestWorldPointOnBounds>);
			Bind(B::GetKinematicTarget, &ZeroOut<bd::GetKinematicTarget>);
			Bind(B::GetIsSleeping, &ZeroOut<bd::GetIsSleeping>);
			Bind(B::GetSleepThresholdOverride, &ZeroOut<bd::GetSleepThresholdOverride>);
			Bind(B::GetSolverIterationsOverride, &ZeroOut<bd::GetSolverIterationsOverride>);

			BindNoops({ B::SetCollisionDetectionMode, B::AddForce, B::AddTorque, B::SetLinearVelocity, B::SetAngularVelocity,
				B::SetMaxLinearVelocity, B::SetMaxAngularVelocity, B::SetMaxDepenetrationVelocity, B::SetInertiaTensor,
				B::SetInertiaTensorRotation, B::SetLocalCenterOfMass, B::RecomputeMassProperties, B::SetKinematicTarget,
				B::Sleep, B::WakeUp, B::SetSleepThresholdOverride, B::SetSolverIterationsOverride });
		}

		void RegisterShape()
		{
			using S = pc::Shape;
			Bind(S::CreateShape, &CreateShapeHandler);
			Bind(S::DestroyShape, &DestroyShapeHandler);
			Bind(S::GetActorDescriptor, &GetActorDescriptorHandler);
			Bind(S::GetUserData, &LoadField<Kind::Shape, &sd::GetUserData::value, &Record::userData>);
			Bind(S::GetFlags, &LoadField<Kind::Shape, &sd::GetFlags::flags, &Record::shapeFlags>);
			Bind(S::SetFlags, &StoreField<Kind::Shape, &sd::SetFlags::flags, &Record::shapeFlags>);
			Bind(S::GetFilterData, &LoadField<Kind::Shape, &sd::GetFilterData::data, &Record::filter>);
			Bind(S::SetFilterData, &StoreField<Kind::Shape, &sd::SetFilterData::data, &Record::filter>);
			Bind(S::SetMaterial, &StoreField<Kind::Shape, &sd::SetMaterial::value, &Record::material>);
			Bind(S::GetLocalPose, &LoadField<Kind::Shape, &sd::GetLocalPose::value, &Record::localPose>);
			Bind(S::GetPose, &LoadField<Kind::Shape, &sd::GetPose::value, &Record::localPose>);
			Bind(S::SetPose, &StoreField<Kind::Shape, &sd::SetPose::value, &Record::localPose>);
			Bind(S::GetWorldBounds, &ZeroOut<sd::GetWorldBounds>);
			Bind(S::GetBoundsAtPose, &GetBoundsAtPoseHandler);
			Bind(S::ShouldIgnoreCollision, &ShouldIgnoreCollisionHandler);
			Bind(S::IgnoreCollision, &IgnoreCollisionHandler);
			Bind(S::CleanupIgnoredColliders, &CleanupIgnoredCollidersHandler);
			Bind(S::CookCollisionMesh, &CookCollisionMeshHandler);
			Bind(S::CookCollisionMeshStream, &CookCollisionMeshStreamHandler);
			Bind(S::CreateCollisionMeshFromStream, &CreateCollisionMeshFromStreamHandler);
			Bind(S::CreateHeightField, &CreateHeightFieldHandler);
			Bind(S::DestroyHeightField, &DestroyHeightFieldHandler);
			Bind(S::DestroyCollisionMesh, &DestroyCollisionMeshHandler);
			Bind(S::ExtractCollisionMeshData, &ExtractCollisionMeshDataHandler);
			Bind(S::GetGeometryType, &GetGeometryTypeHandler);
			Bind(S::SetGeometry, &StoreField<Kind::Shape, &sd::SetGeometry::value, &Record::geometry>);
			Bind(S::GetGeometry, &LoadField<Kind::Shape, &sd::GetGeometry::value, &Record::geometry>);
			Bind(S::RemapTriangleIndexToSourceIndex, &RemapTriangleIndexHandler);
			BindNoops({ S::SetModifiableContacts, S::SetSupportedMessages, S::SetContactOffset, S::UpdateHeightFieldRegion });
		}

		void RegisterQuery()
		{
			using Q = pc::Query;
			Bind(Q::ComputeShapePenetration, [](Handle, const Context&, Command& c) {
				auto& cmd = static_cast<qd::ComputeShapePenetration&>(c);
				cmd.outDirection = Physics::Vec3(0.0f, 0.0f, 0.0f);
				cmd.outDistance = 0.0f;
				cmd.areOverlapping = false;
			});
			Bind(Q::ClosestPointOnShape, [](Handle, const Context&, Command& c) {
				auto& cmd = static_cast<qd::ClosestPointOnShape&>(c);
				cmd.outPoint = cmd.point;
				cmd.outDistance = 0.0f;
			});
			Bind(Q::RayCast, [](Handle, const Context&, Command& c) {
				auto& cmd = static_cast<qd::RayCast&>(c);
				cmd.hit = Physics::QueryHit::Invalid();
				cmd.hitCount = 0;
			});
			Bind(Q::RayCastAgainstShape, [](Handle, const Context&, Command& c) {
				auto& cmd = static_cast<qd::RayCastAgainstShape&>(c);
				cmd.hit = Physics::QueryHit::Invalid();
				cmd.hitCount = 0;
			});
			Bind(Q::Overlap, [](Handle, const Context&, Command& c) {
				static_cast<qd::Overlap&>(c).shapeUserData = nullptr;
			});
			Bind(Q::ShapeCast, [](Handle, const Context&, Command& c) {
				auto& cmd = static_cast<qd::ShapeCast&>(c);
				cmd.hit = Physics::QueryHit::Invalid();
				cmd.hitCount = 0;
			});
			Bind(Q::ShapeCastAgainstShape, [](Handle, const Context&, Command& c) {
				auto& cmd = static_cast<qd::ShapeCastAgainstShape&>(c);
				cmd.hit = Physics::QueryHit::Invalid();
				cmd.hitCount = 0;
			});
			Bind(Q::BodyCast, [](Handle, const Context&, Command& c) {
				auto& cmd = static_cast<qd::BodyCast&>(c);
				cmd.hit = Physics::QueryHit::Invalid();
				cmd.hitCount = 0;
			});
			// Collector variants report nothing: the collector callback is never invoked.
			BindNoops({ Q::RayCastWithCollector, Q::OverlapWithCollector, Q::OverlapWithBroadphaseCollector,
				Q::ShapeCastWithCollector, Q::BodyCastWithCollector });
		}

		void RegisterJoint()
		{
			using J = pc::Joint;
			Bind(J::SetFlags, &SetJointFlagsHandler);
			Bind(J::GetFlags, &LoadField<Kind::Joint, &jd::GetFlags::value, &Record::jointFlags>);
			Bind(J::GetUserData, &LoadField<Kind::Joint, &jd::GetUserData::value, &Record::userData>);
			Bind(J::GetActorLocalPose, &ZeroOut<jd::GetActorLocalPose>);
			Bind(J::GetIsConstraintValid, [](Handle, const Context&, Command& c) {
				static_cast<jd::GetIsConstraintValid&>(c).value = true;
			});
			Bind(J::GetForces, &ZeroOut<jd::GetForces>);
			Bind(J::SetActors, &SetJointActorsHandler);
			Bind(J::GetActors, &GetJointActorsHandler);
			Bind(J::Get6DofLimit, &ZeroOut<jd::Get6DofLimit>);
			BindNoops({ J::SetActorLocalPose, J::SetBreakForce, J::SetInvMassAndInertiaScale, J::WakeUpActors,
				J::Set6DofLimit, J::Set6DofSpringLimit, J::Set6DofAxisLock, J::Set6DofAxisMotorConfiguration,
				J::Set6DofMotorTargetPosition, J::Set6DofMotorTargetRotation, J::Set6DofMotorTargetLinearVelocity,
				J::Set6DofMotorTargetAngularVelocity, J::Set6DofProjectionTolerance, J::SetDistanceLimit,
				J::SetDistanceSpringLimit, J::SetDistanceErrorTolerance, J::SetHingeMotorConfiguration,
				J::SetHingeMotorTargetVelocity, J::SetHingeLimit, J::SetHingeLimitEnabled });
		}

		void RegisterArticulation()
		{
			using A = pc::Articulation;

			// Articulation link (mirrors Body)
			Bind(A::GetLinkIndex, &GetLinkIndexHandler);
			Bind(A::GetLinkJoint, &GetLinkJointHandler);
			Bind(A::SetLinkFlags, &SetBodyFlags<Kind::Link, ald::SetFlags>);
			Bind(A::GetLinkFlags, &LoadField<Kind::Link, &ald::GetFlags::value, &Record::bodyFlags>);
			Bind(A::SetLinkMass, &StoreField<Kind::Link, &ald::SetMass::value, &Record::mass>);
			Bind(A::GetLinkMass, &LoadField<Kind::Link, &ald::GetMass::value, &Record::mass>);
			Bind(A::SetLinkDamping, &SetDampingH<Kind::Link, ald::SetDamping>);
			Bind(A::GetLinkDamping, &GetDampingH<Kind::Link, ald::GetDamping>);
			Bind(A::SetLinkPose, &StoreField<Kind::Link, &ald::SetPose::value, &Record::pose>);
			Bind(A::GetLinkPose, &LoadField<Kind::Link, &ald::GetPose::value, &Record::pose>);
			Bind(A::AttachShapeToLink, &AttachShapeH<Kind::Link, ald::AttachShape>);
			Bind(A::DetachShapeFromLink, &DetachShapeH<Kind::Link, ald::DetachShape>);
			Bind(A::GetLinkShapeCount, &GetShapeCountH<Kind::Link, ald::GetShapeCount>);
			Bind(A::GetLinkShapesUserData, &GetShapesUserDataH<Kind::Link, ald::GetShapesUserData>);
			Bind(A::GetLinkUserData, &LoadField<Kind::Link, &ald::GetUserData::value, &Record::userData>);
			Bind(A::GetLinkLinearVelocity, &ZeroOut<ald::GetLinearVelocity>);
			Bind(A::GetLinkAngularVelocity, &ZeroOut<ald::GetAngularVelocity>);
			Bind(A::GetLinkMaxLinearVelocity, &ConstantFloat<ald::GetMaxLinearVelocity, 0>);
			Bind(A::GetLinkMaxAngularVelocity, &ConstantFloat<ald::GetMaxAngularVelocity, 1>);
			Bind(A::GetLinkMaxDepenetrationVelocity, &ConstantFloat<ald::GetMaxDepenetrationVelocity, 2>);
			Bind(A::GetLinkInertiaTensor, &ZeroOut<ald::GetInertiaTensor>);
			Bind(A::GetLinkInertiaTensorRotation, &IdentityQuat<ald::GetInertiaTensorRotation>);
			Bind(A::GetLinkLocalCenterOfMass, &ZeroOut<ald::GetLocalCenterOfMass>);
			Bind(A::GetLinkWorldInertiaTensorMatrix, &ZeroOut<ald::GetWorldInertiaTensorMatrix>);
			Bind(A::GetLinkAccumulatedForce, &ZeroOut<ald::GetAccumulatedForce>);
			Bind(A::GetLinkAccumulatedTorque, &ZeroOut<ald::GetAccumulatedTorque>);
			Bind(A::GetLinkLocalPointVelocity, &ZeroOut<ald::GetLocalPointVelocity>);
			Bind(A::GetLinkWorldPointVelocity, &ZeroOut<ald::GetWorldPointVelocity>);
			Bind(A::GetLinkClosestWorldPoint, &ZeroOut<ald::GetClosestWorldPoint>);
			BindNoops({ A::SetLinkCollisionDetectionMode, A::AddForceToLink, A::AddTorqueToLink, A::SetLinkLinearVelocity,
				A::SetLinkAngularVelocity, A::SetLinkMaxLinearVelocity, A::SetLinkMaxAngularVelocity,
				A::SetLinkMaxDepenetrationVelocity, A::SetLinkInertiaTensor, A::SetLinkInertiaTensorRotation,
				A::SetLinkLocalCenterOfMass, A::RecomputeLinkMassProperties });

			// Articulation link joint
			Bind(A::GetLinkJointType, &ZeroOut<ajd::GetType>);
			Bind(A::GetLinkJointPosition, &ZeroOut<ajd::GetPosition>);
			Bind(A::GetLinkJointVelocity, &ZeroOut<ajd::GetVelocity>);
			Bind(A::GetLinkJointAcceleration, &ZeroOut<ajd::GetAcceleration>);
			Bind(A::GetLinkJointForce, &ZeroOut<ajd::GetForce>);
			Bind(A::GetLinkJointMotorForce, &ZeroOut<ajd::GetMotorForce>);
			Bind(A::GetLinkJointForcesForAcceleration, &ZeroOut<ajd::GetForcesForAcceleration>);
			Bind(A::GetLinkJointMaxVelocity, &ZeroOut<ajd::GetMaxVelocity>);
			Bind(A::GetLinkJointDofCount, &ZeroOut<ajd::GetDofCount>);
			BindNoops({ A::SetLinkJointType, A::SetLinkJointAxisLock, A::SetLinkJointLimit, A::SetLinkJointActorLocalPose,
				A::SetLinkJointAxisMotorConfiguration, A::SetLinkJointFriction, A::SetLinkJointPosition,
				A::SetLinkJointVelocity, A::SetLinkJointAcceleration, A::SetLinkJointForce, A::SetLinkJointMaxVelocity });

			// Articulation
			Bind(A::CreateLink, &CreateLinkHandler);
			Bind(A::DestroyLink, &DestroyLinkHandler);
			Bind(A::GetLinkCount, &GetLinkCountHandler);
			Bind(A::GetLinksUserData, &GetLinksUserDataHandler);
			Bind(A::GetLinkParent, &GetLinkParentHandler);
			Bind(A::GetLinkParentUserData, &GetLinkParentUserDataHandler);
			Bind(A::GetLinkChildCount, &GetLinkChildCountHandler);
			Bind(A::GetLinkChildrenUserData, &GetLinkChildrenUserDataHandler);
			Bind(A::SetFlags, &SetArticulationFlagsHandler);
			Bind(A::GetFlags, &LoadField<Kind::Articulation, &ad::GetFlags::value, &Record::articulationFlags>);
			Bind(A::GetUserData, &LoadField<Kind::Articulation, &ad::GetUserData::value, &Record::userData>);
			Bind(A::GetIsSleeping, &ZeroOut<ad::GetIsSleeping>);
			Bind(A::GetSleepThresholdOverride, &ZeroOut<ad::GetSleepThresholdOverride>);
			Bind(A::GetSolverIterationsOverride, &ZeroOut<ad::GetSolverIterationsOverride>);
			Bind(A::GetDenseJacobianAsFloatBuffer, &ZeroOut<ad::GetDenseJacobianAsBuffer>);
			Bind(A::GetJointsPositions, &ZeroOut<ad::GetJointsPositions>);
			Bind(A::GetJointsVelocities, &ZeroOut<ad::GetJointsVelocities>);
			Bind(A::GetJointsAccelerations, &ZeroOut<ad::GetJointsAccelerations>);
			Bind(A::GetJointsForces, &ZeroOut<ad::GetJointsForces>);
			Bind(A::GetJointsDriveForces, &ZeroOut<ad::GetJointsDriveForces>);
			Bind(A::GetJointsGravityForces, &ZeroOut<ad::GetJointsGravityForces>);
			Bind(A::GetJointsCoriolisCentrifugalForces, &ZeroOut<ad::GetJointsCoriolisCentrifugalForces>);
			Bind(A::GetJointsExternalForces, &ZeroOut<ad::GetJointsExternalForces>);
			BindNoops({ A::Sleep, A::WakeUp, A::SetSleepThresholdOverride, A::SetSolverIterationsOverride,
				A::SetJointsPositions, A::SetJointsVelocities, A::SetJointsAccelerations, A::SetJointsForces,
				A::ReleaseDataBuffer });
		}
	}

	void RegisterStubWorldHandlers()
	{
		RegisterSdk();
		RegisterWorld();
		RegisterBody();
		RegisterShape();
		RegisterQuery();
		RegisterJoint();
		RegisterArticulation();
	}

	void ResetStubWorld()
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		g_records.clear();
		g_ignoredPairs.clear();
		g_defaultMaterial = nullptr;
	}

	size_t GetLiveHandleCount()
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		return g_records.size();
	}
}
