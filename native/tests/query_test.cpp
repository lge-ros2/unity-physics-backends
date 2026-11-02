// Drives the scene query commands through the Unity command entry point.
// Usage: query_test [path/to/libBackend.so]

#include <dlfcn.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "IUnityInterface.h"
#include "IUnityLog.h"
#include "IUnityPhysicsBackendSystem.h"
#include "PhysicsCommands.h"
#include "PhysicsGeneralCommandData.h"
#include "PhysicsMaterialCommandData.h"
#include "PhysicsWorldCommandData.h"
#include "PhysicsBodyCommandData.h"
#include "PhysicsShapeCommandData.h"
#include "PhysicsQueryCommandData.h"

#ifndef FALL_TEST_BACKEND_PATH
#define FALL_TEST_BACKEND_PATH "libPhysX5Backend.so"
#endif
#ifndef FALL_TEST_SDK
#define FALL_TEST_SDK 0
#endif

using namespace PhysicsCommands;
using Handle = Physics::SDKObjectHandle;
using Opt = Physics::QueryFilterFlags;

namespace
{
	BackendEntryPoint g_entry = nullptr;
	int g_errorLogs = 0;
	int g_failures = 0;
	int g_checks = 0;

	void LogFn(UnityLogType type, const char* message, const char*, int)
	{
		if (type == kUnityLogTypeError)
		{
			++g_errorLogs;
			std::printf("  [backend error] %s\n", message);
		}
		else if (std::getenv("QUERY_TEST_VERBOSE") != nullptr)
		{
			std::printf("  [backend log %d] %s\n", static_cast<int>(type), message);
		}
	}

	IUnityLog g_log{ {}, LogFn };

	bool RegisterFn(BackendEntryPoint ep)
	{
		g_entry = ep;
		return true;
	}

	IUnityPhysicsBackendSystem g_system{ {}, RegisterFn };

	IUnityInterface* GetInterface(UnityInterfaceGUID guid)
	{
		if (guid == GetUnityInterfaceGUID<IUnityLog>())
		{
			return reinterpret_cast<IUnityInterface*>(&g_log);
		}
		if (guid == GetUnityInterfaceGUID<IUnityPhysicsBackendSystem>())
		{
			return reinterpret_cast<IUnityInterface*>(&g_system);
		}
		return nullptr;
	}

	template <typename E, typename C>
	void Run(Handle obj, const Context& ctx, E id, C& cmd)
	{
		g_entry(obj, ctx, ETP(id), cmd);
	}

	bool Check(bool ok, const char* what)
	{
		++g_checks;
		if (!ok)
		{
			++g_failures;
			std::printf("  FAIL: %s\n", what);
		}
		return ok;
	}

	bool Near(float a, float b, float tol)
	{
		return std::fabs(a - b) <= tol;
	}

	struct Dummy
	{
		uint64_t id;
	};

	// ---- scene building ----

	Physics::FilterData MakeFilter(uint32_t belongsTo)
	{
		Physics::FilterData f;
		f.belongsTo = belongsTo;
		f.collidesWith = 0xFFFFFFFFu;
		return f;
	}

	Physics::ShapeGeometry BoxGeom(float hx, float hy, float hz)
	{
		Physics::ShapeGeometry g = {};
		g.type = Physics::GeometryType::Box;
		g.boxGeometry.halfExtents = Physics::Vec3(hx, hy, hz);
		return g;
	}

	Physics::ShapeGeometry SphereGeom(float r)
	{
		Physics::ShapeGeometry g = {};
		g.type = Physics::GeometryType::Sphere;
		g.sphereGeometry.radius = r;
		return g;
	}

	Physics::ShapeGeometry CapsuleGeom(float r, float height)
	{
		Physics::ShapeGeometry g = {};
		g.type = Physics::GeometryType::Capsule;
		g.capsuleGeometry.radius = r;
		g.capsuleGeometry.height = height;
		return g;
	}

	Physics::ShapeGeometry MeshGeom(Physics::GeometryType type)
	{
		Physics::ShapeGeometry g = {};
		g.type = type;
		g.meshGeometry.mesh = nullptr;
		g.meshGeometry.scale = Physics::Vec3(1.0f, 1.0f, 1.0f);
		return g;
	}

	Handle CreateBody(Handle world, bool isStatic, const Physics::Pose& pose)
	{
		WorldData::CreateBody cb{};
		cb.pose = pose;
		cb.mass = 1.0f;
		cb.angularDamping = 0.05f;
		cb.flags = Physics::BodyFlags::Simulated;
		cb.isStatic = isStatic;
		Run(world, Context(world, nullptr), World::CreateBody, cb);
		return cb.outBody;
	}

	Handle CreateShape(Handle world, const Physics::ShapeGeometry& geom, Handle material, void* userData, uint32_t layer, bool trigger)
	{
		ShapeData::CreateShape cs{};
		cs.geom = geom;
		cs.materialPtr = material;
		cs.userData = userData;
		cs.flags = Physics::ShapeFlags::SceneQuery;
		Run(nullptr, Context(world, nullptr), Shape::CreateShape, cs);
		if (cs.value != nullptr)
		{
			ShapeData::SetFilterData sf{};
			sf.data = MakeFilter(layer);
			Run(cs.value, Context(world, nullptr), Shape::SetFilterData, sf);
			if (trigger)
			{
				ShapeData::SetFlags sfl{};
				sfl.flags = Physics::ShapeFlags::SceneQuery | Physics::ShapeFlags::Trigger;
				Run(cs.value, Context(world, nullptr), Shape::SetFlags, sfl);
			}
		}
		return cs.value;
	}

	void AttachShape(Handle world, Handle body, Handle shape, const Physics::Pose& local = Physics::Pose())
	{
		BodyData::AttachShape as{};
		as.shape = shape;
		as.pose = local;
		as.forceLocalPose = true;
		Run(body, Context(world, body), Body::AttachShape, as);
	}

	void AddToScene(Handle world, Handle body)
	{
		Handle list[1] = { body };
		WorldData::AddActorsToScene aa{};
		aa.actors = list;
		aa.bufferSize = 1;
		Run(world, Context(world, nullptr), World::AddActorsToScene, aa);
	}

	void Recompute(Handle world, Handle body)
	{
		BodyData::RecomputeMassProperties rp;
		Run(body, Context(world, body), Body::RecomputeMassProperties, rp);
	}

	struct Placed
	{
		Handle body = nullptr;
		Handle shape = nullptr;
	};

	Placed Place(Handle world, Handle material, bool isStatic, const Physics::Vec3& pos, const Physics::ShapeGeometry& geom,
		Dummy* user, uint32_t layer, bool trigger = false)
	{
		Placed p;
		p.body = CreateBody(world, isStatic, Physics::Pose(pos.x, pos.y, pos.z, 0, 0, 0, 1));
		p.shape = CreateShape(world, geom, material, user, layer, trigger);
		Check(p.body != nullptr && p.shape != nullptr, "placed object created");
		AttachShape(world, p.body, p.shape);
		if (!isStatic)
		{
			Recompute(world, p.body);
		}
		AddToScene(world, p.body);
		return p;
	}

	// ---- query helpers ----

	constexpr Opt kAll = Opt::UseDynamicBodies | Opt::UseStaticBodies;
	constexpr uint32_t kAllLayers = 0xFFFFFFFFu;

	Physics::QueryFilterData Filter(Opt options, uint32_t mask = kAllLayers)
	{
		Physics::QueryFilterData f;
		f.options = options;
		f.mask = static_cast<int>(mask);
		return f;
	}

	namespace qd = QueryData;

	struct RayResult
	{
		uint32_t count;
		Physics::QueryHit hit;
	};

	RayResult RayCast(Handle world, const Physics::Vec3& origin, const Physics::Vec3& dir, float maxDistance, const Physics::QueryFilterData& f)
	{
		qd::RayCast cmd{};
		cmd.origin = origin;
		cmd.direction = dir;
		cmd.maxDistance = maxDistance;
		cmd.filter = f;
		Run(world, Context(world, nullptr), Query::RayCast, cmd);
		return { cmd.hitCount, cmd.hit };
	}

	// Collector state shared by the hit collectors.
	struct HitCollector
	{
		int calls = 0;
		bool keepGoing = true;
		std::vector<Physics::QueryHit> hits;

		static bool Callback(void* context, Physics::QueryHit* hits, const size_t count)
		{
			auto* self = static_cast<HitCollector*>(context);
			++self->calls;
			for (size_t i = 0; i < count; ++i)
			{
				self->hits.push_back(hits[i]);
			}
			return self->keepGoing;
		}
	};

	struct ShapeCollector
	{
		int calls = 0;
		std::vector<void*> userDatas;

		static bool Callback(void* context, void** userDatas, const size_t count)
		{
			auto* self = static_cast<ShapeCollector*>(context);
			++self->calls;
			for (size_t i = 0; i < count; ++i)
			{
				self->userDatas.push_back(userDatas[i]);
			}
			return true;
		}
	};

	struct BroadPhaseCollector
	{
		struct Entry
		{
			Handle shape;
			Handle body;
			void* userData;
			Physics::GeometryType type;
		};
		std::vector<Entry> entries;
		size_t stopAfter = 0; // 0 = never stop

		static bool Callback(void* context, Handle shape, Handle body, void* userData, Physics::GeometryType type)
		{
			auto* self = static_cast<BroadPhaseCollector*>(context);
			self->entries.push_back({ shape, body, userData, type });
			return self->stopAfter == 0 || self->entries.size() < self->stopAfter;
		}
	};

	bool Contains(const std::vector<void*>& list, void* value)
	{
		for (void* v : list)
		{
			if (v == value)
			{
				return true;
			}
		}
		return false;
	}

	uint64_t IdOf(const Physics::QueryHit& hit)
	{
		return hit.userData.data;
	}

	void* OverlapFirst(Handle world, const Physics::ShapeGeometry& geom, const Physics::Pose& pose, const Physics::QueryFilterData& f)
	{
		qd::Overlap cmd{};
		cmd.geometry = geom;
		cmd.pose = pose;
		cmd.filter = f;
		cmd.shapeUserData = reinterpret_cast<void*>(0x1);
		Run(world, Context(world, nullptr), Query::Overlap, cmd);
		return cmd.shapeUserData;
	}

	ShapeCollector OverlapAll(Handle world, const Physics::ShapeGeometry& geom, const Physics::Pose& pose, const Physics::QueryFilterData& f)
	{
		ShapeCollector collector;
		qd::OverlapWithCollector cmd{};
		cmd.geometry = geom;
		cmd.pose = pose;
		cmd.filter = f;
		cmd.dataContext = &collector;
		cmd.collector = &ShapeCollector::Callback;
		Run(world, Context(world, nullptr), Query::OverlapWithCollector, cmd);
		return collector;
	}

	qd::ShapeCast ShapeCast(Handle world, const Physics::ShapeGeometry& geom, const Physics::Pose& pose, const Physics::Vec3& dir,
		float maxDistance, const Physics::QueryFilterData& f)
	{
		qd::ShapeCast cmd{};
		cmd.geometry = geom;
		cmd.pose = pose;
		cmd.filter = f;
		cmd.direction = dir;
		cmd.maxDistance = maxDistance;
		Run(world, Context(world, nullptr), Query::ShapeCast, cmd);
		return cmd;
	}

	HitCollector ShapeCastAll(Handle world, const Physics::ShapeGeometry& geom, const Physics::Pose& pose, const Physics::Vec3& dir,
		float maxDistance, const Physics::QueryFilterData& f, bool keepInitialOverlaps)
	{
		HitCollector collector;
		qd::ShapeCastWithCollector cmd{};
		cmd.geometry = geom;
		cmd.pose = pose;
		cmd.keepInitialOverlaps = keepInitialOverlaps;
		cmd.filter = f;
		cmd.direction = dir;
		cmd.maxDistance = maxDistance;
		cmd.dataContext = &collector;
		cmd.collector = &HitCollector::Callback;
		Run(world, Context(world, nullptr), Query::ShapeCastWithCollector, cmd);
		return collector;
	}

	qd::RayCastAgainstShape RayAgainst(Handle world, Handle target, const Physics::Vec3& origin, const Physics::Vec3& dir, float maxDistance, Opt options)
	{
		qd::RayCastAgainstShape cmd{};
		cmd.origin = origin;
		cmd.direction = dir;
		cmd.filterOptions = options;
		cmd.maxDistance = maxDistance;
		Run(target, Context(world, nullptr), Query::RayCastAgainstShape, cmd);
		return cmd;
	}

	qd::ShapeCastAgainstShape ShapeAgainst(Handle world, Handle target, const Physics::ShapeGeometry& geom, const Physics::Pose& pose,
		const Physics::Vec3& dir, float maxDistance, const Physics::QueryFilterData& f, bool distanceAsDepth)
	{
		qd::ShapeCastAgainstShape cmd{};
		cmd.targetGeometry = geom;
		cmd.targetPose = pose;
		cmd.filter = f;
		cmd.direction = dir;
		cmd.maxDistance = maxDistance;
		cmd.distanceAsDepth = distanceAsDepth;
		Run(target, Context(world, nullptr), Query::ShapeCastAgainstShape, cmd);
		return cmd;
	}

	qd::ComputeShapePenetration Penetration(Handle a, Handle b, const Physics::Pose& poseA, const Physics::Pose& poseB)
	{
		qd::ComputeShapePenetration cmd{};
		cmd.shapeA = a;
		cmd.shapeB = b;
		cmd.poseA = poseA;
		cmd.poseB = poseB;
		Run(nullptr, Context(), Query::ComputeShapePenetration, cmd);
		return cmd;
	}

	qd::ClosestPointOnShape ClosestPoint(Handle shape, const Physics::Pose& pose, const Physics::Vec3& point)
	{
		qd::ClosestPointOnShape cmd{};
		cmd.pose = pose;
		cmd.point = point;
		Run(shape, Context(), Query::ClosestPointOnShape, cmd);
		return cmd;
	}

	const Physics::Vec3 kPlusX(1.0f, 0.0f, 0.0f);
	const Physics::Vec3 kPlusZ(0.0f, 0.0f, 1.0f);
	const Physics::Vec3 kOrigin(0.0f, 0.0f, 0.0f);
}

int main(int argc, char** argv)
{
	const std::string path = argc > 1 ? argv[1] : FALL_TEST_BACKEND_PATH;
	void* lib = dlopen(path.c_str(), RTLD_NOW);
	if (lib == nullptr)
	{
		std::printf("dlopen(%s) failed: %s\n", path.c_str(), dlerror());
		return 2;
	}
	auto load = reinterpret_cast<void (*)(IUnityInterfaces*)>(dlsym(lib, "UnityPluginLoad"));
	auto unload = reinterpret_cast<void (*)()>(dlsym(lib, "UnityPluginUnload"));
	if (load == nullptr || unload == nullptr)
	{
		std::printf("missing plugin exports\n");
		return 2;
	}

	IUnityInterfaces interfaces{};
	interfaces.GetInterface = GetInterface;
	load(&interfaces);
	if (g_entry == nullptr)
	{
		std::printf("backend did not register an entry point\n");
		return 2;
	}
	std::printf("query_test: backend=%s (requested SDK %d)\n", path.c_str(), FALL_TEST_SDK);

	SdkData::InitializeSdk init;
	init.engineObjectIdOffset = 0;
	Run(nullptr, Context(), SDK::InitializeSdk, init);
	Check(init.result, "InitializeSdk");

	MaterialData::GetDefaultMaterial dm{};
	Run(nullptr, Context(), SDK::GetDefaultMaterial, dm);
	const Handle material = dm.value;
	Check(material != nullptr, "GetDefaultMaterial");

	WorldData::CreateWorld cw;
	cw.jobWorkerCount = 1;
	cw.gravity = Physics::Vec3(0.0f, 0.0f, 0.0f);
	Run(nullptr, Context(), World::CreateWorld, cw);
	const Handle world = cw.outWorld;
	Check(world != nullptr, "CreateWorld");

	// Row along +X, all at y = z = 0:
	//   box     (static,  layer 1)  center 10, half 1        -> front face x = 9
	//   sphere  (dynamic, layer 2)  center 20, radius 1      -> front x = 19
	//   capsule (static,  layer 4)  center 30, r 1, h 4      -> axis along X, front x = 28
	//   trigger (static,  layer 8)  center 40, box half 1    -> front x = 39
	Dummy users[8] = { { 101 }, { 102 }, { 103 }, { 104 }, { 105 }, { 106 }, { 107 }, { 108 } };
	const Placed box = Place(world, material, true, Physics::Vec3(10, 0, 0), BoxGeom(1, 1, 1), &users[0], 1);
	const Placed sphere = Place(world, material, false, Physics::Vec3(20, 0, 0), SphereGeom(1), &users[1], 2);
	const Placed capsule = Place(world, material, true, Physics::Vec3(30, 0, 0), CapsuleGeom(1, 4), &users[2], 4);
	const Placed trigger = Place(world, material, true, Physics::Vec3(40, 0, 0), BoxGeom(1, 1, 1), &users[3], 8, true);

	// Multi-shape body for the body casts (kept away from the row): shapes at local (0,0,0) sphere and (0,5,0) box.
	Placed obstacle1 = Place(world, material, true, Physics::Vec3(0, 105, 10), BoxGeom(1, 1, 1), &users[4], 1);
	Placed obstacle2 = Place(world, material, true, Physics::Vec3(0, 100, 20), BoxGeom(1, 1, 1), &users[5], 1);
	const Handle compound = CreateBody(world, false, Physics::Pose(0, 100, 0, 0, 0, 0, 1));
	const Handle compoundA = CreateShape(world, SphereGeom(0.5f), material, &users[6], 1, false);
	const Handle compoundB = CreateShape(world, BoxGeom(0.5f, 0.5f, 0.5f), material, &users[7], 1, false);
	AttachShape(world, compound, compoundA, Physics::Pose(0, 0, 0, 0, 0, 0, 1));
	AttachShape(world, compound, compoundB, Physics::Pose(0, 5, 0, 0, 0, 0, 1));
	Recompute(world, compound);
	AddToScene(world, compound);

	WorldData::Simulate sim;
	sim.deltaTime = 0.02f;
	Run(world, Context(world, nullptr), World::Simulate, sim);
	Check(sim.ranSimulation, "Simulate ran");

	// ---- (a) RayCast ----
	std::printf("[a] ray casts\n");
	{
		RayResult r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(kAll));
		Check(r.count == 1 && Near(r.hit.distance, 9.0f, 1e-3f), "ray hits box front at 9");
		Check(Near(r.hit.point.x, 9.0f, 1e-3f) && Near(r.hit.normal.x, -1.0f, 1e-3f), "ray hit point/normal");
		Check(IdOf(r.hit) == 101, "ray hit object is the box");

		r = RayCast(world, kOrigin, kPlusX, 5.0f, Filter(kAll));
		Check(r.count == 0, "ray stops at max distance");
		r = RayCast(world, kOrigin, kPlusX, std::numeric_limits<float>::infinity(), Filter(kAll));
		Check(r.count == 1 && Near(r.hit.distance, 9.0f, 1e-3f), "infinite max distance is clamped");
		r = RayCast(world, kOrigin, Physics::Vec3(0, 0, 0), 100.0f, Filter(kAll));
		Check(r.count == 0, "zero direction is no hit");
		r = RayCast(world, kOrigin, Physics::Vec3(3.0f, 0, 0), 100.0f, Filter(kAll));
		Check(r.count == 1 && Near(r.hit.distance, 9.0f, 1e-3f), "direction is normalized");

		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(kAll, 2 | 4));
		Check(r.count == 1 && Near(r.hit.distance, 19.0f, 1e-3f) && IdOf(r.hit) == 102, "layer mask picks the sphere");
		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(kAll, 4));
		Check(r.count == 1 && Near(r.hit.distance, 28.0f, 1e-3f) && IdOf(r.hit) == 103, "layer mask picks the capsule");
		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(kAll, 8));
		Check(r.count == 0, "trigger excluded by default");
		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(kAll | Opt::UseTriggerShapes, 8));
		Check(r.count == 1 && Near(r.hit.distance, 39.0f, 1e-3f) && IdOf(r.hit) == 104, "trigger included with UseTriggerShapes");
		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(Opt::UseStaticBodies, 2));
		Check(r.count == 0, "static-only skips the dynamic sphere");
		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(Opt::UseDynamicBodies, kAllLayers));
		Check(r.count == 1 && IdOf(r.hit) == 102, "dynamic-only hits the sphere first");
		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(Opt::UseStaticBodies, kAllLayers));
		Check(r.count == 1 && IdOf(r.hit) == 101, "static-only hits the box first");
		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(Opt::None));
		Check(r.count == 0, "no body kind enabled is no hit");
		r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(kAll | Opt::AllowEarlyOutOnFirstHit));
		Check(r.count == 1, "early-out still reports a hit");

		// origin inside a shape: not reported (Unity), the next shape is
		r = RayCast(world, Physics::Vec3(10, 0, 0), kPlusX, 100.0f, Filter(kAll));
		Check(r.count == 1 && IdOf(r.hit) == 102, "ray starting inside the box skips it");

		HitCollector col;
		qd::RaycastWithCollector cmd{};
		cmd.origin = kOrigin;
		cmd.direction = kPlusX;
		cmd.maxDistance = 100.0f;
		cmd.filter = Filter(kAll | Opt::UseTriggerShapes);
		cmd.dataContext = &col;
		cmd.collector = &HitCollector::Callback;
		Run(world, Context(world, nullptr), Query::RayCastWithCollector, cmd);
		Check(col.hits.size() == 4, "collector receives all 4 hits");
		if (col.hits.size() == 4)
		{
			Check(Near(col.hits[0].distance, 9.0f, 1e-3f) && Near(col.hits[1].distance, 19.0f, 1e-3f)
					&& Near(col.hits[2].distance, 28.0f, 1e-3f) && Near(col.hits[3].distance, 39.0f, 1e-3f),
				"collector hits sorted by distance");
			Check(IdOf(col.hits[0]) == 101 && IdOf(col.hits[1]) == 102 && IdOf(col.hits[2]) == 103 && IdOf(col.hits[3]) == 104,
				"collector hit objects");
		}

		HitCollector col2;
		cmd.filter = Filter(kAll);
		cmd.dataContext = &col2;
		Run(world, Context(world, nullptr), Query::RayCastWithCollector, cmd);
		Check(col2.hits.size() == 3, "collector skips the trigger by default");
		HitCollector col3;
		cmd.filter = Filter(kAll, 2);
		cmd.dataContext = &col3;
		Run(world, Context(world, nullptr), Query::RayCastWithCollector, cmd);
		Check(col3.hits.size() == 1 && IdOf(col3.hits[0]) == 102, "collector honors the layer mask");
		HitCollector col4;
		cmd.filter = Filter(kAll, 16);
		cmd.dataContext = &col4;
		Run(world, Context(world, nullptr), Query::RayCastWithCollector, cmd);
		Check(col4.calls == 0, "collector not called without hits");
		HitCollector col5;
		cmd.filter = Filter(kAll | Opt::AllowEarlyOutOnFirstHit);
		cmd.dataContext = &col5;
		Run(world, Context(world, nullptr), Query::RayCastWithCollector, cmd);
		Check(col5.hits.size() == 1, "early-out collector reports a single hit");
	}

	// ---- (b) Overlap ----
	std::printf("[b] overlaps\n");
	{
		const Physics::QueryFilterData f = Filter(kAll);
		Check(OverlapFirst(world, BoxGeom(0.2f, 0.2f, 0.2f), Physics::Pose(10.5f, 0, 0, 0, 0, 0, 1), f) == &users[0], "box query overlaps the box");
		Check(OverlapFirst(world, SphereGeom(0.1f), Physics::Pose(19.5f, 0, 0, 0, 0, 0, 1), f) == &users[1], "sphere query overlaps the sphere");
		Check(OverlapFirst(world, CapsuleGeom(0.1f, 1.0f), Physics::Pose(28.5f, 0, 0, 0, 0, 0, 1), f) == &users[2], "capsule query overlaps the capsule");
		Check(OverlapFirst(world, SphereGeom(0.5f), Physics::Pose(0, 50, 0, 0, 0, 0, 1), f) == nullptr, "empty space has no overlap");
		Check(OverlapFirst(world, SphereGeom(0.5f), Physics::Pose(40, 0, 0, 0, 0, 0, 1), f) == nullptr, "trigger not overlapped by default");
		Check(OverlapFirst(world, SphereGeom(0.5f), Physics::Pose(40, 0, 0, 0, 0, 0, 1), Filter(kAll | Opt::UseTriggerShapes)) == &users[3],
			"trigger overlapped with UseTriggerShapes");
		Check(OverlapFirst(world, BoxGeom(0.2f, 0.2f, 0.2f), Physics::Pose(10.5f, 0, 0, 0, 0, 0, 1), Filter(kAll, 2)) == nullptr,
			"layer mask excludes the box");
		Check(OverlapFirst(world, BoxGeom(0.2f, 0.2f, 0.2f), Physics::Pose(10.5f, 0, 0, 0, 0, 0, 1), Filter(Opt::UseDynamicBodies)) == nullptr,
			"dynamic-only excludes the static box");

		const Physics::ShapeGeometry big = BoxGeom(20, 5, 5);
		const Physics::Pose bigPose(25, 0, 0, 0, 0, 0, 1);
		ShapeCollector all = OverlapAll(world, big, bigPose, f);
		Check(all.userDatas.size() == 3 && Contains(all.userDatas, &users[0]) && Contains(all.userDatas, &users[1])
				&& Contains(all.userDatas, &users[2]) && all.calls == 1,
			"overlap collector reports 3 shapes in one call");
		ShapeCollector withTrigger = OverlapAll(world, big, bigPose, Filter(kAll | Opt::UseTriggerShapes));
		Check(withTrigger.userDatas.size() == 4 && Contains(withTrigger.userDatas, &users[3]), "overlap collector with trigger reports 4");
		ShapeCollector layered = OverlapAll(world, big, bigPose, Filter(kAll, 1 | 4));
		Check(layered.userDatas.size() == 2 && !Contains(layered.userDatas, &users[1]), "overlap collector honors the layer mask");
		ShapeCollector none = OverlapAll(world, SphereGeom(0.5f), Physics::Pose(0, 50, 0, 0, 0, 0, 1), f);
		Check(none.calls == 0, "overlap collector not called without hits");

		BroadPhaseCollector broad;
		qd::OverlapWithBroadphaseCollector bp{};
		bp.geometry = big;
		bp.pose = bigPose;
		bp.filter = f;
		bp.dataContext = &broad;
		bp.collector = &BroadPhaseCollector::Callback;
		Run(world, Context(world, nullptr), Query::OverlapWithBroadphaseCollector, bp);
		Check(broad.entries.size() == 3, "broadphase collector reports 3 shapes");
		bool handlesOk = true;
		for (const BroadPhaseCollector::Entry& e : broad.entries)
		{
			if (e.userData == &users[0])
			{
				handlesOk = handlesOk && e.shape == box.shape && e.body == box.body && e.type == Physics::GeometryType::Box;
			}
			else if (e.userData == &users[1])
			{
				handlesOk = handlesOk && e.shape == sphere.shape && e.body == sphere.body && e.type == Physics::GeometryType::Sphere;
			}
			else if (e.userData == &users[2])
			{
				handlesOk = handlesOk && e.shape == capsule.shape && e.body == capsule.body && e.type == Physics::GeometryType::Capsule;
			}
			else
			{
				handlesOk = false;
			}
		}
		Check(handlesOk, "broadphase collector handles, user data and types");
		BroadPhaseCollector stopped;
		stopped.stopAfter = 1;
		bp.dataContext = &stopped;
		Run(world, Context(world, nullptr), Query::OverlapWithBroadphaseCollector, bp);
		Check(stopped.entries.size() == 1, "broadphase collector stops when the callback returns false");
	}

	// ---- (c) ShapeCast ----
	std::printf("[c] shape casts\n");
	{
		const Physics::QueryFilterData f = Filter(kAll);
		qd::ShapeCast r = ShapeCast(world, SphereGeom(0.5f), Physics::Pose(0, 0, 0, 0, 0, 0, 1), kPlusX, 100.0f, f);
		Check(r.hitCount == 1 && Near(r.hit.distance, 8.5f, 1e-3f), "sphere cast hits the box at 8.5");
		Check(Near(r.hit.normal.x, -1.0f, 1e-3f) && Near(r.hit.point.x, 9.0f, 1e-3f) && IdOf(r.hit) == 101, "sphere cast normal/point/object");
		r = ShapeCast(world, BoxGeom(0.5f, 0.5f, 0.5f), Physics::Pose(0, 0, 0, 0, 0, 0, 1), kPlusX, 100.0f, f);
		// box-vs-box sweeps in PhysX stop ~2.5e-3 early, hence the looser tolerance
		Check(r.hitCount == 1 && Near(r.hit.distance, 8.5f, 1e-2f), "box cast hits the box at 8.5");
		r = ShapeCast(world, CapsuleGeom(0.5f, 2.0f), Physics::Pose(0, 0, 0, 0, 0, 0, 1), kPlusX, 100.0f, f);
		Check(r.hitCount == 1 && Near(r.hit.distance, 8.0f, 1e-3f), "capsule cast hits the box at 8.0");
		r = ShapeCast(world, SphereGeom(0.5f), Physics::Pose(0, 0, 0, 0, 0, 0, 1), kPlusX, 5.0f, f);
		Check(r.hitCount == 0, "sphere cast shorter than the gap");
		r = ShapeCast(world, SphereGeom(0.5f), Physics::Pose(0, 0, 0, 0, 0, 0, 1), kPlusX, 100.0f, Filter(kAll, 2));
		Check(r.hitCount == 1 && Near(r.hit.distance, 18.5f, 1e-3f) && IdOf(r.hit) == 102, "sphere cast layer mask");
		r = ShapeCast(world, SphereGeom(0.5f), Physics::Pose(0, 0, 0, 0, 0, 0, 1), kPlusX, std::numeric_limits<float>::infinity(), f);
		Check(r.hitCount == 1 && Near(r.hit.distance, 8.5f, 1e-3f), "sphere cast with infinite distance");
		r = ShapeCast(world, SphereGeom(0.5f), Physics::Pose(0, 0, 0, 0, 0, 0, 1), kPlusX, 0.0f, f);
		Check(r.hitCount == 0, "zero distance cast is no hit");

		// start inside the box: the initial overlap is discarded and the cast continues to the sphere
		r = ShapeCast(world, SphereGeom(0.5f), Physics::Pose(10, 0, 0, 0, 0, 0, 1), kPlusX, 100.0f, f);
		Check(r.hitCount == 1 && Near(r.hit.distance, 8.5f, 1e-3f) && IdOf(r.hit) == 102, "initial overlap discarded, next hit found");

		HitCollector discard = ShapeCastAll(world, SphereGeom(0.5f), Physics::Pose(10, 0, 0, 0, 0, 0, 1), kPlusX, 100.0f, f, false);
		Check(discard.hits.size() == 2, "collector without keepInitialOverlaps: 2 hits");
		HitCollector keep = ShapeCastAll(world, SphereGeom(0.5f), Physics::Pose(10, 0, 0, 0, 0, 0, 1), kPlusX, 100.0f, f, true);
		Check(keep.hits.size() == 3, "collector with keepInitialOverlaps: 3 hits");
		if (keep.hits.size() == 3)
		{
			Check(Near(keep.hits[0].distance, 0.0f, 1e-4f) && IdOf(keep.hits[0]) == 101 && Near(keep.hits[0].normal.x, -1.0f, 1e-3f),
				"initial overlap hit: distance 0, normal opposite to the cast");
			Check(Near(keep.hits[1].distance, 8.5f, 1e-3f) && Near(keep.hits[2].distance, 17.5f, 1e-3f), "remaining hit distances");
		}
		HitCollector zero = ShapeCastAll(world, SphereGeom(0.5f), Physics::Pose(10, 0, 0, 0, 0, 0, 1), kPlusX, 0.0f, f, true);
		Check(zero.hits.size() == 1 && IdOf(zero.hits[0]) == 101, "zero distance keeps the initial overlap");
		HitCollector zeroDiscard = ShapeCastAll(world, SphereGeom(0.5f), Physics::Pose(10, 0, 0, 0, 0, 0, 1), kPlusX, 0.0f, f, false);
		Check(zeroDiscard.calls == 0, "zero distance without keepInitialOverlaps reports nothing");
		HitCollector ahead = ShapeCastAll(world, SphereGeom(0.5f), Physics::Pose(0, 0, 0, 0, 0, 0, 1), kPlusX, 100.0f, f, false);
		Check(ahead.hits.size() == 3 && Near(ahead.hits[0].distance, 8.5f, 1e-3f) && Near(ahead.hits[2].distance, 27.5f, 1e-3f),
			"shape cast collector reports every hit sorted");
		HitCollector stopper;
		stopper.keepGoing = false;
		qd::ShapeCastWithCollector cmd{};
		cmd.geometry = SphereGeom(0.5f);
		cmd.pose = Physics::Pose(0, 0, 0, 0, 0, 0, 1);
		cmd.filter = f;
		cmd.direction = kPlusX;
		cmd.maxDistance = 100.0f;
		cmd.dataContext = &stopper;
		cmd.collector = &HitCollector::Callback;
		Run(world, Context(world, nullptr), Query::ShapeCastWithCollector, cmd);
		Check(stopper.calls == 1, "collector returning false is not called again");
	}

	// ---- (d) BodyCast ----
	std::printf("[d] body casts\n");
	{
		const Physics::QueryFilterData f = Filter(kAll);
		qd::BodyCast bc{};
		bc.direction = kPlusZ;
		bc.maxDistance = 100.0f;
		bc.filter = f;
		bc.body = compound;
		Run(world, Context(world, compound), Query::BodyCast, bc);
		Check(bc.hitCount == 1 && Near(bc.hit.distance, 8.5f, 1e-2f) && IdOf(bc.hit) == 105,
			"body cast uses the second shape (box at local y=5) and hits obstacle 1 at 8.5");

		bc = qd::BodyCast{};
		bc.direction = kPlusZ;
		bc.maxDistance = 5.0f;
		bc.filter = f;
		bc.body = compound;
		Run(world, Context(world, compound), Query::BodyCast, bc);
		Check(bc.hitCount == 0, "body cast shorter than the gap");

		HitCollector col;
		qd::BodyCastWithCollector bcc{};
		bcc.direction = kPlusZ;
		bcc.maxDistance = 100.0f;
		bcc.filter = f;
		bcc.body = compound;
		bcc.dataContext = &col;
		bcc.collector = &HitCollector::Callback;
		Run(world, Context(world, compound), Query::BodyCastWithCollector, bcc);
		Check(col.hits.size() == 2, "body cast collector reports both obstacles");
		if (col.hits.size() == 2)
		{
			Check(Near(col.hits[0].distance, 8.5f, 1e-2f) && IdOf(col.hits[0]) == 105 && Near(col.hits[1].distance, 18.5f, 1e-3f)
					&& IdOf(col.hits[1]) == 106,
				"body cast collector distances and objects");
		}
		HitCollector colMask;
		bcc.filter = Filter(kAll, 2);
		bcc.dataContext = &colMask;
		Run(world, Context(world, compound), Query::BodyCastWithCollector, bcc);
		Check(colMask.calls == 0, "body cast collector honors the layer mask");

		// the body is not hit by itself even when sweeping through its own shapes
		bc = qd::BodyCast{};
		bc.direction = Physics::Vec3(0, 1, 0);
		bc.maxDistance = 20.0f;
		bc.filter = f;
		bc.body = compound;
		Run(world, Context(world, compound), Query::BodyCast, bc);
		Check(bc.hitCount == 0, "body cast ignores the casting body itself");
	}

	// ---- (e) AgainstShape ----
	std::printf("[e] against-shape queries\n");
	{
		qd::RayCastAgainstShape r = RayAgainst(world, sphere.shape, kOrigin, kPlusX, 100.0f, kAll);
		Check(r.hitCount == 1 && Near(r.hit.distance, 19.0f, 1e-3f) && IdOf(r.hit) == 102, "ray against the sphere skips the box in front");
		r = RayAgainst(world, sphere.body, kOrigin, kPlusX, 100.0f, kAll);
		Check(r.hitCount == 1 && Near(r.hit.distance, 19.0f, 1e-3f), "ray against a body handle uses its shapes");
		r = RayAgainst(world, sphere.shape, kOrigin, Physics::Vec3(-1, 0, 0), 100.0f, kAll);
		Check(r.hitCount == 0, "ray against the sphere pointing away");
		r = RayAgainst(world, sphere.shape, kOrigin, kPlusX, 10.0f, kAll);
		Check(r.hitCount == 0, "ray against the sphere out of range");
		r = RayAgainst(world, sphere.shape, kOrigin, kPlusX, 100.0f, Opt::UseStaticBodies);
		Check(r.hitCount == 0, "ray against a dynamic shape with static-only options");
		r = RayAgainst(world, trigger.shape, kOrigin, kPlusX, 100.0f, kAll);
		Check(r.hitCount == 0, "ray against a trigger without UseTriggerShapes");
		r = RayAgainst(world, trigger.shape, kOrigin, kPlusX, 100.0f, kAll | Opt::UseTriggerShapes);
		Check(r.hitCount == 1 && Near(r.hit.distance, 39.0f, 1e-3f), "ray against a trigger with UseTriggerShapes");

		const Physics::Pose start(0, 0, 0, 0, 0, 0, 1);
		qd::ShapeCastAgainstShape s = ShapeAgainst(world, capsule.shape, SphereGeom(0.5f), start, kPlusX, 100.0f, Filter(kAll), false);
		Check(s.hitCount == 1 && Near(s.hit.distance, 27.5f, 1e-3f) && IdOf(s.hit) == 103, "sphere cast against the capsule only");
		s = ShapeAgainst(world, sphere.shape, SphereGeom(0.5f), start, kPlusX, 100.0f, Filter(kAll), false);
		Check(s.hitCount == 1 && Near(s.hit.distance, 18.5f, 1e-3f), "sphere cast against the sphere only");
		s = ShapeAgainst(world, sphere.shape, SphereGeom(0.5f), start, kPlusX, 100.0f, Filter(kAll, 1), false);
		Check(s.hitCount == 0, "cast against a shape excluded by the layer mask");
		s = ShapeAgainst(world, sphere.shape, SphereGeom(0.5f), start, Physics::Vec3(-1, 0, 0), 100.0f, Filter(kAll), false);
		Check(s.hitCount == 0, "cast against the sphere pointing away");

		const Physics::Pose inside(19.5f, 0, 0, 0, 0, 0, 1);
		s = ShapeAgainst(world, sphere.shape, SphereGeom(0.5f), inside, kPlusX, 10.0f, Filter(kAll), false);
		Check(s.hitCount == 0, "initial overlap is discarded without distanceAsDepth");
		s = ShapeAgainst(world, sphere.shape, SphereGeom(0.5f), inside, kPlusX, 10.0f, Filter(kAll), true);
		Check(s.hitCount == 1 && Near(s.hit.distance, 1.0f, 1e-2f), "distanceAsDepth reports the penetration depth");
		s = ShapeAgainst(world, sphere.shape, SphereGeom(0.5f), inside, kPlusX, 0.0f, Filter(kAll), true);
		Check(s.hitCount == 1 && Near(s.hit.distance, 1.0f, 1e-2f), "distanceAsDepth works with zero distance");
		s = ShapeAgainst(world, sphere.shape, SphereGeom(0.5f), start, kPlusX, 100.0f, Filter(kAll), true);
		Check(s.hitCount == 1 && Near(s.hit.distance, 18.5f, 1e-3f), "distanceAsDepth without overlap keeps the sweep distance");
	}

	// ---- (f) penetration / closest point ----
	std::printf("[f] penetration and closest point\n");
	{
		qd::ComputeShapePenetration p = Penetration(sphere.shape, box.shape, Physics::Pose(0, 0, 0, 0, 0, 0, 1), Physics::Pose(1.5f, 0, 0, 0, 0, 0, 1));
		Check(p.areOverlapping && Near(p.outDistance, 0.5f, 1e-3f) && Near(p.outDirection.x, -1.0f, 1e-3f),
			"sphere vs box: depth 0.5, A moves along -X");
		p = Penetration(box.shape, sphere.shape, Physics::Pose(0, 0, 0, 0, 0, 0, 1), Physics::Pose(1.5f, 0, 0, 0, 0, 0, 1));
		Check(p.areOverlapping && Near(p.outDistance, 0.5f, 1e-3f) && Near(p.outDirection.x, -1.0f, 1e-3f),
			"box vs sphere: depth 0.5, A moves along -X");
		p = Penetration(box.shape, box.shape, Physics::Pose(0, 0, 0, 0, 0, 0, 1), Physics::Pose(1.5f, 0, 0, 0, 0, 0, 1));
		Check(!p.areOverlapping, "same shape twice is not a pair");
		const Handle otherBox = CreateShape(world, BoxGeom(1, 1, 1), material, nullptr, 1, false);
		p = Penetration(box.shape, otherBox, Physics::Pose(0, 0, 0, 0, 0, 0, 1), Physics::Pose(1.5f, 0, 0, 0, 0, 0, 1));
		Check(p.areOverlapping && Near(p.outDistance, 0.5f, 1e-3f) && Near(p.outDirection.x, -1.0f, 1e-3f), "box vs loose box: depth 0.5");
		p = Penetration(box.shape, otherBox, Physics::Pose(0, 0, 0, 0, 0, 0, 1), Physics::Pose(5.0f, 0, 0, 0, 0, 0, 1));
		Check(!p.areOverlapping && p.outDistance == 0.0f, "separated shapes do not overlap");

		qd::ClosestPointOnShape cp = ClosestPoint(box.shape, Physics::Pose(10, 0, 0, 0, 0, 0, 1), Physics::Vec3(15, 0.5f, 0.2f));
		Check(Near(cp.outPoint.x, 11.0f, 1e-3f) && Near(cp.outPoint.y, 0.5f, 1e-3f) && Near(cp.outPoint.z, 0.2f, 1e-3f) && Near(cp.outDistance, 4.0f, 1e-3f),
			"closest point on the box");
		cp = ClosestPoint(sphere.shape, Physics::Pose(20, 0, 0, 0, 0, 0, 1), Physics::Vec3(20, 5, 0));
		Check(Near(cp.outPoint.y, 1.0f, 1e-3f) && Near(cp.outPoint.x, 20.0f, 1e-3f) && Near(cp.outDistance, 4.0f, 1e-3f), "closest point on the sphere");
		cp = ClosestPoint(capsule.shape, Physics::Pose(0, 0, 0, 0, 0, 0, 1), Physics::Vec3(0, 3, 0));
		Check(Near(cp.outPoint.y, 1.0f, 1e-3f) && Near(cp.outDistance, 2.0f, 1e-3f), "closest point on the capsule");
		cp = ClosestPoint(box.shape, Physics::Pose(10, 0, 0, 0, 0, 0, 1), Physics::Vec3(10.2f, 0, 0));
		Check(Near(cp.outPoint.x, 10.2f, 1e-4f) && cp.outDistance == 0.0f, "point inside the shape is returned as is");
	}

	// ---- (g) invalid handles / unsupported geometry ----
	std::printf("[g] invalid input\n");
	{
		void* const bogus = reinterpret_cast<void*>(0x1234);
		const Physics::QueryFilterData f = Filter(kAll);

		qd::RayCast rc{};
		rc.origin = kOrigin;
		rc.direction = kPlusX;
		rc.maxDistance = 100.0f;
		rc.filter = f;
		rc.hitCount = 7;
		Run(bogus, Context(bogus, nullptr), Query::RayCast, rc);
		Check(rc.hitCount == 0, "ray cast with a bogus world handle");
		Run(box.shape, Context(), Query::RayCast, rc);
		Check(rc.hitCount == 0, "ray cast with a shape handle as world");
		Run(nullptr, Context(), Query::RayCast, rc);
		Check(rc.hitCount == 0, "ray cast with null handles");

		HitCollector col;
		qd::RaycastWithCollector rcc{};
		rcc.origin = kOrigin;
		rcc.direction = kPlusX;
		rcc.maxDistance = 100.0f;
		rcc.filter = f;
		rcc.dataContext = &col;
		rcc.collector = &HitCollector::Callback;
		Run(bogus, Context(bogus, nullptr), Query::RayCastWithCollector, rcc);
		rcc.collector = nullptr;
		Run(world, Context(world, nullptr), Query::RayCastWithCollector, rcc);
		Check(col.calls == 0, "collector queries with bogus world / null callback");

		Check(OverlapFirst(bogus, SphereGeom(1.0f), Physics::Pose(10, 0, 0, 0, 0, 0, 1), f) == nullptr, "overlap with a bogus world");
		Check(OverlapAll(bogus, SphereGeom(1.0f), Physics::Pose(10, 0, 0, 0, 0, 0, 1), f).calls == 0, "overlap collector with a bogus world");

		qd::ShapeCast sc = ShapeCast(bogus, SphereGeom(0.5f), Physics::Pose(), kPlusX, 10.0f, f);
		Check(sc.hitCount == 0, "shape cast with a bogus world");

		qd::RayCastAgainstShape ra = RayAgainst(world, bogus, kOrigin, kPlusX, 100.0f, kAll);
		Check(ra.hitCount == 0, "ray against a bogus shape");
		ra = RayAgainst(world, nullptr, kOrigin, kPlusX, 100.0f, kAll);
		Check(ra.hitCount == 0, "ray against a null shape");
		qd::ShapeCastAgainstShape sa = ShapeAgainst(world, bogus, SphereGeom(0.5f), Physics::Pose(), kPlusX, 100.0f, f, false);
		Check(sa.hitCount == 0, "shape cast against a bogus shape");
		qd::ComputeShapePenetration pen = Penetration(bogus, box.shape, Physics::Pose(), Physics::Pose());
		Check(!pen.areOverlapping, "penetration with a bogus shape");
		pen = Penetration(nullptr, nullptr, Physics::Pose(), Physics::Pose());
		Check(!pen.areOverlapping, "penetration with null shapes");
		qd::ClosestPointOnShape cp = ClosestPoint(bogus, Physics::Pose(), Physics::Vec3(1, 2, 3));
		Check(Near(cp.outPoint.x, 1.0f, 1e-6f) && cp.outDistance == 0.0f, "closest point on a bogus shape returns the input point");

		qd::BodyCast bc{};
		bc.direction = kPlusZ;
		bc.maxDistance = 100.0f;
		bc.filter = f;
		bc.body = bogus;
		bc.hitCount = 5;
		Run(world, Context(world, nullptr), Query::BodyCast, bc);
		Check(bc.hitCount == 0, "body cast with a bogus body");
		bc.body = nullptr;
		Run(world, Context(world, nullptr), Query::BodyCast, bc);
		Check(bc.hitCount == 0, "body cast with a null body");

		// unsupported query geometry
		const Physics::GeometryType unsupported[] = { Physics::GeometryType::ConvexMesh, Physics::GeometryType::TriangleMesh,
			Physics::GeometryType::Terrain, Physics::GeometryType::Invalid };
		bool unsupportedOk = true;
		for (const Physics::GeometryType type : unsupported)
		{
			const Physics::ShapeGeometry g = MeshGeom(type);
			unsupportedOk = unsupportedOk && OverlapFirst(world, g, Physics::Pose(10, 0, 0, 0, 0, 0, 1), f) == nullptr;
			unsupportedOk = unsupportedOk && OverlapAll(world, g, Physics::Pose(10, 0, 0, 0, 0, 0, 1), f).calls == 0;
			unsupportedOk = unsupportedOk && ShapeCast(world, g, Physics::Pose(), kPlusX, 100.0f, f).hitCount == 0;
			unsupportedOk = unsupportedOk && ShapeCastAll(world, g, Physics::Pose(), kPlusX, 100.0f, f, true).calls == 0;
			unsupportedOk = unsupportedOk && ShapeAgainst(world, box.shape, g, Physics::Pose(), kPlusX, 100.0f, f, false).hitCount == 0;
		}
		Check(unsupportedOk, "unsupported query geometry reports no hit");

		// destroyed shapes and bodies
		WorldData::DestroyBody db{};
		db.value = capsule.body;
		Run(world, Context(world, nullptr), World::DestroyBody, db);
		RayResult r = RayCast(world, kOrigin, kPlusX, 100.0f, Filter(kAll, 4));
		Check(r.count == 0, "destroyed body is not hit");
		ra = RayAgainst(world, capsule.shape, kOrigin, kPlusX, 100.0f, kAll);
		Check(ra.hitCount <= 1, "ray against the former shape handle does not crash");
	}

	WorldData::DestroyWorld dw{};
	Run(world, Context(world, nullptr), World::DestroyWorld, dw);
	// every query on the destroyed world is a no-op
	Check(RayCast(world, kOrigin, kPlusX, 100.0f, Filter(kAll)).count == 0, "ray cast on a destroyed world");

	(void)obstacle1;
	(void)obstacle2;

	SdkData::ShutdownSdk shutdown;
	Run(nullptr, Context(), SDK::ShutdownSdk, shutdown);
	unload();
	dlclose(lib);

	Check(g_errorLogs == 0, "no backend error logs");
	std::printf("query_test: %d checks, %d failures, %d error logs -> %s\n", g_checks, g_failures, g_errorLogs,
		g_failures == 0 ? "PASS" : "FAIL");
	return g_failures == 0 ? 0 : 1;
}
