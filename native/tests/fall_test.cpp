// Drives the backend through the Unity command entry point: rigid bodies + box/sphere/capsule
// colliders fall under gravity onto a static floor. Usage: fall_test [path/to/libBackend.so]

#include <dlfcn.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
#include "PhysicsJointCommandData.h"
#include "PhysicsArticulationCommandData.h"

#ifndef FALL_TEST_BACKEND_PATH
#define FALL_TEST_BACKEND_PATH "libPhysX5Backend.so"
#endif
#ifndef FALL_TEST_SDK
#define FALL_TEST_SDK 0
#endif

using namespace PhysicsCommands;
using Handle = Physics::SDKObjectHandle;

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
		else if (std::getenv("FALL_TEST_VERBOSE") != nullptr)
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

	// ---- thin command wrappers ----

	struct Dummy
	{
		uint64_t id;
	};

	Handle CreateWorld(float gravityY, uint32_t workers = 2)
	{
		WorldData::CreateWorld cw;
		cw.jobWorkerCount = workers;
		cw.gravity = Physics::Vec3(0.0f, gravityY, 0.0f);
		Run(nullptr, Context(), World::CreateWorld, cw);
		return cw.outWorld;
	}

	void Simulate(Handle world, float dt)
	{
		WorldData::Simulate sim;
		sim.deltaTime = dt;
		Run(world, Context(world, nullptr), World::Simulate, sim);
		Check(sim.ranSimulation, "Simulate ran");
	}

	Handle CreateBody(Handle world, bool isStatic, const Physics::Pose& pose, float mass, Physics::BodyFlags flags)
	{
		WorldData::CreateBody cb{};
		cb.pose = pose;
		cb.mass = mass;
		cb.angularDamping = 0.05f;
		cb.flags = flags;
		cb.isStatic = isStatic;
		Run(world, Context(world, nullptr), World::CreateBody, cb);
		return cb.outBody;
	}

	Handle CreateShape(Handle world, const Physics::ShapeGeometry& geom, Handle material, void* userData, const Physics::FilterData& filter)
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
			sf.data = filter;
			Run(cs.value, Context(world, nullptr), Shape::SetFilterData, sf);
		}
		return cs.value;
	}

	void AttachShape(Handle world, Handle body, Handle shape)
	{
		BodyData::AttachShape as{};
		as.shape = shape;
		as.pose = Physics::Pose();
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

	Physics::Pose GetPose(Handle world, Handle body)
	{
		BodyData::GetPose gp{};
		Run(body, Context(world, body), Body::GetPose, gp);
		return gp.value;
	}

	Physics::Vec3 GetLinearVelocity(Handle world, Handle body)
	{
		BodyData::GetLinearVelocity gv{};
		Run(body, Context(world, body), Body::GetLinearVelocity, gv);
		return gv.value;
	}

	float Length(const Physics::Vec3& v)
	{
		return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	}

	Physics::FilterData MakeFilter(uint32_t belongsTo, uint32_t collidesWith, uint16_t ignoreId = 0)
	{
		Physics::FilterData f;
		f.belongsTo = belongsTo;
		f.collidesWith = collidesWith;
		f.ignorePairID = ignoreId;
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

	struct Faller
	{
		const char* name;
		Handle body = nullptr;
		Handle shape = nullptr;
		float expectRestY = 1.0f;
		bool collides = true;
	};

	void PrintRow(Handle world, const char* label, std::vector<Faller>& fallers)
	{
		std::printf("  %-8s", label);
		for (const Faller& f : fallers)
		{
			std::printf(" %s y=%8.4f", f.name, GetPose(world, f.body).pos.y);
		}
		std::printf("\n");
	}
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
	std::printf("fall_test: backend=%s (requested SDK %d)\n", path.c_str(), FALL_TEST_SDK);

	// ---- SDK / material ----
	SdkData::InitializeSdk init;
	init.engineObjectIdOffset = 0;
	Run(nullptr, Context(), SDK::InitializeSdk, init);
	Check(init.result, "InitializeSdk");
	SdkData::GetSdkIntegrationInfo info;
	Run(nullptr, Context(), SDK::GetIntegrationInfo, info);
	std::printf("  integration: %s sdk=%u.%u.%u\n", info.value.name, info.value.sdkVersion[0], info.value.sdkVersion[1], info.value.sdkVersion[2]);

	SdkData::SetErrorVerbosityLevel ev;
	ev.value = Physics::ErrorVerbosityLevel::Verbose;
	Run(nullptr, Context(), SDK::SetErrorVerbosityLevel, ev);

	MaterialData::GetDefaultMaterial dm{};
	Run(nullptr, Context(), SDK::GetDefaultMaterial, dm);
	Check(dm.value != nullptr, "GetDefaultMaterial");
	Handle defaultMaterial = dm.value;

	MaterialData::SetMaterialConfiguration smc{};
	smc.dynamicFriction = 0.6f;
	smc.staticFriction = 0.6f;
	smc.bounciness = 0.0f;
	Run(defaultMaterial, Context(), SDK::SetMaterialConfiguration, smc);
	MaterialData::SetMaterialCombineModes scm{};
	scm.friction = Physics::MaterialCombineMode::Average;
	scm.bounciness = Physics::MaterialCombineMode::Average;
	Run(defaultMaterial, Context(), SDK::SetMaterialCombineModes, scm);
	MaterialData::SetImprovedPatchFrictionEnabledGlobal sip{};
	sip.value = true;
	Run(nullptr, Context(), SDK::SetImprovedPatchFrictionEnabledGlobal, sip);

	// material round trip on a fresh material
	MaterialData::CreateMaterial cm{};
	Run(nullptr, Context(), SDK::CreateMaterial, cm);
	Check(cm.outMaterial != nullptr && cm.outMaterial != defaultMaterial, "CreateMaterial");
	smc.dynamicFriction = 0.5f;
	smc.staticFriction = 0.4f;
	smc.bounciness = 0.25f;
	Run(cm.outMaterial, Context(), SDK::SetMaterialConfiguration, smc);
	MaterialData::GetMaterialConfiguration gmc{};
	Run(cm.outMaterial, Context(), SDK::GetMaterialConfiguration, gmc);
	Check(Near(gmc.dynamicFriction, 0.5f, 1e-5f) && Near(gmc.staticFriction, 0.4f, 1e-5f) && Near(gmc.bounciness, 0.25f, 1e-5f),
		"material configuration round trip");
	scm.friction = Physics::MaterialCombineMode::Maximum;
	scm.bounciness = Physics::MaterialCombineMode::Multiply;
	Run(cm.outMaterial, Context(), SDK::SetMaterialCombineModes, scm);
	MaterialData::GetMaterialCombineModes gcm{};
	Run(cm.outMaterial, Context(), SDK::GetMaterialCombineModes, gcm);
	Check(gcm.friction == Physics::MaterialCombineMode::Maximum && gcm.bounciness == Physics::MaterialCombineMode::Multiply,
		"material combine modes round trip");

	// ---- world ----
	Handle world = CreateWorld(0.0f);
	Check(world != nullptr, "CreateWorld");
	WorldData::SetGravity sg;
	sg.value = Physics::Vec3(0.0f, -9.81f, 0.0f);
	Run(world, Context(world, nullptr), World::SetGravity, sg);
	WorldData::GetGravity gg{};
	Run(world, Context(world, nullptr), World::GetGravity, gg);
	Check(gg.value.y == -9.81f, "gravity round trip");
	WorldData::SetBounceThreshold sbt;
	sbt.value = 2.0f;
	Run(world, Context(world, nullptr), World::SetBounceThreshold, sbt);
	WorldData::GetBounceThreshold gbt{};
	Run(world, Context(world, nullptr), World::GetBounceThreshold, gbt);
	Check(gbt.value == 2.0f, "bounce threshold round trip");
	WorldData::SetScratchBufferChunkCount ssc;
	ssc.value = 4;
	Run(world, Context(world, nullptr), World::SetScratchBufferChunkCount, ssc);

	const Physics::BodyFlags dynamicFlags = Physics::BodyFlags::Gravity | Physics::BodyFlags::Simulated;

	// static floor: top surface at y = 0.5
	Dummy floorUser{ 100 };
	Handle floorBody = CreateBody(world, true, Physics::Pose(0, 0, 0, 0, 0, 0, 1), 0.0f, Physics::BodyFlags::Simulated);
	Handle floorShape = CreateShape(world, BoxGeom(50.0f, 0.5f, 50.0f), defaultMaterial, &floorUser, MakeFilter(1, 0xFFFFFFFFu, 6));
	Check(floorBody != nullptr && floorShape != nullptr, "floor created");
	AttachShape(world, floorBody, floorShape);
	AddToScene(world, floorBody);

	ShapeData::GetActorDescriptor gad{};
	Run(floorShape, Context(world, floorBody), Shape::GetActorDescriptor, gad);
	Check(gad.value.actor == floorBody && gad.value.type == Physics::ActorType::Static, "floor actor descriptor");
	ShapeData::GetWorldBounds gwb{};
	Run(floorShape, Context(world, floorBody), Shape::GetWorldBounds, gwb);
	Check(Near(gwb.extents.x, 50.0f, 1e-2f) && Near(gwb.extents.y, 0.5f, 1e-2f) && Near(gwb.center.y, 0.0f, 1e-3f), "floor world bounds");

	Dummy users[8] = { { 1 }, { 2 }, { 3 }, { 4 }, { 5 }, { 6 }, { 7 }, { 8 } };
	std::vector<Faller> fallers(5);
	fallers[0] = { "box", nullptr, nullptr, 1.0f, true };
	fallers[1] = { "sphere", nullptr, nullptr, 1.0f, true };
	fallers[2] = { "capsule", nullptr, nullptr, 1.0f, true };
	fallers[3] = { "ghost", nullptr, nullptr, 0.0f, false };
	fallers[4] = { "ignored", nullptr, nullptr, 0.0f, false };
	const float xs[5] = { -4.0f, 0.0f, 4.0f, 8.0f, -10.0f };
	const Physics::ShapeGeometry geoms[5] = { BoxGeom(0.5f, 0.5f, 0.5f), SphereGeom(0.5f), CapsuleGeom(0.5f, 2.0f), BoxGeom(0.5f, 0.5f, 0.5f), BoxGeom(0.5f, 0.5f, 0.5f) };
	const Physics::FilterData filters[5] = { MakeFilter(1, 0xFFFFFFFFu), MakeFilter(1, 0xFFFFFFFFu), MakeFilter(1, 0xFFFFFFFFu),
		MakeFilter(2, 2), MakeFilter(1, 0xFFFFFFFFu, 5) };

	for (size_t i = 0; i < fallers.size(); ++i)
	{
		Faller& f = fallers[i];
		f.body = CreateBody(world, false, Physics::Pose(xs[i], 5.0f, 0.0f, 0, 0, 0, 1), 1.0f, dynamicFlags);
		f.shape = CreateShape(world, geoms[i], defaultMaterial, &users[i], filters[i]);
		Check(f.body != nullptr && f.shape != nullptr, "faller created");

		// Unity-like configuration traffic
		BodyData::SetDamping sd;
		sd.linear = 0.0f;
		sd.angular = 0.05f;
		Run(f.body, Context(world, f.body), Body::SetDamping, sd);
		BodyData::SetSolverIterationsOverride si;
		si.positionIterations = 6;
		si.velocityIterations = 1;
		Run(f.body, Context(world, f.body), Body::SetSolverIterationsOverride, si);
		BodyData::SetSleepThresholdOverride st;
		st.value = 0.005f;
		Run(f.body, Context(world, f.body), Body::SetSleepThresholdOverride, st);
		BodyData::SetMaxDepenetrationVelocity md;
		md.value = 10.0f;
		Run(f.body, Context(world, f.body), Body::SetMaxDepenetrationVelocity, md);
		ShapeData::SetContactOffset co;
		co.value = 0.01f;
		Run(f.shape, Context(world, f.body), Shape::SetContactOffset, co);

		AttachShape(world, f.body, f.shape);
		BodyData::SetMass sm;
		sm.value = 1.0f;
		Run(f.body, Context(world, f.body), Body::SetMass, sm);
		Recompute(world, f.body);
		AddToScene(world, f.body);
	}

	// (d2) IgnoreCollision between the "ignored" body (id 5) and the floor (id 6)
	ShapeData::ShouldIgnoreCollision sic{};
	sic.ignoreId0 = 5;
	sic.ignoreId1 = 6;
	Run(nullptr, Context(), Shape::ShouldIgnoreCollision, sic);
	Check(!sic.value, "no ignore pair initially");
	ShapeData::IgnoreCollision ic{};
	ic.ignoreId0 = 6;
	ic.ignoreId1 = 5;
	ic.value = true;
	Run(nullptr, Context(), Shape::IgnoreCollision, ic);
	Run(nullptr, Context(), Shape::ShouldIgnoreCollision, sic);
	Check(sic.value, "ignore pair set");

	// shape bookkeeping
	BodyData::GetShapeCount gsc{};
	Run(fallers[0].body, Context(world, fallers[0].body), Body::GetShapeCount, gsc);
	Check(gsc.value == 1, "shape count");
	Handle shapeBuf[4] = {};
	BodyData::GetShapes gs{};
	gs.buffer = shapeBuf;
	gs.bufferSize = 4;
	Run(fallers[0].body, Context(world, fallers[0].body), Body::GetShapes, gs);
	Check(gs.written == 1 && shapeBuf[0] == fallers[0].shape, "GetShapes");
	void* udBuf[4] = {};
	BodyData::GetShapesUserData gsu{};
	gsu.buffer = udBuf;
	gsu.bufferSize = 4;
	Run(fallers[0].body, Context(world, fallers[0].body), Body::GetShapesUserData, gsu);
	Check(gsu.written == 1 && udBuf[0] == &users[0], "GetShapesUserData");
	BodyData::GetMass gm{};
	Run(fallers[0].body, Context(world, fallers[0].body), Body::GetMass, gm);
	Check(Near(gm.value, 1.0f, 1e-5f), "mass after recompute");
	BodyData::GetFlags gf{};
	Run(fallers[0].body, Context(world, fallers[0].body), Body::GetFlags, gf);
	Check(gf.value == dynamicFlags, "body flags round trip");

	// ---- (a)(b)(d) simulate 120 steps @ 60 Hz ----
	const float dt = 1.0f / 60.0f;
	std::printf("free fall / rest (y of body center):\n");
	std::printf("  analytic y(0.5s)=%.4f y(1.0s)=%.4f (continuous); floor top y=0.5\n", 5.0f - 0.5f * 9.81f * 0.25f, 5.0f - 0.5f * 9.81f);
	for (int step = 1; step <= 120; ++step)
	{
		Simulate(world, dt);
		if (step == 30 || step == 60 || step == 120)
		{
			char label[16];
			std::snprintf(label, sizeof(label), "t=%.2fs", step * dt);
			PrintRow(world, label, fallers);
		}
		if (step == 30)
		{
			for (const Faller& f : fallers)
			{
				Check(Near(GetPose(world, f.body).pos.y, 3.75f, 0.1f), "y at 0.5s follows free fall");
			}
		}
		if (step == 60)
		{
			// (a) bodies that cannot collide with the floor are still in free fall: 5 - 0.5 g t^2 ~ 0.095
			for (const Faller& f : fallers)
			{
				if (!f.collides)
				{
					Check(Near(GetPose(world, f.body).pos.y, 0.095f, 0.2f), "y at 1.0s ~ 0.095 for pass-through body");
				}
			}
		}
	}

	for (const Faller& f : fallers)
	{
		const Physics::Pose pose = GetPose(world, f.body);
		const float speed = Length(GetLinearVelocity(world, f.body));
		if (f.collides)
		{
			// (b) resting on the floor
			Check(Near(pose.pos.y, f.expectRestY, 0.05f), "rests on floor");
			Check(speed < 0.05f, "rest velocity ~ 0");
			Check(Near(pose.pos.x, std::round(pose.pos.x / 2.0f) * 2.0f, 0.5f), "no sideways drift");
		}
		else
		{
			// (d) filter mismatch / ignore pair: passed through the floor and keeps falling
			Check(pose.pos.y < -5.0f, "pass-through body keeps falling");
		}
		std::printf("  final %-8s y=%9.4f speed=%.5f\n", f.name, pose.pos.y, speed);
	}

	BodyData::GetIsSleeping gsl{};
	Run(fallers[1].body, Context(world, fallers[1].body), Body::GetIsSleeping, gsl);
	std::printf("  sphere sleeping=%d\n", gsl.value ? 1 : 0);

	// ---- queries ----
	{
		QueryData::RayCast rc{};
		rc.origin = Physics::Vec3(0.0f, 10.0f, 0.0f);
		rc.direction = Physics::Vec3(0.0f, -1.0f, 0.0f);
		rc.maxDistance = 100.0f;
		rc.filter.options = Physics::QueryFilterFlags::UseDynamicBodies | Physics::QueryFilterFlags::UseStaticBodies;
		rc.filter.mask = -1;
		Run(world, Context(world, nullptr), Query::RayCast, rc);
		const float expected = 10.0f - (fallers[1].expectRestY + 0.5f);
		Check(rc.hitCount == 1 && Near(rc.hit.distance, expected, 0.06f) && rc.hit.userData.data == 2, "raycast hits the resting sphere");
		std::printf("  raycast: hits=%u distance=%.4f (expected %.4f) id=%llu\n", rc.hitCount, rc.hit.distance, expected,
			static_cast<unsigned long long>(rc.hit.userData.data));

		rc.origin = Physics::Vec3(20.0f, 10.0f, 20.0f);
		Run(world, Context(world, nullptr), Query::RayCast, rc);
		Check(rc.hitCount == 1 && Near(rc.hit.distance, 9.5f, 0.01f) && rc.hit.userData.data == 100, "raycast hits the floor");

		rc.filter.options = Physics::QueryFilterFlags::UseDynamicBodies;
		Run(world, Context(world, nullptr), Query::RayCast, rc);
		Check(rc.hitCount == 0, "raycast dynamic-only skips the floor");
	}

	// ---- (c) get/set round trips on a probe in a second world (zero gravity) ----
	{
		Handle probeWorld = CreateWorld(0.0f, 1);
		Check(probeWorld != nullptr && probeWorld != world, "second world");
		const Context pctx(probeWorld, nullptr);

		Dummy probeUser{ 50 };
		Handle probe = CreateBody(probeWorld, false, Physics::Pose(1, 2, 3, 0, 0, 0, 1), 2.0f, dynamicFlags);
		Handle probeShape = CreateShape(probeWorld, BoxGeom(0.5f, 0.5f, 0.5f), defaultMaterial, &probeUser, MakeFilter(4, 4));
		AttachShape(probeWorld, probe, probeShape);
		AddToScene(probeWorld, probe);
		const Context bctx(probeWorld, probe);

		BodyData::SetMass sm;
		sm.value = 3.5f;
		Run(probe, bctx, Body::SetMass, sm);
		Recompute(probeWorld, probe);
		BodyData::GetMass gm2{};
		Run(probe, bctx, Body::GetMass, gm2);
		Check(Near(gm2.value, 3.5f, 1e-4f), "mass round trip");
		BodyData::GetInertiaTensor git{};
		Run(probe, bctx, Body::GetInertiaTensor, git);
		// box 1x1x1, m=3.5: I = m/6 = 0.5833 on every axis
		Check(Near(git.value.x, 3.5f / 6.0f, 1e-3f) && Near(git.value.y, 3.5f / 6.0f, 1e-3f), "inertia from box shape");

		BodyData::SetDamping sd;
		sd.linear = 0.1f;
		sd.angular = 0.2f;
		Run(probe, bctx, Body::SetDamping, sd);
		BodyData::GetDamping gd{};
		Run(probe, bctx, Body::GetDamping, gd);
		Check(Near(gd.linear, 0.1f, 1e-6f) && Near(gd.angular, 0.2f, 1e-6f), "damping round trip");

		BodyData::SetPose sp{};
		sp.value = Physics::Pose(-3, 4, 5, 0, 0.7071068f, 0, 0.7071068f);
		sp.awake = true;
		Run(probe, bctx, Body::SetPose, sp);
		const Physics::Pose gp = GetPose(probeWorld, probe);
		Check(Near(gp.pos.x, -3, 1e-5f) && Near(gp.pos.y, 4, 1e-5f) && Near(gp.pos.z, 5, 1e-5f) && Near(gp.quat.y, 0.7071068f, 1e-4f)
			&& Near(gp.quat.w, 0.7071068f, 1e-4f), "pose round trip");

		BodyData::SetLinearVelocity slv{};
		slv.value = Physics::Vec3(1, 2, 3);
		slv.awake = true;
		Run(probe, bctx, Body::SetLinearVelocity, slv);
		const Physics::Vec3 lv = GetLinearVelocity(probeWorld, probe);
		Check(lv.x == 1.0f && lv.y == 2.0f && lv.z == 3.0f, "linear velocity round trip");
		BodyData::SetAngularVelocity sav{};
		sav.value = Physics::Vec3(0, 0, 1);
		sav.awake = true;
		Run(probe, bctx, Body::SetAngularVelocity, sav);
		BodyData::GetAngularVelocity gav{};
		Run(probe, bctx, Body::GetAngularVelocity, gav);
		Check(gav.value.z == 1.0f, "angular velocity round trip");

		BodyData::SetMaxLinearVelocity sml;
		sml.value = 55.0f;
		Run(probe, bctx, Body::SetMaxLinearVelocity, sml);
		BodyData::GetMaxLinearVelocity gml{};
		Run(probe, bctx, Body::GetMaxLinearVelocity, gml);
		Check(gml.value == 55.0f, "max linear velocity round trip");
		BodyData::GetMaxAngularVelocity gma{};
		Run(probe, bctx, Body::GetMaxAngularVelocity, gma);
		Check(Near(gma.value, 7.0f, 1e-5f), "default max angular velocity 7");
		BodyData::SetSleepThresholdOverride sst;
		sst.value = 0.02f;
		Run(probe, bctx, Body::SetSleepThresholdOverride, sst);
		BodyData::GetSleepThresholdOverride gst{};
		Run(probe, bctx, Body::GetSleepThresholdOverride, gst);
		Check(Near(gst.value, 0.02f, 1e-6f), "sleep threshold round trip");
		BodyData::SetSolverIterationsOverride ssi;
		ssi.positionIterations = 8;
		ssi.velocityIterations = 3;
		Run(probe, bctx, Body::SetSolverIterationsOverride, ssi);
		BodyData::GetSolverIterationsOverride gsi{};
		Run(probe, bctx, Body::GetSolverIterationsOverride, gsi);
		Check(gsi.positionIterations == 8 && gsi.velocityIterations == 3, "solver iterations round trip");
		BodyData::SetMaxDepenetrationVelocity smd;
		smd.value = 3.0f;
		Run(probe, bctx, Body::SetMaxDepenetrationVelocity, smd);
		BodyData::GetMaxDepenetrationVelocity gmd{};
		Run(probe, bctx, Body::GetMaxDepenetrationVelocity, gmd);
		Check(gmd.value == 3.0f, "max depenetration velocity round trip");
		BodyData::SetLocalCenterOfMass slc;
		slc.value = Physics::Vec3(0.1f, 0.2f, 0.3f);
		Run(probe, bctx, Body::SetLocalCenterOfMass, slc);
		BodyData::GetLocalCenterOfMass glc{};
		Run(probe, bctx, Body::GetLocalCenterOfMass, glc);
		Check(Near(glc.value.x, 0.1f, 1e-6f) && Near(glc.value.z, 0.3f, 1e-6f), "center of mass round trip");
		slc.value = Physics::Vec3(0, 0, 0);
		Run(probe, bctx, Body::SetLocalCenterOfMass, slc);

		// flags: gravity off, kinematic toggle
		BodyData::SetFlags sf{};
		sf.value = Physics::BodyFlags::Gravity;
		sf.enabled = false;
		Run(probe, bctx, Body::SetFlags, sf);
		BodyData::GetFlags gf2{};
		Run(probe, bctx, Body::GetFlags, gf2);
		Check(gf2.value == Physics::BodyFlags::Simulated, "SetFlags clears gravity");

		// world point velocity: w=(0,0,1), probe at x=-3 -> point (1,0,0) local is rotated by the pose; use identity pose
		sp.value = Physics::Pose(0, 0, 0, 0, 0, 0, 1);
		Run(probe, bctx, Body::SetPose, sp);
		slv.value = Physics::Vec3(0, 0, 0);
		Run(probe, bctx, Body::SetLinearVelocity, slv);
		BodyData::GetLocalPointVelocity glp{};
		glp.localPoint = Physics::Vec3(1, 0, 0);
		Run(probe, bctx, Body::GetLocalPointVelocity, glp);
		Check(Near(glp.value.x, 0.0f, 1e-4f) && Near(glp.value.y, 1.0f, 1e-4f), "local point velocity = w x r");
		BodyData::GetWorldPointVelocity gwp{};
		gwp.worldPoint = Physics::Vec3(0, 2, 0);
		Run(probe, bctx, Body::GetWorldPointVelocity, gwp);
		Check(Near(gwp.value.x, -2.0f, 1e-4f), "world point velocity = w x r");

		// impulse: m=3.5 -> dv = 7/3.5 = 2 (no gravity, no damping on this step beyond 0.1)
		sav.value = Physics::Vec3(0, 0, 0);
		Run(probe, bctx, Body::SetAngularVelocity, sav);
		sd.linear = 0.0f;
		sd.angular = 0.0f;
		Run(probe, bctx, Body::SetDamping, sd);
		BodyData::AddForce af{};
		af.value = Physics::Vec3(0, 7, 0);
		af.mode = Physics::ForceMode::Impulse;
		af.awake = true;
		Run(probe, bctx, Body::AddForce, af);
		Simulate(probeWorld, dt);
		const Physics::Vec3 vAfter = GetLinearVelocity(probeWorld, probe);
		Check(Near(vAfter.y, 2.0f, 0.02f), "impulse changes velocity by J/m");
		std::printf("  impulse: vy=%.4f (expected 2.0)\n", vAfter.y);

		// kinematic target
		sf.value = Physics::BodyFlags::Kinematic;
		sf.enabled = true;
		Run(probe, bctx, Body::SetFlags, sf);
		BodyData::SetKinematicTarget skt{};
		skt.value = Physics::Pose(3, 4, 5, 0, 0, 0, 1);
		Run(probe, bctx, Body::SetKinematicTarget, skt);
		Simulate(probeWorld, dt);
		const Physics::Pose kp = GetPose(probeWorld, probe);
		Check(Near(kp.pos.x, 3.0f, 1e-3f) && Near(kp.pos.y, 4.0f, 1e-3f) && Near(kp.pos.z, 5.0f, 1e-3f), "kinematic target reached");
		BodyData::GetKinematicTarget gkt{};
		Run(probe, bctx, Body::GetKinematicTarget, gkt);
		Check(Near(gkt.value.pos.y, 4.0f, 1e-3f), "kinematic target readable");

		// shape side
		ShapeData::SetFlags shf{};
		shf.flags = Physics::ShapeFlags::Trigger | Physics::ShapeFlags::SceneQuery;
		Run(probeShape, Context(probeWorld, probe), Shape::SetFlags, shf);
		ShapeData::GetFlags ghf{};
		Run(probeShape, Context(probeWorld, probe), Shape::GetFlags, ghf);
		Check(ghf.flags == shf.flags, "shape flags round trip (trigger)");
		shf.flags = Physics::ShapeFlags::SceneQuery;
		Run(probeShape, Context(probeWorld, probe), Shape::SetFlags, shf);

		ShapeData::SetGeometry sgm{};
		sgm.value = BoxGeom(1.0f, 2.0f, 3.0f);
		Run(probeShape, Context(probeWorld, probe), Shape::SetGeometry, sgm);
		ShapeData::GetGeometry ggm{};
		Run(probeShape, Context(probeWorld, probe), Shape::GetGeometry, ggm);
		Check(ggm.value.type == Physics::GeometryType::Box && ggm.value.boxGeometry.halfExtents.z == 3.0f, "geometry round trip");
		ShapeData::GetBoundsAtPose gbp{};
		gbp.pose = Physics::Pose(10, 0, 0, 0, 0, 0, 1);
		Run(probeShape, Context(probeWorld, probe), Shape::GetBoundsAtPose, gbp);
		Check(Near(gbp.extents.y, 2.0f, 1e-3f) && Near(gbp.center.x, 10.0f, 1e-3f), "bounds at pose");

		ShapeData::SetPose ssp{};
		ssp.value = Physics::Pose(0, 1, 0, 0, 0, 0, 1);
		Run(probeShape, Context(probeWorld, probe), Shape::SetPose, ssp);
		ShapeData::GetLocalPose glp2{};
		Run(probeShape, Context(probeWorld, probe), Shape::GetLocalPose, glp2);
		ShapeData::GetPose gwp2{};
		Run(probeShape, Context(probeWorld, probe), Shape::GetPose, gwp2);
		Check(Near(glp2.value.pos.y, 1.0f, 1e-5f) && Near(gwp2.value.pos.y, 5.0f, 1e-3f), "shape local / world pose");

		// unsupported geometry is rejected without a crash
		ShapeData::CreateShape bad{};
		bad.geom.type = Physics::GeometryType::TriangleMesh;
		bad.geom.meshGeometry.mesh = nullptr;
		bad.geom.meshGeometry.scale = Physics::Vec3(1, 1, 1);
		Run(nullptr, Context(probeWorld, nullptr), Shape::CreateShape, bad);
		Check(bad.value == nullptr, "triangle mesh shape rejected");

		// detach / re-attach / destroy shape while attached
		BodyData::DetachShape ds{};
		ds.shape = probeShape;
		Run(probe, bctx, Body::DetachShape, ds);
		BodyData::GetShapeCount gsc2{};
		Run(probe, bctx, Body::GetShapeCount, gsc2);
		Check(gsc2.value == 0, "detach shape");
		AttachShape(probeWorld, probe, probeShape);
		Run(probe, bctx, Body::GetShapeCount, gsc2);
		Check(gsc2.value == 1, "re-attach shape");

		ShapeData::DestroyShape dsh;
		Run(probeShape, Context(probeWorld, probe), Shape::DestroyShape, dsh);
		Run(probe, bctx, Body::GetShapeCount, gsc2);
		Check(gsc2.value == 0, "destroyed shape leaves the body");

		WorldData::GetBasicStats bs{};
		Run(probeWorld, pctx, World::GetBasicStats, bs);
		Check(bs.value.bodies == 1, "basic stats bodies");

		// leave probeWorld alive with a body and a loose shape: ShutdownSdk must clean it up
		(void)CreateShape(probeWorld, SphereGeom(1.0f), defaultMaterial, nullptr, MakeFilter(1, 1));
	}

	// ---- ignore pair cleanup ----
	{
		ShapeData::CleanupIgnoredColliders cl{ {}, 5 };
		Run(nullptr, Context(), Shape::CleanupIgnoredColliders, cl);
		ShapeData::ShouldIgnoreCollision sic2{};
		sic2.ignoreId0 = 5;
		sic2.ignoreId1 = 6;
		Run(nullptr, Context(), Shape::ShouldIgnoreCollision, sic2);
		Check(!sic2.value, "ignore pair cleaned up");
	}

	// ---- robustness: invalid / foreign handles must not crash ----
	{
		Handle bogus[] = { nullptr, reinterpret_cast<Handle>(0x1234), reinterpret_cast<Handle>(0x10010), cm.outMaterial };
		for (Handle h : bogus)
		{
			const Context bad(h, h);
			BodyData::SetMass sm;
			sm.value = 2.0f;
			BodyData::GetPose gp{};
			ShapeData::SetFilterData sf{};
			ShapeData::GetFlags gfl{};
			ShapeData::DestroyShape dsh;
			WorldData::Simulate sim;
			sim.deltaTime = dt;
			WorldData::SetGravity sg2;
			sg2.value = Physics::Vec3(0, -1, 0);
			QueryData::RayCast rc{};
			rc.maxDistance = 1.0f;
			rc.direction = Physics::Vec3(0, 1, 0);
			WorldData::DestroyBody db{};
			db.value = h;
			BodyData::AttachShape as{};
			as.shape = h;
			Run(h, bad, Body::SetMass, sm);
			Run(h, bad, Body::GetPose, gp);
			Run(h, bad, Body::AttachShape, as);
			Run(h, bad, Shape::SetFilterData, sf);
			Run(h, bad, Shape::GetFlags, gfl);
			Run(h, bad, World::SetGravity, sg2);
			Run(h, bad, Query::RayCast, rc);
			Run(h, bad, World::DestroyBody, db);
			Run(h, bad, Shape::DestroyShape, dsh);
			Run(h, bad, World::Simulate, sim);
		}
		Check(true, "invalid handles survived");
	}

	// ---- (e) teardown ----
	{
		for (Faller& f : fallers)
		{
			ShapeData::DestroyShape dsh;
			Run(f.shape, Context(world, f.body), Shape::DestroyShape, dsh);
			WorldData::DestroyBody db{};
			db.value = f.body;
			Run(world, Context(world, nullptr), World::DestroyBody, db);
		}
		WorldData::DestroyBody dbf{};
		dbf.value = floorBody;
		Run(world, Context(world, nullptr), World::DestroyBody, dbf);
		BodyData::GetPose after{};
		Run(floorBody, Context(world, floorBody), Body::GetPose, after);
		Check(true, "destroyed body queried without crash");

		ShapeData::DestroyShape dsh;
		Run(floorShape, Context(world, nullptr), Shape::DestroyShape, dsh);
		Run(world, Context(world, nullptr), World::DestroyWorld, dsh);
		Run(world, Context(world, nullptr), World::DestroyWorld, dsh); // double destroy

		// a world destroyed with live bodies and shapes
		Handle w3 = CreateWorld(-9.81f, 1);
		Handle b3 = CreateBody(w3, false, Physics::Pose(), 1.0f, dynamicFlags);
		Handle s3 = CreateShape(w3, SphereGeom(0.5f), defaultMaterial, nullptr, MakeFilter(1, 1));
		AttachShape(w3, b3, s3);
		AddToScene(w3, b3);
		Simulate(w3, dt);
		Run(w3, Context(w3, nullptr), World::DestroyWorld, dsh);
		ShapeData::GetFlags gfl{};
		Run(s3, Context(), Shape::GetFlags, gfl); // shape survives its world as a loose shape
		Run(s3, Context(), Shape::DestroyShape, dsh);

		MaterialData::ReleaseMaterial rm;
		Run(cm.outMaterial, Context(), SDK::ReleaseMaterial, rm);
	}

	SdkData::ShutdownSdk shutdown;
	Run(nullptr, Context(), SDK::ShutdownSdk, shutdown);
	unload();
	dlclose(lib);

	Check(g_errorLogs == 0, "no backend error logs");
	std::printf("fall_test: %d checks, %d failures, %d error logs -> %s\n", g_checks, g_failures, g_errorLogs,
		g_failures == 0 ? "PASS" : "FAIL");
	return g_failures == 0 ? 0 : 1;
}
