// Drives the backend through the Unity command entry point and checks the simulation events:
// contact begin / persist / end, trigger enter / exit, removal while touching, result ownership and teardown.
// Usage: event_test [path/to/libBackend.so]

#include <dlfcn.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <thread>
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

#ifndef FALL_TEST_BACKEND_PATH
#define FALL_TEST_BACKEND_PATH "libPhysX5Backend.so"
#endif
#ifndef FALL_TEST_SDK
#define FALL_TEST_SDK 0
#endif

using namespace PhysicsCommands;
using Handle = Physics::SDKObjectHandle;
using Physics::ShapePairEventFlags;

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

	constexpr float kDt = 1.0f / 60.0f;

	// ---- objects ----

	struct Dummy
	{
		uint64_t id;
	};

	std::deque<Dummy> g_dummies;

	Dummy* NewDummy(uint64_t id)
	{
		g_dummies.push_back(Dummy{ id });
		return &g_dummies.back();
	}

	Handle g_material = nullptr;

	Handle CreateWorld(float gravityY)
	{
		WorldData::CreateWorld cw;
		cw.jobWorkerCount = 2;
		cw.gravity = Physics::Vec3(0.0f, gravityY, 0.0f);
		Run(nullptr, Context(), World::CreateWorld, cw);
		return cw.outWorld;
	}

	void DestroyWorld(Handle world)
	{
		WorldData::DestroyWorld dw{};
		Run(world, Context(world, nullptr), World::DestroyWorld, dw);
	}

	void Simulate(Handle world)
	{
		WorldData::Simulate sim;
		sim.deltaTime = kDt;
		Run(world, Context(world, nullptr), World::Simulate, sim);
	}

	Physics::FilterData MakeFilter(ShapePairEventFlags pairFlags = ShapePairEventFlags::None)
	{
		Physics::FilterData f;
		f.belongsTo = 1;
		f.collidesWith = 0xFFFFFFFFu;
		f.pairFlags = pairFlags;
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

	struct Obj
	{
		Handle world = nullptr;
		Handle body = nullptr;
		Handle shape = nullptr;
		uint64_t bodyId = 0;
		uint64_t shapeId = 0;
	};

	Obj AddObject(Handle world, bool isStatic, const Physics::Pose& pose, const Physics::ShapeGeometry& geom,
		uint64_t bodyId, uint64_t shapeId, const Physics::FilterData& filter, Physics::ShapeFlags shapeFlags = Physics::ShapeFlags::SceneQuery)
	{
		Obj o;
		o.world = world;
		o.bodyId = bodyId;
		o.shapeId = shapeId;

		WorldData::CreateBody cb{};
		cb.pose = pose;
		cb.userData = NewDummy(bodyId);
		cb.mass = 1.0f;
		cb.angularDamping = 0.05f;
		cb.flags = isStatic ? Physics::BodyFlags::Simulated : (Physics::BodyFlags::Gravity | Physics::BodyFlags::Simulated);
		cb.isStatic = isStatic;
		Run(world, Context(world, nullptr), World::CreateBody, cb);
		o.body = cb.outBody;

		ShapeData::CreateShape cs{};
		cs.geom = geom;
		cs.materialPtr = g_material;
		cs.userData = NewDummy(shapeId);
		cs.flags = Physics::ShapeFlags::SceneQuery;
		Run(nullptr, Context(world, nullptr), Shape::CreateShape, cs);
		o.shape = cs.value;

		ShapeData::SetFilterData sf{};
		sf.data = filter;
		Run(o.shape, Context(world, nullptr), Shape::SetFilterData, sf);
		if (shapeFlags != Physics::ShapeFlags::SceneQuery)
		{
			ShapeData::SetFlags fl{};
			fl.flags = shapeFlags;
			Run(o.shape, Context(world, nullptr), Shape::SetFlags, fl);
		}

		BodyData::AttachShape as{};
		as.shape = o.shape;
		as.pose = Physics::Pose();
		as.forceLocalPose = true;
		Run(o.body, Context(world, o.body), Body::AttachShape, as);

		if (!isStatic)
		{
			BodyData::RecomputeMassProperties rp;
			Run(o.body, Context(world, o.body), Body::RecomputeMassProperties, rp);
		}

		Handle list[1] = { o.body };
		WorldData::AddActorsToScene aa{};
		aa.actors = list;
		aa.bufferSize = 1;
		Run(world, Context(world, nullptr), World::AddActorsToScene, aa);
		return o;
	}

	void SetVelocity(const Obj& o, const Physics::Vec3& v)
	{
		BodyData::SetLinearVelocity sv{};
		sv.value = v;
		sv.awake = true;
		Run(o.body, Context(o.world, o.body), Body::SetLinearVelocity, sv);
	}

	void SetPose(const Obj& o, const Physics::Pose& pose)
	{
		BodyData::SetPose sp{};
		sp.value = pose;
		sp.awake = true;
		Run(o.body, Context(o.world, o.body), Body::SetPose, sp);
	}

	float GetY(const Obj& o)
	{
		BodyData::GetPose gp{};
		Run(o.body, Context(o.world, o.body), Body::GetPose, gp);
		return gp.value.pos.y;
	}

	void DestroyBody(const Obj& o)
	{
		WorldData::DestroyBody db{};
		db.value = o.body;
		Run(o.world, Context(o.world, nullptr), World::DestroyBody, db);
	}

	void DetachShape(const Obj& o)
	{
		BodyData::DetachShape ds{};
		ds.shape = o.shape;
		Run(o.body, Context(o.world, o.body), Body::DetachShape, ds);
	}

	void DestroyShape(const Obj& o)
	{
		ShapeData::DestroyShape ds{};
		Run(o.shape, Context(o.world, nullptr), Shape::DestroyShape, ds);
	}

	// ---- results ----

	struct ShapeRec
	{
		uint64_t thisId = 0, otherId = 0;
		uint16_t flags = 0, events = 0;
		Physics::Vec3 impulseSum = Physics::Vec3(0.0f);
		std::vector<Physics::ContactPoint> points;
	};

	struct PairRec
	{
		uint64_t thisBody = 0, otherBody = 0;
		uint16_t flags = 0;
		Physics::Vec3 linThis = Physics::Vec3(0.0f), linOther = Physics::Vec3(0.0f);
		std::vector<ShapeRec> shapes;
	};

	struct TrigRec
	{
		Physics::TriggerEventType type;
		Handle triggerShape, colliderShape;
		uint64_t trigger, collider;
	};

	struct Step
	{
		std::vector<PairRec> pairs;
		std::vector<TrigRec> trigs;
		size_t jointBreaks = 0;
	};

	constexpr uint16_t kFirstTouch = static_cast<uint16_t>(Physics::ShapePairFlags::ActorPairHasFirstTouch);
	constexpr uint16_t kLostTouch = static_cast<uint16_t>(Physics::ShapePairFlags::ActorPairLostTouch);

	Step Fetch(Handle world, bool release = true)
	{
		Step step;
		WorldData::GetLastSimulationStepResults r{};
		Run(world, Context(world, nullptr), World::GetLastSimulationStepResults, r);
		for (size_t i = 0; i < r.bodyContactPairsCount; ++i)
		{
			const Physics::BodyPair& bp = r.bodyContactPairs[i];
			PairRec pr;
			pr.thisBody = bp.thisBodyID.data;
			pr.otherBody = bp.otherBodyID.data;
			pr.flags = static_cast<uint16_t>(bp.flags);
			pr.linThis = bp.thisBodyLinearVelocity;
			pr.linOther = bp.otherBodyLinearVelocity;
			for (uint32_t s = 0; s < bp.nbPairs; ++s)
			{
				const Physics::ShapePair& sp = bp.startPtr[s];
				ShapeRec sr;
				sr.thisId = sp.thisColliderID.data;
				sr.otherId = sp.otherColliderID.data;
				sr.flags = static_cast<uint16_t>(sp.flags);
				sr.events = static_cast<uint16_t>(sp.events);
				sr.impulseSum = sp.impulseSum;
				for (uint32_t p = 0; p < sp.nbPoints; ++p)
				{
					sr.points.push_back(sp.startPtr[p]);
				}
				pr.shapes.push_back(sr);
			}
			step.pairs.push_back(pr);
		}
		for (size_t i = 0; i < r.triggerEventsCount; ++i)
		{
			const Physics::TriggerEvent& te = r.triggerEvents[i];
			step.trigs.push_back(TrigRec{ te.type, te.triggerShape, te.colliderShape, te.trigger.data, te.collider.data });
		}
		step.jointBreaks = r.jointBreakEventsCount;
		if (release)
		{
			WorldData::ReleaseLastSimulationStepData rel{};
			Run(world, Context(world, nullptr), World::ReleaseLastSimulationStepData, rel);
		}
		return step;
	}

	Step Advance(Handle world)
	{
		Simulate(world);
		return Fetch(world);
	}

	Physics::SimulationStepStats GetStats(Handle world)
	{
		WorldData::GetLastSimulationStepStats st{};
		Run(world, Context(world, nullptr), World::GetLastSimulationStats, st);
		return st.value;
	}

	struct Counts
	{
		int first = 0, lost = 0, persist = 0, shapePairs = 0, removedActor = 0, removedShape = 0;
	};

	void Accumulate(Counts& c, const Step& step)
	{
		for (const PairRec& p : step.pairs)
		{
			if (p.flags & (static_cast<uint16_t>(Physics::BodyPairFlags::RemovedActor) | static_cast<uint16_t>(Physics::BodyPairFlags::RemovedOtherActor)))
			{
				++c.removedActor;
			}
			for (const ShapeRec& s : p.shapes)
			{
				++c.shapePairs;
				if (s.flags & kFirstTouch)
				{
					++c.first;
				}
				if (s.flags & kLostTouch)
				{
					++c.lost;
				}
				if (!(s.flags & (kFirstTouch | kLostTouch)))
				{
					++c.persist;
				}
				if (s.flags & (static_cast<uint16_t>(Physics::ShapePairFlags::RemovedShape) | static_cast<uint16_t>(Physics::ShapePairFlags::RemovedOtherShape)))
				{
					++c.removedShape;
				}
			}
		}
	}

	bool IsPair(uint64_t a, uint64_t b, uint64_t x, uint64_t y)
	{
		return (a == x && b == y) || (a == y && b == x);
	}

	float Dot(const Physics::Vec3& a, const Physics::Vec3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	float Length(const Physics::Vec3& v)
	{
		return std::sqrt(Dot(v, v));
	}

	constexpr uint64_t kFloorBody = 100, kFloorShape = 1000;
	constexpr uint64_t kBallBody = 200, kBallShape = 2000;

	ShapePairEventFlags AllContactEvents()
	{
		return ShapePairEventFlags::ContactDefault | ShapePairEventFlags::NotifyTouchFound | ShapePairEventFlags::NotifyTouchPersists
			| ShapePairEventFlags::NotifyTouchLost | ShapePairEventFlags::NotifyContactPoints;
	}

	// Floor (top at y = 0) and a sphere r = 0.5 that is dropped from 1 m above the floor.
	struct DropScene
	{
		Handle world = nullptr;
		Obj floor;
		Obj ball;
	};

	DropScene MakeDrop(ShapePairEventFlags ballFlags, ShapePairEventFlags floorFlags = ShapePairEventFlags::None, uint64_t idBase = 0)
	{
		DropScene s;
		s.world = CreateWorld(-9.81f);
		s.floor = AddObject(s.world, true, Physics::Pose(0, -0.5f, 0, 0, 0, 0, 1), BoxGeom(20.0f, 0.5f, 20.0f),
			kFloorBody + idBase, kFloorShape + idBase, MakeFilter(floorFlags));
		s.ball = AddObject(s.world, false, Physics::Pose(0, 1.5f, 0, 0, 0, 0, 1), SphereGeom(0.5f),
			kBallBody + idBase, kBallShape + idBase, MakeFilter(ballFlags));
		return s;
	}

	// Steps until the first touch is reported (or the limit is hit) and returns that step.
	bool StepToFirstTouch(const DropScene& s, Step& out, Counts& counts, int limit = 120)
	{
		for (int i = 0; i < limit; ++i)
		{
			out = Advance(s.world);
			Counts c;
			Accumulate(c, out);
			if (c.first > 0)
			{
				Accumulate(counts, out);
				return true;
			}
		}
		return false;
	}

	// ---- scenarios ----

	void TestContactLifecycle()
	{
		std::printf("(a) contact begin / persist / end\n");
		DropScene s = MakeDrop(AllContactEvents());

		Step found;
		Counts total;
		const bool touched = StepToFirstTouch(s, found, total);
		Check(touched, "(a) first touch reported");
		if (!touched)
		{
			DestroyWorld(s.world);
			return;
		}

		Check(found.pairs.size() == 1 && found.pairs[0].shapes.size() == 1, "(a) one body pair with one shape pair");
		const PairRec& pr = found.pairs[0];
		const ShapeRec& sr = pr.shapes[0];
		Check(IsPair(pr.thisBody, pr.otherBody, kFloorBody, kBallBody), "(a) body ids are floor and ball");
		Check(IsPair(sr.thisId, sr.otherId, kFloorShape, kBallShape), "(a) shape ids are floor and ball");
		Check((sr.events & static_cast<uint16_t>(ShapePairEventFlags::NotifyTouchFound)) != 0, "(a) event flags carry NotifyTouchFound");
		Check(sr.points.size() >= 1, "(a) contact points present");
		const bool ballIsThis = pr.thisBody == kBallBody;
		Check(ballIsThis == (sr.thisId == kBallShape), "(a) body and shape pair order agree");

		bool pointsOk = !sr.points.empty();
		bool impulseOk = !sr.points.empty();
		for (const Physics::ContactPoint& p : sr.points)
		{
			pointsOk = pointsOk && Near(p.position.y, 0.0f, 0.06f) && Near(std::fabs(p.normal.y), 1.0f, 0.01f);
			// the normal points at the first shape of the pair, so a ball in the first slot gets an upward normal
			pointsOk = pointsOk && ((p.normal.y > 0.0f) == ballIsThis);
			pointsOk = pointsOk && p.separation < 0.03f && p.separation > -0.2f;
			impulseOk = impulseOk && Dot(p.impulse, p.normal) >= 0.0f;
		}
		Check(pointsOk, "(a) contact point position / normal / separation plausible");
		Check(impulseOk, "(a) point impulses push along the normal");
		// ball of 1 kg falling about 1 m (v ~ 4.4 m/s): the stopping impulse is about m * v
		Check(std::fabs(sr.impulseSum.y) > 1.0f && std::fabs(sr.impulseSum.y) < 15.0f, "(a) impact impulse sum plausible");
		const Physics::Vec3 ballVel = ballIsThis ? pr.linThis : pr.linOther;
		const Physics::Vec3 floorVel = ballIsThis ? pr.linOther : pr.linThis;
		Check(ballVel.y < -3.0f && ballVel.y > -6.0f, "(a) pre-solver ball velocity reported");
		Check(Length(floorVel) < 1e-4f, "(a) static floor velocity is zero");

		// persisting contact while the ball rests: expect events and a weight-sized impulse
		Counts after;
		float restImpulse = 0.0f;
		for (int i = 0; i < 10; ++i)
		{
			Step st = Advance(s.world);
			Accumulate(after, st);
			if (i == 8 && !st.pairs.empty() && !st.pairs[0].shapes.empty())
			{
				restImpulse = std::fabs(st.pairs[0].shapes[0].impulseSum.y);
			}
		}
		Check(after.first == 0, "(a) no second first-touch while resting");
		Check(after.persist >= 8, "(a) persist events delivered every step while resting");
		Check(after.lost == 0, "(a) no lost touch while resting");
		Check(Near(restImpulse, 9.81f * kDt, 0.08f), "(a) resting impulse about m*g*dt");

		// lift the ball: exactly one lost event
		SetVelocity(s.ball, Physics::Vec3(0.0f, 5.0f, 0.0f));
		Counts lift;
		for (int i = 0; i < 8; ++i)
		{
			Step st = Advance(s.world);
			Accumulate(lift, st);
			for (const PairRec& p : st.pairs)
			{
				Check(IsPair(p.thisBody, p.otherBody, kFloorBody, kBallBody), "(a) lost event ids intact");
				Check((p.flags & 3) == 0, "(a) lifted ball is not reported as removed");
			}
		}
		Check(lift.lost == 1, "(a) exactly one lost touch when the ball leaves");
		Check(lift.first == 0 && lift.removedShape == 0, "(a) no extra first-touch or removed flags on lift");
		Counts quiet;
		for (int i = 0; i < 20; ++i)
		{
			Accumulate(quiet, Advance(s.world));
		}
		Check(quiet.shapePairs == 0, "(a) no events while the ball is in the air");
		Check(GetY(s.ball) > 1.0f, "(a) ball is above the floor");

		DestroyWorld(s.world);
	}

	void TestStats()
	{
		std::printf("(g) simulation stats\n");
		DropScene s = MakeDrop(AllContactEvents());
		bool statsOk = false;
		bool newTouchOk = false;
		for (int i = 0; i < 120 && !newTouchOk; ++i)
		{
			Simulate(s.world);
			const Physics::SimulationStepStats st = GetStats(s.world);
			Fetch(s.world);
			if (st.dynamicBodies == 1 && st.staticBodies == 1)
			{
				statsOk = true;
			}
			newTouchOk = st.newTouches >= 1 && st.contactPairs >= 1;
		}
		Check(statsOk, "(g) body counts in stats");
		Check(newTouchOk, "(g) newTouches / contactPairs reported on the touch step");
		DestroyWorld(s.world);
	}

	void TestOnlyRequestedEvents()
	{
		std::printf("(a2) only requested events\n");
		const ShapePairEventFlags foundLost = ShapePairEventFlags::ContactDefault | ShapePairEventFlags::NotifyTouchFound | ShapePairEventFlags::NotifyTouchLost;
		DropScene s = MakeDrop(foundLost);
		Counts c;
		for (int i = 0; i < 100; ++i)
		{
			Accumulate(c, Advance(s.world));
		}
		Check(c.first == 1, "(a2) first touch once");
		Check(c.persist == 0, "(a2) no persist events when not requested");
		DestroyWorld(s.world);

		// the flags of the static side alone also enable the events
		DropScene t = MakeDrop(ShapePairEventFlags::None, foundLost, 10);
		Counts d;
		for (int i = 0; i < 100; ++i)
		{
			Accumulate(d, Advance(t.world));
		}
		Check(d.first == 1, "(a2) flags set on the other shape enable the events");
		DestroyWorld(t.world);

		// contact points are only delivered when requested
		DropScene u = MakeDrop(foundLost, ShapePairEventFlags::None, 20);
		Step found;
		Counts tmp;
		const bool touched = StepToFirstTouch(u, found, tmp);
		Check(touched && found.pairs.size() == 1 && found.pairs[0].shapes[0].points.empty(), "(a2) no contact points without NotifyContactPoints");
		DestroyWorld(u.world);
	}

	void TestDisabledEvents()
	{
		std::printf("(b) events disabled\n");
		DropScene s = MakeDrop(ShapePairEventFlags::None);
		Counts c;
		for (int i = 0; i < 150; ++i)
		{
			const Step st = Advance(s.world);
			c.shapePairs += static_cast<int>(st.pairs.size());
			Check(st.trigs.empty() && st.jointBreaks == 0, "(b) no trigger / joint events");
		}
		Check(c.shapePairs == 0, "(b) no contact events without pair flags");
		Check(Near(GetY(s.ball), 0.5f, 0.05f), "(b) the ball still collides and rests on the floor");
		DestroyWorld(s.world);
	}

	void TestSupportedMessages()
	{
		std::printf("(h) SetSupportedMessages\n");
		const ShapePairEventFlags foundLost = ShapePairEventFlags::NotifyTouchFound | ShapePairEventFlags::NotifyTouchLost;

		// enabled through SetSupportedMessages only
		{
			DropScene s = MakeDrop(ShapePairEventFlags::None);
			ShapeData::SetSupportedMessages sm{};
			sm.flags = foundLost;
			sm.layer = 3;
			Run(s.ball.shape, Context(s.world, s.ball.body), Shape::SetSupportedMessages, sm);
			ShapeData::SetModifiableContacts mc{};
			mc.modifiable = true;
			Run(s.ball.shape, Context(s.world, s.ball.body), Shape::SetModifiableContacts, mc);
			Counts c;
			for (int i = 0; i < 100; ++i)
			{
				Accumulate(c, Advance(s.world));
			}
			Check(c.first == 1, "(h) SetSupportedMessages enables touch found");
			DestroyWorld(s.world);
		}
		// per layer entries are replaced independently, turning the last one off disables the events again
		{
			DropScene s = MakeDrop(ShapePairEventFlags::None, ShapePairEventFlags::None, 10);
			ShapeData::SetSupportedMessages sm{};
			sm.flags = foundLost;
			sm.layer = 3;
			Run(s.ball.shape, Context(s.world, s.ball.body), Shape::SetSupportedMessages, sm);
			sm.flags = ShapePairEventFlags::None;
			Run(s.ball.shape, Context(s.world, s.ball.body), Shape::SetSupportedMessages, sm);
			Counts c;
			for (int i = 0; i < 100; ++i)
			{
				Accumulate(c, Advance(s.world));
			}
			Check(c.shapePairs == 0, "(h) clearing the layer entry disables the events");
			DestroyWorld(s.world);
		}
		// switched on after the pair already exists: the lost touch is still reported
		{
			DropScene s = MakeDrop(ShapePairEventFlags::None, ShapePairEventFlags::None, 20);
			for (int i = 0; i < 60; ++i)
			{
				Advance(s.world);
			}
			ShapeData::SetSupportedMessages sm{};
			sm.flags = foundLost;
			sm.layer = 0;
			Run(s.ball.shape, Context(s.world, s.ball.body), Shape::SetSupportedMessages, sm);
			Advance(s.world);
			SetVelocity(s.ball, Physics::Vec3(0.0f, 5.0f, 0.0f));
			Counts c;
			for (int i = 0; i < 10; ++i)
			{
				Accumulate(c, Advance(s.world));
			}
			Check(c.lost == 1, "(h) lost touch reported for a pair enabled after it started");
			DestroyWorld(s.world);
		}
		// changing the flags while touching refilters the live pair in place: no lost / found pair, the new flag takes effect
		{
			DropScene s = MakeDrop(foundLost, ShapePairEventFlags::None, 30);
			Step found;
			Counts total;
			Check(StepToFirstTouch(s, found, total), "(h) touch before the flag change");
			for (int i = 0; i < 3; ++i)
			{
				Advance(s.world);
			}
			ShapeData::SetSupportedMessages sm{};
			sm.flags = ShapePairEventFlags::NotifyContactPoints;
			sm.layer = 0;
			Run(s.ball.shape, Context(s.world, s.ball.body), Shape::SetSupportedMessages, sm);
			Counts c;
			for (int i = 0; i < 10; ++i)
			{
				Accumulate(c, Advance(s.world));
			}
			Check(c.first == 0 && c.lost == 0, "(h) a flag change on a touching pair reports no lost / found touch");
			// the refiltered pair still reports its end
			SetVelocity(s.ball, Physics::Vec3(0.0f, 5.0f, 0.0f));
			Counts after;
			for (int i = 0; i < 10; ++i)
			{
				Accumulate(after, Advance(s.world));
			}
			Check(after.lost == 1 && after.first == 0, "(h) the pair refiltered in place reports its lost touch once");
			DestroyWorld(s.world);
		}
		// persist events switched on while touching start right away (the pair is registered again for them)
		{
			DropScene s = MakeDrop(foundLost, ShapePairEventFlags::None, 40);
			Step found;
			Counts total;
			Check(StepToFirstTouch(s, found, total), "(h) touch before enabling persist");
			ShapeData::SetSupportedMessages sm{};
			sm.flags = ShapePairEventFlags::NotifyTouchPersists;
			sm.layer = 0;
			Run(s.ball.shape, Context(s.world, s.ball.body), Shape::SetSupportedMessages, sm);
			Counts c;
			for (int i = 0; i < 10; ++i)
			{
				Accumulate(c, Advance(s.world));
			}
			Check(c.persist >= 6, "(h) persist enabled while touching reports persisting contacts");
			DestroyWorld(s.world);
		}
	}

	void TestContactRemoval()
	{
		std::printf("(d0) removal while touching\n");
		// 0: destroy body only, 1: destroy body and shape, 2: detach + destroy the shape, 3: destroy the floor body
		for (int mode = 0; mode < 4; ++mode)
		{
			DropScene s = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, static_cast<uint64_t>(mode) * 10);
			const uint64_t floorBody = kFloorBody + mode * 10, floorShape = kFloorShape + mode * 10;
			const uint64_t ballBody = kBallBody + mode * 10, ballShape = kBallShape + mode * 10;
			Step found;
			Counts total;
			if (!Check(StepToFirstTouch(s, found, total), "(d0) touch before removal"))
			{
				DestroyWorld(s.world);
				continue;
			}
			for (int i = 0; i < 10; ++i)
			{
				Advance(s.world);
			}

			switch (mode)
			{
			case 0: DestroyBody(s.ball); break;
			case 1: DestroyBody(s.ball); DestroyShape(s.ball); break;
			case 2: DetachShape(s.ball); DestroyShape(s.ball); break;
			default: DestroyBody(s.floor); DestroyShape(s.floor); break;
			}

			Counts c;
			Step lostStep;
			for (int i = 0; i < 5; ++i)
			{
				Step st = Advance(s.world);
				Counts one;
				Accumulate(one, st);
				if (one.lost > 0)
				{
					lostStep = st;
				}
				Accumulate(c, st);
			}
			char label[96];
			std::snprintf(label, sizeof(label), "(d0) mode %d: exactly one lost touch", mode);
			Check(c.lost == 1, label);
			std::snprintf(label, sizeof(label), "(d0) mode %d: removed objects flagged", mode);
			Check(c.removedActor > 0 || c.removedShape > 0, label);
			if (lostStep.pairs.size() == 1 && lostStep.pairs[0].shapes.size() == 1)
			{
				const PairRec& p = lostStep.pairs[0];
				std::snprintf(label, sizeof(label), "(d0) mode %d: body ids survive the removal", mode);
				Check(IsPair(p.thisBody, p.otherBody, floorBody, ballBody) || (mode == 2 && (p.thisBody == floorBody || p.otherBody == floorBody)), label);
				std::snprintf(label, sizeof(label), "(d0) mode %d: shape ids survive the removal", mode);
				Check(IsPair(p.shapes[0].thisId, p.shapes[0].otherId, floorShape, ballShape), label);
			}
			else
			{
				Check(false, "(d0) lost step has one pair");
			}
			Counts later;
			for (int i = 0; i < 20; ++i)
			{
				Accumulate(later, Advance(s.world));
			}
			Check(later.shapePairs == 0, "(d0) the removed pair does not report again");
			DestroyWorld(s.world);
		}
	}

	constexpr uint64_t kTrigBody = 300, kTrigShape = 3000;

	struct TriggerScene
	{
		Handle world = nullptr;
		Obj trigger;
	};

	// Static trigger box covering y in [4, 6]; no gravity.
	TriggerScene MakeTrigger()
	{
		TriggerScene t;
		t.world = CreateWorld(0.0f);
		t.trigger = AddObject(t.world, true, Physics::Pose(0, 5.0f, 0, 0, 0, 0, 1), BoxGeom(2.0f, 1.0f, 2.0f),
			kTrigBody, kTrigShape, MakeFilter(), Physics::ShapeFlags::Trigger);
		return t;
	}

	void TestTriggers()
	{
		std::printf("(c) trigger enter / exit\n");
		// pass through
		{
			TriggerScene t = MakeTrigger();
			Obj ball = AddObject(t.world, false, Physics::Pose(0, 8.0f, 0, 0, 0, 0, 1), SphereGeom(0.5f),
				kBallBody, kBallShape, MakeFilter());
			SetVelocity(ball, Physics::Vec3(0.0f, -6.0f, 0.0f));
			std::vector<TrigRec> events;
			bool pairsSeen = false;
			bool triggerPairStat = false;
			for (int i = 0; i < 120; ++i)
			{
				Simulate(t.world);
				if (GetStats(t.world).triggerPairs > 0)
				{
					triggerPairStat = true;
				}
				const Step st = Fetch(t.world);
				pairsSeen = pairsSeen || !st.pairs.empty();
				for (const TrigRec& e : st.trigs)
				{
					events.push_back(e);
				}
			}
			Check(events.size() == 2, "(c) one enter and one exit");
			if (events.size() == 2)
			{
				Check(events[0].type == Physics::TriggerEventType::Enter && events[1].type == Physics::TriggerEventType::Exit, "(c) enter before exit");
				for (const TrigRec& e : events)
				{
					Check(e.trigger == kTrigShape && e.collider == kBallShape, "(c) trigger / collider ids are the shape ids");
					Check(e.triggerShape == t.trigger.shape && e.colliderShape == ball.shape, "(c) trigger / collider shape handles");
				}
			}
			Check(!pairsSeen, "(c) trigger overlap produces no contact pair");
			Check(triggerPairStat, "(c) trigger pairs counted in the stats");
			DestroyWorld(t.world);
		}
		// resting inside: enter once, nothing afterwards (stay is derived by the engine)
		{
			TriggerScene t = MakeTrigger();
			Obj ball = AddObject(t.world, false, Physics::Pose(0, 5.0f, 0, 0, 0, 0, 1), SphereGeom(0.5f),
				kBallBody, kBallShape, MakeFilter());
			(void)ball;
			int enters = 0, exits = 0;
			for (int i = 0; i < 90; ++i)
			{
				for (const TrigRec& e : Advance(t.world).trigs)
				{
					(e.type == Physics::TriggerEventType::Enter ? enters : exits)++;
				}
			}
			Check(enters == 1 && exits == 0, "(c) resting inside gives a single enter and no exit");
			DestroyWorld(t.world);
		}
		// teleported out: exit
		{
			TriggerScene t = MakeTrigger();
			Obj ball = AddObject(t.world, false, Physics::Pose(0, 5.0f, 0, 0, 0, 0, 1), SphereGeom(0.5f),
				kBallBody, kBallShape, MakeFilter());
			int enters = 0, exits = 0;
			for (int i = 0; i < 5; ++i)
			{
				for (const TrigRec& e : Advance(t.world).trigs)
				{
					(e.type == Physics::TriggerEventType::Enter ? enters : exits)++;
				}
			}
			SetPose(ball, Physics::Pose(0, 20.0f, 0, 0, 0, 0, 1));
			for (int i = 0; i < 5; ++i)
			{
				for (const TrigRec& e : Advance(t.world).trigs)
				{
					(e.type == Physics::TriggerEventType::Enter ? enters : exits)++;
				}
			}
			Check(enters == 1 && exits == 1, "(c) teleport out gives one exit");
			DestroyWorld(t.world);
		}
	}

	void TestTriggerRemoval()
	{
		std::printf("(d) removal inside a trigger\n");
		const char* names[] = { "destroy body", "destroy body + shape", "detach + destroy shape", "destroy trigger shape", "destroy trigger body + shape", "destroy world" };
		for (int mode = 0; mode < 5; ++mode)
		{
			TriggerScene t = MakeTrigger();
			Obj ball = AddObject(t.world, false, Physics::Pose(0, 5.0f, 0, 0, 0, 0, 1), SphereGeom(0.5f),
				kBallBody, kBallShape, MakeFilter());
			int enters = 0;
			for (int i = 0; i < 4; ++i)
			{
				for (const TrigRec& e : Advance(t.world).trigs)
				{
					enters += e.type == Physics::TriggerEventType::Enter ? 1 : 0;
				}
			}
			char label[128];
			std::snprintf(label, sizeof(label), "(d) %s: entered before removal", names[mode]);
			Check(enters == 1, label);

			switch (mode)
			{
			case 0: DestroyBody(ball); break;
			case 1: DestroyBody(ball); DestroyShape(ball); break;
			case 2: DetachShape(ball); DestroyShape(ball); break;
			case 3: DetachShape(t.trigger); DestroyShape(t.trigger); break;
			default: DestroyBody(t.trigger); DestroyShape(t.trigger); break;
			}

			std::vector<TrigRec> events;
			for (int i = 0; i < 5; ++i)
			{
				for (const TrigRec& e : Advance(t.world).trigs)
				{
					events.push_back(e);
				}
			}
			std::snprintf(label, sizeof(label), "(d) %s: pair ends with exactly one exit", names[mode]);
			Check(events.size() == 1 && events[0].type == Physics::TriggerEventType::Exit, label);
			if (events.size() == 1)
			{
				std::snprintf(label, sizeof(label), "(d) %s: ids of the removed shape are still reported", names[mode]);
				Check(events[0].trigger == kTrigShape && events[0].collider == kBallShape, label);
			}
			size_t later = 0;
			for (int i = 0; i < 20; ++i)
			{
				later += Advance(t.world).trigs.size();
			}
			std::snprintf(label, sizeof(label), "(d) %s: no further trigger events", names[mode]);
			Check(later == 0, label);
			DestroyWorld(t.world);
		}
	}

	void TestResultsOwnership()
	{
		std::printf("(e) results ownership\n");
		DropScene s = MakeDrop(AllContactEvents());
		Step found;
		Counts total;
		Check(StepToFirstTouch(s, found, total), "(e) touch");

		// repeated get / release
		for (int i = 0; i < 50; ++i)
		{
			Simulate(s.world);
			const Step st = Fetch(s.world, false);
			if (i < 8)
			{
				Check(!st.pairs.empty(), "(e) persist step delivers a pair"); // the resting ball falls asleep later
			}
			if (i % 2 == 0)
			{
				WorldData::ReleaseLastSimulationStepData rel{};
				Run(s.world, Context(s.world, nullptr), World::ReleaseLastSimulationStepData, rel);
				Run(s.world, Context(s.world, nullptr), World::ReleaseLastSimulationStepData, rel); // double release
			}
			else
			{
				// get again without releasing: the earlier data is replaced, the new call returns nothing new
				const Step again = Fetch(s.world, false);
				Check(again.pairs.empty(), "(e) a second get without a step returns no events");
				WorldData::ReleaseLastSimulationStepData rel{};
				Run(s.world, Context(s.world, nullptr), World::ReleaseLastSimulationStepData, rel);
			}
		}
		// get without any release, then more steps
		for (int i = 0; i < 5; ++i)
		{
			Simulate(s.world);
			Fetch(s.world, false);
		}
		// release without get, bogus handles
		WorldData::ReleaseLastSimulationStepData rel{};
		Run(s.world, Context(s.world, nullptr), World::ReleaseLastSimulationStepData, rel);
		Run(nullptr, Context(), World::ReleaseLastSimulationStepData, rel);
		WorldData::GetLastSimulationStepResults r{};
		r.bodyContactPairsCount = 77;
		Run(nullptr, Context(), World::GetLastSimulationStepResults, r);
		Check(r.bodyContactPairsCount == 0 && r.triggerEventsCount == 0 && r.jointBreakEventsCount == 0, "(e) get on a null world is empty");
		Dummy bogus{ 0 };
		Run(&bogus, Context(), World::GetLastSimulationStepResults, r);
		Run(&bogus, Context(), World::ReleaseLastSimulationStepData, rel);
		WorldData::GetLastSimulationStepStats stats{};
		Run(&bogus, Context(), World::GetLastSimulationStats, stats);
		Check(stats.value.dynamicBodies == 0, "(e) stats of an unknown world are zero");

		// results of steps that were never fetched are dropped when the next step starts (scenes without listeners
		// must not accumulate events): only the last step is delivered, the touch found long before is gone
		DropScene u = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 10);
		for (int i = 0; i < 90; ++i)
		{
			Simulate(u.world);
		}
		const Step batch = Fetch(u.world);
		Counts c;
		Accumulate(c, batch);
		Check(c.first == 0 && c.persist <= 1, "(e) results of unfetched steps are dropped");
		// fetching every step still delivers everything
		DropScene f = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 10);
		Counts all;
		for (int i = 0; i < 90; ++i)
		{
			Simulate(f.world);
			Accumulate(all, Fetch(f.world));
		}
		Check(all.first == 1 && all.persist > 5, "(e) fetching every step delivers the touch and its persists");
		DestroyWorld(f.world);
		DestroyWorld(u.world);
		DestroyWorld(s.world);
	}

	struct WorldRun
	{
		Counts counts;
		bool idsOk = true;
	};

	void RunDrop(const DropScene& s, uint64_t idBase, WorldRun& out)
	{
		for (int i = 0; i < 120; ++i)
		{
			const Step st = Advance(s.world);
			Accumulate(out.counts, st);
			for (const PairRec& p : st.pairs)
			{
				out.idsOk = out.idsOk && IsPair(p.thisBody, p.otherBody, kFloorBody + idBase, kBallBody + idBase);
			}
		}
	}

	void TestMultipleWorlds()
	{
		std::printf("(e2) several worlds\n");
		DropScene a = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 0);
		DropScene b = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 1000);
		WorldRun ra, rb;
		// sequential interleaved
		for (int i = 0; i < 120; ++i)
		{
			const Step sa = Advance(a.world);
			const Step sb = Advance(b.world);
			Accumulate(ra.counts, sa);
			Accumulate(rb.counts, sb);
			for (const PairRec& p : sa.pairs)
			{
				ra.idsOk = ra.idsOk && IsPair(p.thisBody, p.otherBody, kFloorBody, kBallBody);
			}
			for (const PairRec& p : sb.pairs)
			{
				rb.idsOk = rb.idsOk && IsPair(p.thisBody, p.otherBody, kFloorBody + 1000, kBallBody + 1000);
			}
		}
		Check(ra.counts.first == 1 && rb.counts.first == 1 && ra.idsOk && rb.idsOk, "(e2) worlds report only their own events");
		DestroyWorld(a.world);
		DestroyWorld(b.world);

		// concurrent worlds on separate threads (each world is its own lock domain)
		DropScene c = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 2000);
		DropScene d = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 3000);
		WorldRun rc, rd;
		std::thread tc([&] { RunDrop(c, 2000, rc); });
		std::thread td([&] { RunDrop(d, 3000, rd); });
		tc.join();
		td.join();
		Check(rc.counts.first == 1 && rd.counts.first == 1 && rc.idsOk && rd.idsOk, "(e2) concurrent worlds report only their own events");
		DestroyWorld(c.world);
		DestroyWorld(d.world);
	}

	void TestDestroyWithEvents()
	{
		std::printf("(f) world destroyed with events pending\n");
		// pending, never fetched
		{
			DropScene s = MakeDrop(AllContactEvents());
			for (int i = 0; i < 90; ++i)
			{
				Simulate(s.world);
			}
			DestroyWorld(s.world);
			Step st = Fetch(s.world);
			Check(st.pairs.empty() && st.trigs.empty(), "(f) get after destroy is empty");
			WorldData::ReleaseLastSimulationStepData rel{};
			Run(s.world, Context(s.world, nullptr), World::ReleaseLastSimulationStepData, rel);
		}
		// fetched but not released
		{
			DropScene s = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 10);
			Step found;
			Counts c;
			StepToFirstTouch(s, found, c);
			Simulate(s.world);
			WorldData::GetLastSimulationStepResults r{};
			Run(s.world, Context(s.world, nullptr), World::GetLastSimulationStepResults, r);
			Check(r.bodyContactPairsCount > 0, "(f) results outstanding");
			DestroyWorld(s.world);
			WorldData::ReleaseLastSimulationStepData rel{};
			Run(s.world, Context(s.world, nullptr), World::ReleaseLastSimulationStepData, rel);
			Run(s.world, Context(s.world, nullptr), World::ReleaseLastSimulationStepData, rel);
		}
		// removal events pending when the world dies
		{
			DropScene s = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 20);
			Step found;
			Counts c;
			StepToFirstTouch(s, found, c);
			Advance(s.world);
			DestroyBody(s.ball);
			DestroyShape(s.ball);
			DestroyWorld(s.world);
			Fetch(s.world);
		}
		// a trigger world destroyed right after removal
		{
			TriggerScene t = MakeTrigger();
			Obj ball = AddObject(t.world, false, Physics::Pose(0, 5.0f, 0, 0, 0, 0, 1), SphereGeom(0.5f),
				kBallBody, kBallShape, MakeFilter());
			for (int i = 0; i < 3; ++i)
			{
				Simulate(t.world);
			}
			DestroyBody(ball);
			DestroyWorld(t.world);
			Fetch(t.world);
		}
		// left alive with pending and outstanding events for SDK shutdown
		{
			DropScene s = MakeDrop(AllContactEvents(), ShapePairEventFlags::None, 30);
			for (int i = 0; i < 60; ++i)
			{
				Simulate(s.world);
			}
			WorldData::GetLastSimulationStepResults r{};
			Run(s.world, Context(s.world, nullptr), World::GetLastSimulationStepResults, r);
			for (int i = 0; i < 10; ++i)
			{
				Simulate(s.world);
			}
			// intentionally not destroyed: ShutdownSdk owns the cleanup
		}
		Check(true, "(f) no crash");
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
	std::printf("event_test: backend=%s (requested SDK %d)\n", path.c_str(), FALL_TEST_SDK);

	SdkData::InitializeSdk init;
	init.engineObjectIdOffset = 0;
	Run(nullptr, Context(), SDK::InitializeSdk, init);
	Check(init.result, "InitializeSdk");

	MaterialData::GetDefaultMaterial dm{};
	Run(nullptr, Context(), SDK::GetDefaultMaterial, dm);
	g_material = dm.value;
	Check(g_material != nullptr, "GetDefaultMaterial");

	TestContactLifecycle();
	TestStats();
	TestOnlyRequestedEvents();
	TestDisabledEvents();
	TestSupportedMessages();
	TestContactRemoval();
	TestTriggers();
	TestTriggerRemoval();
	TestResultsOwnership();
	TestMultipleWorlds();
	TestDestroyWithEvents();

	SdkData::ShutdownSdk shutdown;
	Run(nullptr, Context(), SDK::ShutdownSdk, shutdown);
	unload();
	dlclose(lib);

	Check(g_errorLogs == 0, "no backend error logs");
	std::printf("event_test: %d checks, %d failures, %d error logs -> %s\n", g_checks, g_failures, g_errorLogs,
		g_failures == 0 ? "PASS" : "FAIL");
	return g_failures == 0 ? 0 : 1;
}
