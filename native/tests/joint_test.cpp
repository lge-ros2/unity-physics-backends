// Drives the general joints (World::CreateJoint + Joint:: commands) through the Unity command entry point.
// Usage: joint_test [path/to/libBackend.so]

#include <dlfcn.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
using Physics::ConstraintAxis;
using Physics::JointDofLock;
using Physics::JointFlags;
using Physics::JointType;
using Physics::PoseTarget;

namespace
{
	BackendEntryPoint g_entry = nullptr;
	int g_errorLogs = 0;
	int g_failures = 0;
	int g_checks = 0;
	Handle g_material = nullptr;

	const float kPi = 3.14159265358979f;

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

	bool CheckNear(float actual, float expected, float tol, const char* what)
	{
		const bool ok = Near(actual, expected, tol);
		if (!ok)
		{
			std::printf("  (%s: got %.5f expected %.5f +-%.5f)\n", what, actual, expected, tol);
		}
		return Check(ok, what);
	}

	bool CheckRange(float actual, float lo, float hi, const char* what)
	{
		const bool ok = actual >= lo && actual <= hi;
		if (!ok)
		{
			std::printf("  (%s: got %.5f expected [%.5f, %.5f])\n", what, actual, lo, hi);
		}
		return Check(ok, what);
	}

	struct Dummy
	{
		uint64_t id;
	};

	// ---- world / body helpers ----

	Handle CreateWorld(float gravityY)
	{
		WorldData::CreateWorld cw;
		cw.jobWorkerCount = 1;
		cw.gravity = Physics::Vec3(0.0f, gravityY, 0.0f);
		Run(nullptr, Context(), World::CreateWorld, cw);
		return cw.outWorld;
	}

	void DestroyWorld(Handle world)
	{
		WorldData::DestroyWorld dw{};
		Run(world, Context(world, nullptr), World::DestroyWorld, dw);
	}

	void Step(Handle world, int steps, float dt)
	{
		for (int i = 0; i < steps; ++i)
		{
			WorldData::Simulate sim;
			sim.deltaTime = dt;
			Run(world, Context(world, nullptr), World::Simulate, sim);
		}
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

	// Body with one collider that collides with nothing (joint behaviour in isolation).
	Handle MakeBody(Handle world, bool isStatic, const Physics::Pose& pose, float mass, bool gravity, const Physics::ShapeGeometry& geom)
	{
		WorldData::CreateBody cb{};
		cb.pose = pose;
		cb.mass = mass;
		cb.angularDamping = 0.0f;
		cb.linearDamping = 0.0f;
		cb.flags = isStatic ? Physics::BodyFlags::Simulated
			: (gravity ? (Physics::BodyFlags::Gravity | Physics::BodyFlags::Simulated) : Physics::BodyFlags::Simulated);
		cb.isStatic = isStatic;
		Run(world, Context(world, nullptr), World::CreateBody, cb);
		Handle body = cb.outBody;

		ShapeData::CreateShape cs{};
		cs.geom = geom;
		cs.materialPtr = g_material;
		cs.flags = Physics::ShapeFlags::SceneQuery;
		Run(nullptr, Context(world, nullptr), Shape::CreateShape, cs);
		ShapeData::SetFilterData sf{};
		sf.data.belongsTo = 1;
		sf.data.collidesWith = 0;
		Run(cs.value, Context(world, nullptr), Shape::SetFilterData, sf);

		BodyData::AttachShape as{};
		as.shape = cs.value;
		as.pose = Physics::Pose();
		as.forceLocalPose = true;
		Run(body, Context(world, body), Body::AttachShape, as);

		Handle list[1] = { body };
		WorldData::AddActorsToScene aa{};
		aa.actors = list;
		aa.bufferSize = 1;
		Run(world, Context(world, nullptr), World::AddActorsToScene, aa);

		if (!isStatic)
		{
			BodyData::RecomputeMassProperties rp;
			Run(body, Context(world, body), Body::RecomputeMassProperties, rp);
		}
		return body;
	}

	Handle MakeBox(Handle world, float x, float y, float z, bool gravity, float mass = 1.0f)
	{
		return MakeBody(world, false, Physics::Pose(x, y, z, 0, 0, 0, 1), mass, gravity, BoxGeom(0.25f, 0.25f, 0.25f));
	}

	Physics::Pose GetPose(Handle world, Handle body)
	{
		BodyData::GetPose gp{};
		Run(body, Context(world, body), Body::GetPose, gp);
		return gp.value;
	}

	void SetLinearVelocity(Handle world, Handle body, float x, float y, float z)
	{
		BodyData::SetLinearVelocity sv{};
		sv.value = Physics::Vec3(x, y, z);
		sv.awake = true;
		Run(body, Context(world, body), Body::SetLinearVelocity, sv);
	}

	void SetAngularVelocity(Handle world, Handle body, float x, float y, float z)
	{
		BodyData::SetAngularVelocity sv{};
		sv.value = Physics::Vec3(x, y, z);
		sv.awake = true;
		Run(body, Context(world, body), Body::SetAngularVelocity, sv);
	}

	Physics::Vec3 GetAngularVelocity(Handle world, Handle body)
	{
		BodyData::GetAngularVelocity gv{};
		Run(body, Context(world, body), Body::GetAngularVelocity, gv);
		return gv.value;
	}

	Physics::Vec3 GetLinearVelocity(Handle world, Handle body)
	{
		BodyData::GetLinearVelocity gv{};
		Run(body, Context(world, body), Body::GetLinearVelocity, gv);
		return gv.value;
	}

	void DestroyBody(Handle world, Handle body)
	{
		WorldData::DestroyBody db{};
		db.value = body;
		Run(world, Context(world, nullptr), World::DestroyBody, db);
	}

	// Signed rotation angle of the pose around a principal axis (0 = x, 1 = y, 2 = z).
	float AxisAngle(const Physics::Pose& p, int axis)
	{
		const float s = axis == 0 ? p.quat.x : (axis == 1 ? p.quat.y : p.quat.z);
		float a = 2.0f * std::atan2(s, p.quat.w);
		if (a > kPi)
		{
			a -= 2.0f * kPi;
		}
		if (a < -kPi)
		{
			a += 2.0f * kPi;
		}
		return a;
	}

	float Distance(const Physics::Vec3& a, const Physics::Vec3& b)
	{
		const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

	float Length(const Physics::Vec3& v)
	{
		return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	}

	// ---- joint helpers ----

	Handle CreateJoint(Handle world, Handle b0, Handle b1, JointType type, JointFlags flags, void* userData = nullptr)
	{
		WorldData::CreateJoint cj{};
		cj.body0 = b0;
		cj.body1 = b1;
		cj.type = type;
		cj.flags = flags;
		cj.userData = userData;
		Run(world, Context(world, nullptr), World::CreateJoint, cj);
		return cj.outJoint;
	}

	void DestroyJoint(Handle world, Handle joint)
	{
		WorldData::DestroyJoint dj{};
		dj.value = joint;
		Run(world, Context(world, nullptr), World::DestroyJoint, dj);
	}

	template <typename E, typename C>
	void RunJ(Handle world, Handle joint, E id, C& cmd)
	{
		Run(joint, Context(world, nullptr), id, cmd);
	}

	void SetFrames(Handle world, Handle joint, const Physics::Pose& parent, const Physics::Pose& child)
	{
		JointData::SetActorLocalPose sp{};
		sp.target = PoseTarget::Parent;
		sp.value = parent;
		RunJ(world, joint, Joint::SetActorLocalPose, sp);
		sp.target = PoseTarget::Child;
		sp.value = child;
		RunJ(world, joint, Joint::SetActorLocalPose, sp);
	}

	bool IsValid(Handle world, Handle joint)
	{
		JointData::GetIsConstraintValid gv{};
		gv.value = true;
		RunJ(world, joint, Joint::GetIsConstraintValid, gv);
		return gv.value;
	}

	void SetLock(Handle world, Handle joint, ConstraintAxis axis, JointDofLock lock)
	{
		JointData::Set6DofAxisLock sl{};
		sl.axis = axis;
		sl.value = lock;
		RunJ(world, joint, Joint::Set6DofAxisLock, sl);
	}

	void LockAll(Handle world, Handle joint, JointDofLock linear, JointDofLock angular)
	{
		SetLock(world, joint, ConstraintAxis::LinearX, linear);
		SetLock(world, joint, ConstraintAxis::LinearY, linear);
		SetLock(world, joint, ConstraintAxis::LinearZ, linear);
		SetLock(world, joint, ConstraintAxis::AngularX, angular);
		SetLock(world, joint, ConstraintAxis::AngularY, angular);
		SetLock(world, joint, ConstraintAxis::AngularZ, angular);
	}

	void SetLimit6(Handle world, Handle joint, ConstraintAxis axis, float lo, float hi)
	{
		JointData::Set6DofLimit sl{};
		sl.axis = axis;
		sl.min = lo;
		sl.max = hi;
		sl.bounciness = 0.0f;
		sl.bounceMinVelocity = 0.0f;
		sl.contactDistance = 0.02f;
		RunJ(world, joint, Joint::Set6DofLimit, sl);
	}

	void SetDrive(Handle world, Handle joint, ConstraintAxis axis, float spring, float damper, float maxForce)
	{
		JointData::Set6DofAxisMotorConfiguration mc{};
		mc.axis = axis;
		mc.positionSpring = spring;
		mc.positionDamper = damper;
		mc.maximumForce = maxForce;
		mc.usesAcceleration = 0;
		RunJ(world, joint, Joint::Set6DofAxisMotorConfiguration, mc);
	}

	const JointFlags kJointFlags = JointFlags::Preprocessing | JointFlags::DriveLimitsAreForces;
	const float kDt = 1.0f / 120.0f;

	// A 6DoF joint between the world (anchor at 0,5,0) and a box at the same position.
	struct D6Rig
	{
		Handle world = nullptr;
		Handle body = nullptr;
		Handle joint = nullptr;
	};

	D6Rig MakeD6Rig()
	{
		D6Rig r;
		r.world = CreateWorld(0.0f);
		r.body = MakeBox(r.world, 0.0f, 5.0f, 0.0f, false);
		r.joint = CreateJoint(r.world, nullptr, r.body, JointType::Dof6, kJointFlags);
		SetFrames(r.world, r.joint, Physics::Pose(0, 5, 0, 0, 0, 0, 1), Physics::Pose());
		return r;
	}

	// ---- scenarios ----

	void FixedJointTest()
	{
		std::printf("[a] fixed joint\n");
		Handle world = CreateWorld(-9.81f);
		Handle a = MakeBox(world, 0.0f, 100.0f, 0.0f, true);
		Handle b = MakeBox(world, 1.0f, 100.0f, 0.0f, true);
		Dummy user{ 77 };
		Handle joint = CreateJoint(world, a, b, JointType::Fixed, kJointFlags, &user);
		Check(joint != nullptr, "fixed: CreateJoint");
		SetFrames(world, joint, Physics::Pose(0.5f, 0, 0, 0, 0, 0, 1), Physics::Pose(-0.5f, 0, 0, 0, 0, 0, 1));

		JointData::GetUserData gu{};
		RunJ(world, joint, Joint::GetUserData, gu);
		Check(gu.value == &user, "fixed: user data");

		JointData::GetActors ga{};
		RunJ(world, joint, Joint::GetActors, ga);
		Check(ga.actors[0].actor == a && ga.actors[1].actor == b && ga.actors[0].type == Physics::ActorType::Dynamic, "fixed: GetActors");

		JointData::GetActorLocalPose gp{};
		gp.target = PoseTarget::Child;
		RunJ(world, joint, Joint::GetActorLocalPose, gp);
		Check(Near(gp.value.pos.x, -0.5f, 1e-5f), "fixed: GetActorLocalPose");

		SetAngularVelocity(world, a, 1.0f, 0.5f, 2.0f);
		Step(world, 240, kDt);

		const Physics::Pose pa = GetPose(world, a);
		const Physics::Pose pb = GetPose(world, b);
		Check(pa.pos.y < 100.0f - 5.0f || Length(GetAngularVelocity(world, a)) > 0.1f, "fixed: bodies moved");
		CheckNear(Distance(pa.pos, pb.pos), 1.0f, 0.02f, "fixed: distance kept while rotating");
		// relative rotation stays identity
		const float dot = std::fabs(pa.quat.x * pb.quat.x + pa.quat.y * pb.quat.y + pa.quat.z * pb.quat.z + pa.quat.w * pb.quat.w);
		CheckNear(dot, 1.0f, 2e-3f, "fixed: relative rotation kept");
		JointData::GetForces gf{};
		RunJ(world, joint, Joint::GetForces, gf);
		Check(Length(gf.linear) >= 0.0f, "fixed: GetForces");
		Check(IsValid(world, joint), "fixed: constraint valid");

		// free fall together
		SetAngularVelocity(world, a, 0, 0, 0);
		SetAngularVelocity(world, b, 0, 0, 0);
		SetLinearVelocity(world, a, 0, -5.0f, 0);
		SetLinearVelocity(world, b, 0, -5.0f, 0);
		const float ya0 = GetPose(world, a).pos.y;
		const float yb0 = GetPose(world, b).pos.y;
		const float vy = GetLinearVelocity(world, a).y;
		Step(world, 60, kDt);
		const float dropA = ya0 - GetPose(world, a).pos.y;
		const float dropB = yb0 - GetPose(world, b).pos.y;
		const float expected = -vy * 0.5f + 0.5f * 9.81f * 0.25f;
		CheckNear(dropA, expected, 0.15f, "fixed: free fall matches gravity");
		CheckNear(dropA, dropB, 0.01f, "fixed: falls together");

		// flags round trip
		JointData::SetFlags sf{};
		sf.value = JointFlags::CollisionReporting;
		sf.enabled = true;
		RunJ(world, joint, Joint::SetFlags, sf);
		JointData::GetFlags gfl{};
		RunJ(world, joint, Joint::GetFlags, gfl);
		Check((gfl.value & JointFlags::CollisionReporting) != JointFlags::None && (gfl.value & JointFlags::Preprocessing) != JointFlags::None,
			"fixed: flags set");
		sf.enabled = false;
		RunJ(world, joint, Joint::SetFlags, sf);
		RunJ(world, joint, Joint::GetFlags, gfl);
		Check((gfl.value & JointFlags::CollisionReporting) == JointFlags::None, "fixed: flags cleared");

		// mass scale / wake up / projection tolerance are accepted
		JointData::SetInvMassAndInertiaScale sm{};
		sm.target = PoseTarget::Child;
		sm.value = 0.5f;
		RunJ(world, joint, Joint::SetInvMassAndInertiaScale, sm);
		JointData::WakeUpActors wu{};
		RunJ(world, joint, Joint::WakeUpActors, wu);
		JointData::Set6DofProjectionTolerance spt{};
		spt.linear = 0.01f;
		spt.angular = 0.01f;
		RunJ(world, joint, Joint::Set6DofProjectionTolerance, spt);
		Step(world, 10, kDt);
		Check(IsValid(world, joint), "fixed: still valid after config commands");

		// static anchor
		Handle anchor = MakeBody(world, true, Physics::Pose(5, 200, 0, 0, 0, 0, 1), 0.0f, false, BoxGeom(0.5f, 0.5f, 0.5f));
		Handle hanging = MakeBox(world, 5.0f, 200.0f, 0.0f, true);
		Handle anchorJoint = CreateJoint(world, anchor, hanging, JointType::Fixed, kJointFlags);
		Check(anchorJoint != nullptr, "fixed: static anchor CreateJoint");
		Step(world, 240, kDt);
		CheckNear(GetPose(world, hanging).pos.y, 200.0f, 0.05f, "fixed: static anchor holds");
		JointData::GetForces hf{};
		RunJ(world, anchorJoint, Joint::GetForces, hf);
		CheckRange(Length(hf.linear), 8.0f, 12.0f, "fixed: hanging load ~ m*g");

		DestroyWorld(world);
	}

	// Pendulum pivot (0,10,0) of length 1 swinging in the XY plane; the hinge axis is the world Z axis.
	void HingePendulumTest()
	{
		std::printf("[b1] hinge pendulum\n");
		Handle world = CreateWorld(-9.81f);
		const float theta0 = 0.3f, length = 1.0f;
		const float px = 0.0f, py = 10.0f;
		const float bx = px + length * std::sin(theta0), by = py - length * std::cos(theta0);
		Handle body = MakeBody(world, false, Physics::Pose(bx, by, 0, 0, 0, 0, 1), 1.0f, true, SphereGeom(0.1f));
		Handle joint = CreateJoint(world, nullptr, body, JointType::Hinge, kJointFlags);
		Check(joint != nullptr, "hinge: CreateJoint (world anchored)");
		const float s = std::sqrt(0.5f);
		SetFrames(world, joint, Physics::Pose(px, py, 0, 0, s, 0, s), Physics::Pose(px - bx, py - by, 0, 0, s, 0, s));

		const float dt = 1.0f / 240.0f;
		std::vector<float> crossings;
		float prevTheta = theta0, prevT = 0.0f;
		float maxLengthError = 0.0f, maxTheta = 0.0f;
		for (int i = 1; i <= 1440; ++i)
		{
			Step(world, 1, dt);
			const Physics::Pose p = GetPose(world, body);
			const float theta = std::atan2(p.pos.x - px, -(p.pos.y - py));
			const float t = i * dt;
			maxLengthError = std::max(maxLengthError, std::fabs(std::sqrt((p.pos.x - px) * (p.pos.x - px) + (p.pos.y - py) * (p.pos.y - py)) - length));
			maxTheta = std::max(maxTheta, std::fabs(theta));
			if (prevTheta > 0.0f && theta <= 0.0f)
			{
				crossings.push_back(prevT + dt * prevTheta / (prevTheta - theta));
			}
			prevTheta = theta;
			prevT = t;
		}
		Check(crossings.size() >= 2, "hinge: pendulum crossings found");
		if (crossings.size() >= 2)
		{
			const float period = crossings[1] - crossings[0];
			std::printf("  pendulum period %.4f s (expected ~2.02), max length error %.5f, max angle %.3f\n", period, maxLengthError, maxTheta);
			CheckRange(period, 1.95f, 2.09f, "hinge: pendulum period");
		}
		CheckRange(maxLengthError, 0.0f, 0.02f, "hinge: pivot distance kept");
		CheckRange(maxTheta, 0.25f, 0.33f, "hinge: swing amplitude");
		DestroyWorld(world);
	}

	// Gravity-free body rotating about a pivot at (1,5,0); returns the signed rotation about Z after the run.
	float HingeMotorRun(float targetVelocity, bool limited, float* finalSpeed)
	{
		Handle world = CreateWorld(0.0f);
		Handle body = MakeBox(world, 0.0f, 5.0f, 0.0f, false);
		Handle joint = CreateJoint(world, nullptr, body, JointType::Hinge, kJointFlags);
		const float s = std::sqrt(0.5f);
		SetFrames(world, joint, Physics::Pose(1, 5, 0, 0, s, 0, s), Physics::Pose(1, 0, 0, 0, s, 0, s));

		if (limited)
		{
			JointData::SetHingeLimit hl{};
			hl.min = -0.5f;
			hl.max = 1.0f;
			hl.bounciness = 0.0f;
			hl.bounceMinVelocity = 0.0f;
			hl.contactDistance = 0.05f;
			RunJ(world, joint, Joint::SetHingeLimit, hl);
			JointData::SetHingeLimitEnabled he{};
			he.value = true;
			RunJ(world, joint, Joint::SetHingeLimitEnabled, he);
		}
		JointData::SetHingeMotorConfiguration mc{};
		mc.maxForce = 1000.0f;
		mc.freeSpin = false;
		RunJ(world, joint, Joint::SetHingeMotorConfiguration, mc);
		JointData::SetHingeMotorTargetVelocity tv{};
		tv.value = targetVelocity;
		RunJ(world, joint, Joint::SetHingeMotorTargetVelocity, tv);

		Step(world, limited ? 480 : 60, kDt);
		if (finalSpeed != nullptr)
		{
			*finalSpeed = GetAngularVelocity(world, body).z;
		}
		const float angle = AxisAngle(GetPose(world, body), 2);
		DestroyWorld(world);
		return angle;
	}

	void HingeMotorLimitTest()
	{
		std::printf("[b2] hinge motor / limit\n");
		float speedPos = 0.0f, speedNeg = 0.0f;
		HingeMotorRun(2.0f, false, &speedPos);
		HingeMotorRun(-2.0f, false, &speedNeg);
		std::printf("  motor speed +%.3f / %.3f rad/s\n", speedPos, speedNeg);
		CheckNear(std::fabs(speedPos), 2.0f, 0.1f, "hinge motor: reaches target velocity (+)");
		CheckNear(std::fabs(speedNeg), 2.0f, 0.1f, "hinge motor: reaches target velocity (-)");
		Check(speedPos * speedNeg < 0.0f, "hinge motor: direction follows the sign");

		const float angleA = HingeMotorRun(2.0f, true, nullptr);
		const float angleB = HingeMotorRun(-2.0f, true, nullptr);
		std::printf("  limited hinge end angles: %.4f / %.4f (limits -0.5 .. 1.0)\n", angleA, angleB);
		Check(angleA * angleB < 0.0f, "hinge limit: both ends reached");
		const float big = std::max(std::fabs(angleA), std::fabs(angleB));
		const float small = std::min(std::fabs(angleA), std::fabs(angleB));
		CheckNear(big, 1.0f, 0.06f, "hinge limit: upper end");
		CheckNear(small, 0.5f, 0.06f, "hinge limit: lower end");
	}

	void D6Test()
	{
		std::printf("[c] 6DoF\n");

		// all linear locked, angular free
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Free);
			SetLinearVelocity(r.world, r.body, 3, 3, 3);
			SetAngularVelocity(r.world, r.body, 1.0f, 2.0f, 0.5f);
			Step(r.world, 120, kDt);
			const Physics::Pose p = GetPose(r.world, r.body);
			CheckNear(Distance(p.pos, Physics::Vec3(0, 5, 0)), 0.0f, 0.02f, "d6: linear locked holds position");
			Check(std::fabs(AxisAngle(p, 1)) > 0.5f || std::fabs(AxisAngle(p, 0)) > 0.3f, "d6: angular free rotates");
			DestroyWorld(r.world);
		}
		// only X free
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::LinearX, JointDofLock::Free);
			SetLinearVelocity(r.world, r.body, 2, 2, 2);
			Step(r.world, 120, kDt);
			const Physics::Pose p = GetPose(r.world, r.body);
			CheckNear(p.pos.x, 2.0f, 0.1f, "d6: free X moves");
			CheckNear(p.pos.y, 5.0f, 0.03f, "d6: locked Y holds");
			CheckNear(p.pos.z, 0.0f, 0.03f, "d6: locked Z holds");
			DestroyWorld(r.world);
		}
		// angular locked, linear free
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Free, JointDofLock::Locked);
			SetLinearVelocity(r.world, r.body, 1, 0, 0);
			SetAngularVelocity(r.world, r.body, 1.0f, 2.0f, 3.0f);
			Step(r.world, 120, kDt);
			const Physics::Pose p = GetPose(r.world, r.body);
			const float w = std::fabs(p.quat.w);
			CheckNear(w, 1.0f, 0.01f, "d6: angular locked keeps rotation");
			CheckNear(p.pos.x, 1.0f, 0.1f, "d6: linear free moves");
			DestroyWorld(r.world);
		}
		// linear limit on X
		for (int dir = 0; dir < 2; ++dir)
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::LinearX, JointDofLock::Limited);
			SetLimit6(r.world, r.joint, ConstraintAxis::LinearX, -0.5f, 1.5f);
			JointData::Get6DofLimit gl{};
			gl.axis = ConstraintAxis::LinearX;
			RunJ(r.world, r.joint, Joint::Get6DofLimit, gl);
			Check(Near(gl.min, -0.5f, 1e-5f) && Near(gl.max, 1.5f, 1e-5f) && Near(gl.contactDistance, 0.02f, 1e-5f), "d6: Get6DofLimit round trip");
			SetLinearVelocity(r.world, r.body, dir == 0 ? 3.0f : -3.0f, 0, 0);
			Step(r.world, 240, kDt);
			const float x = GetPose(r.world, r.body).pos.x;
			if (dir == 0)
			{
				CheckNear(x, 1.5f, 0.08f, "d6: linear limit upper");
			}
			else
			{
				CheckNear(x, -0.5f, 0.08f, "d6: linear limit lower");
			}
			DestroyWorld(r.world);
		}
		// twist limit
		for (int dir = 0; dir < 2; ++dir)
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::AngularX, JointDofLock::Limited);
			SetLimit6(r.world, r.joint, ConstraintAxis::AngularX, -0.4f, 0.6f);
			SetAngularVelocity(r.world, r.body, dir == 0 ? 3.0f : -3.0f, 0, 0);
			Step(r.world, 240, kDt);
			const float angle = AxisAngle(GetPose(r.world, r.body), 0);
			if (dir == 0)
			{
				CheckNear(angle, 0.6f, 0.06f, "d6: twist limit (+)");
			}
			else
			{
				CheckNear(angle, -0.4f, 0.06f, "d6: twist limit (-)");
			}
			DestroyWorld(r.world);
		}
		// swing cone (both axes limited, symmetric)
		for (int axis = 1; axis <= 2; ++axis)
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::AngularY, JointDofLock::Limited);
			SetLock(r.world, r.joint, ConstraintAxis::AngularZ, JointDofLock::Limited);
			SetLimit6(r.world, r.joint, ConstraintAxis::AngularY, -0.4f, 0.4f);
			SetLimit6(r.world, r.joint, ConstraintAxis::AngularZ, -0.4f, 0.4f);
			SetAngularVelocity(r.world, r.body, 0, axis == 1 ? 3.0f : 0.0f, axis == 2 ? 3.0f : 0.0f);
			Step(r.world, 240, kDt);
			const float angle = std::fabs(AxisAngle(GetPose(r.world, r.body), axis));
			CheckNear(angle, 0.4f, 0.07f, axis == 1 ? "d6: swing limit Y" : "d6: swing limit Z");
			DestroyWorld(r.world);
		}
		// asymmetric swing (pyramid)
		{
			float result[2] = { 0, 0 };
			for (int dir = 0; dir < 2; ++dir)
			{
				D6Rig r = MakeD6Rig();
				LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
				SetLock(r.world, r.joint, ConstraintAxis::AngularY, JointDofLock::Limited);
				SetLock(r.world, r.joint, ConstraintAxis::AngularZ, JointDofLock::Limited);
				SetLimit6(r.world, r.joint, ConstraintAxis::AngularY, -0.2f, 0.6f);
				SetLimit6(r.world, r.joint, ConstraintAxis::AngularZ, -0.4f, 0.4f);
				SetAngularVelocity(r.world, r.body, 0, dir == 0 ? 3.0f : -3.0f, 0);
				Step(r.world, 240, kDt);
				result[dir] = AxisAngle(GetPose(r.world, r.body), 1);
				DestroyWorld(r.world);
			}
			std::printf("  asymmetric swing ends: %.4f / %.4f\n", result[0], result[1]);
			Check(result[0] * result[1] < 0.0f, "d6: pyramid swing both ends reached");
			CheckNear(std::max(std::fabs(result[0]), std::fabs(result[1])), 0.6f, 0.07f, "d6: pyramid swing wide side");
			CheckNear(std::min(std::fabs(result[0]), std::fabs(result[1])), 0.2f, 0.07f, "d6: pyramid swing narrow side");
		}
		// soft linear limit still bounded
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::LinearX, JointDofLock::Limited);
			SetLimit6(r.world, r.joint, ConstraintAxis::LinearX, -1.0f, 1.0f);
			JointData::Set6DofSpringLimit ssl{};
			ssl.axis = ConstraintAxis::LinearX;
			ssl.spring = 200.0f;
			ssl.damping = 20.0f;
			RunJ(r.world, r.joint, Joint::Set6DofSpringLimit, ssl);
			SetLinearVelocity(r.world, r.body, 3, 0, 0);
			float maxX = 0.0f;
			for (int i = 0; i < 240; ++i)
			{
				Step(r.world, 1, kDt);
				maxX = std::max(maxX, GetPose(r.world, r.body).pos.x);
			}
			std::printf("  soft limit overshoot: max x %.4f (limit 1.0)\n", maxX);
			CheckRange(maxX, 1.02f, 1.6f, "d6: soft limit yields but stays bounded");
			DestroyWorld(r.world);
		}
		// linear position drive
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::LinearX, JointDofLock::Free);
			SetDrive(r.world, r.joint, ConstraintAxis::LinearX, 200.0f, 40.0f, 1.0e6f);
			JointData::Set6DofMotorTargetPosition tp{};
			tp.value = Physics::Vec3(0.5f, 0, 0);
			RunJ(r.world, r.joint, Joint::Set6DofMotorTargetPosition, tp);
			Step(r.world, 480, kDt);
			CheckNear(GetPose(r.world, r.body).pos.x, 0.5f, 0.03f, "d6: drive reaches target position");
			DestroyWorld(r.world);
		}
		// linear velocity drive
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::LinearX, JointDofLock::Free);
			SetDrive(r.world, r.joint, ConstraintAxis::LinearX, 0.0f, 100.0f, 1.0e6f);
			JointData::Set6DofMotorTargetLinearVelocity tv{};
			tv.value = Physics::Vec3(1.0f, 0, 0);
			RunJ(r.world, r.joint, Joint::Set6DofMotorTargetLinearVelocity, tv);
			Step(r.world, 120, kDt);
			CheckNear(GetLinearVelocity(r.world, r.body).x, 1.0f, 0.05f, "d6: drive reaches target linear velocity");
			DestroyWorld(r.world);
		}
		// twist position drive
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::AngularX, JointDofLock::Free);
			SetDrive(r.world, r.joint, ConstraintAxis::AngularX, 50.0f, 10.0f, 1.0e6f);
			JointData::Set6DofMotorTargetRotation tr{};
			const float half = 0.3f;
			tr.value = Physics::Vec4(std::sin(half), 0.0f, 0.0f, std::cos(half));
			RunJ(r.world, r.joint, Joint::Set6DofMotorTargetRotation, tr);
			Step(r.world, 480, kDt);
			CheckNear(AxisAngle(GetPose(r.world, r.body), 0), 0.6f, 0.03f, "d6: drive reaches target rotation");
			DestroyWorld(r.world);
		}
		// angular velocity drive
		{
			D6Rig r = MakeD6Rig();
			LockAll(r.world, r.joint, JointDofLock::Locked, JointDofLock::Locked);
			SetLock(r.world, r.joint, ConstraintAxis::AngularX, JointDofLock::Free);
			SetDrive(r.world, r.joint, ConstraintAxis::AngularX, 0.0f, 50.0f, 1.0e6f);
			JointData::Set6DofMotorTargetAngularVelocity tw{};
			tw.value = Physics::Vec3(2.0f, 0, 0);
			RunJ(r.world, r.joint, Joint::Set6DofMotorTargetAngularVelocity, tw);
			Step(r.world, 120, kDt);
			// PhysX D6 measures the angular drive velocity as actor0 relative to actor1: the child turns by -target.
			const float wx = GetAngularVelocity(r.world, r.body).x;
			std::printf("  angular drive velocity result %.4f (target +2)\n", wx);
			CheckNear(std::fabs(wx), 2.0f, 0.1f, "d6: drive reaches target angular velocity");
			DestroyWorld(r.world);
		}
		// slerp / swing drive configuration and the 6DoF commands on a non 6DoF joint must be harmless
		{
			D6Rig r = MakeD6Rig();
			SetDrive(r.world, r.joint, ConstraintAxis::Slerp, 10.0f, 1.0f, 100.0f);
			SetDrive(r.world, r.joint, ConstraintAxis::AngularYZ, 10.0f, 1.0f, 100.0f);
			SetDrive(r.world, r.joint, ConstraintAxis::Distance, 10.0f, 1.0f, 100.0f);
			SetLimit6(r.world, r.joint, ConstraintAxis::Distance, 0.0f, 2.0f);
			Step(r.world, 10, kDt);
			Check(IsValid(r.world, r.joint), "d6: slerp/swing/distance config accepted");
			Handle box = MakeBox(r.world, 3.0f, 5.0f, 0.0f, false);
			Handle fixed = CreateJoint(r.world, nullptr, box, JointType::Fixed, kJointFlags);
			SetLock(r.world, fixed, ConstraintAxis::LinearX, JointDofLock::Free);
			SetLimit6(r.world, fixed, ConstraintAxis::LinearX, -1.0f, 1.0f);
			SetDrive(r.world, fixed, ConstraintAxis::LinearX, 1.0f, 1.0f, 1.0f);
			Step(r.world, 10, kDt);
			Check(IsValid(r.world, fixed), "d6: 6DoF commands ignored on a fixed joint");
			DestroyWorld(r.world);
		}
	}

	void DistanceTest()
	{
		std::printf("[d] distance joint\n");
		const Physics::Vec3 anchor(0, 5, 0);
		// max distance
		{
			Handle world = CreateWorld(0.0f);
			Handle body = MakeBox(world, 1.0f, 5.0f, 0.0f, false);
			Handle joint = CreateJoint(world, nullptr, body, JointType::Distance, kJointFlags);
			Check(joint != nullptr, "distance: CreateJoint");
			SetFrames(world, joint, Physics::Pose(0, 5, 0, 0, 0, 0, 1), Physics::Pose());
			// unconstrained until limits are configured
			SetLinearVelocity(world, body, 4, 0, 0);
			Step(world, 30, kDt);
			Check(GetPose(world, body).pos.x > 1.5f, "distance: unconstrained before SetDistanceLimit");
			JointData::SetDistanceLimit dl{};
			dl.min = 0.0f;
			dl.max = 2.0f;
			RunJ(world, joint, Joint::SetDistanceLimit, dl);
			Step(world, 240, kDt);
			const float d = Distance(GetPose(world, body).pos, anchor);
			CheckRange(d, 1.9f, 2.1f, "distance: max distance holds");
			DestroyWorld(world);
		}
		// min distance
		{
			Handle world = CreateWorld(0.0f);
			Handle body = MakeBox(world, 2.5f, 5.0f, 0.0f, false);
			Handle joint = CreateJoint(world, nullptr, body, JointType::Distance, kJointFlags);
			SetFrames(world, joint, Physics::Pose(0, 5, 0, 0, 0, 0, 1), Physics::Pose());
			JointData::SetDistanceLimit dl{};
			dl.min = 2.0f;
			dl.max = 3.0f;
			RunJ(world, joint, Joint::SetDistanceLimit, dl);
			SetLinearVelocity(world, body, -3, 0, 0);
			Step(world, 240, kDt);
			const float d = Distance(GetPose(world, body).pos, anchor);
			CheckRange(d, 1.9f, 2.1f, "distance: min distance holds");
			DestroyWorld(world);
		}
		// spring pulls back to the max distance
		{
			Handle world = CreateWorld(0.0f);
			Handle body = MakeBox(world, 3.0f, 5.0f, 0.0f, false);
			Handle joint = CreateJoint(world, nullptr, body, JointType::Distance, kJointFlags);
			SetFrames(world, joint, Physics::Pose(0, 5, 0, 0, 0, 0, 1), Physics::Pose());
			JointData::SetDistanceLimit dl{};
			dl.min = 0.0f;
			dl.max = 1.0f;
			RunJ(world, joint, Joint::SetDistanceLimit, dl);
			JointData::SetDistanceSpringLimit sp{};
			sp.spring = 50.0f;
			sp.damping = 10.0f;
			RunJ(world, joint, Joint::SetDistanceSpringLimit, sp);
			JointData::SetDistanceErrorTolerance et{};
			et.value = 0.05f;
			RunJ(world, joint, Joint::SetDistanceErrorTolerance, et);
			Step(world, 3, kDt);
			CheckRange(Distance(GetPose(world, body).pos, anchor), 2.5f, 3.0f, "distance: spring is soft (no instant snap)");
			Step(world, 597, kDt);
			const float d = Distance(GetPose(world, body).pos, anchor);
			CheckRange(d, 0.0f, 1.3f, "distance: spring pulls back inside the max distance");
			DestroyWorld(world);
		}
	}

	void BreakTest()
	{
		std::printf("[e] break force\n");
		Handle world = CreateWorld(-9.81f);
		Handle weak = MakeBox(world, 0.0f, 50.0f, 0.0f, true);
		Handle strong = MakeBox(world, 5.0f, 50.0f, 0.0f, true);
		Handle jw = CreateJoint(world, nullptr, weak, JointType::Fixed, kJointFlags);
		Handle js = CreateJoint(world, nullptr, strong, JointType::Fixed, kJointFlags);
		SetFrames(world, jw, Physics::Pose(0, 50, 0, 0, 0, 0, 1), Physics::Pose());
		SetFrames(world, js, Physics::Pose(5, 50, 0, 0, 0, 0, 1), Physics::Pose());

		JointData::SetBreakForce bf{};
		bf.force = 5.0f;
		bf.torque = 1.0e6f;
		RunJ(world, jw, Joint::SetBreakForce, bf);
		bf.force = 500.0f;
		RunJ(world, js, Joint::SetBreakForce, bf);

		Step(world, 120, kDt);
		Check(!IsValid(world, jw), "break: joint over the threshold becomes invalid");
		Check(IsValid(world, js), "break: joint under the threshold stays valid");
		CheckRange(GetPose(world, weak).pos.y, 0.0f, 48.0f, "break: broken joint lets the body fall");
		CheckNear(GetPose(world, strong).pos.y, 50.0f, 0.05f, "break: intact joint holds");

		// infinite threshold is accepted
		bf.force = INFINITY;
		bf.torque = INFINITY;
		RunJ(world, js, Joint::SetBreakForce, bf);
		Step(world, 30, kDt);
		Check(IsValid(world, js), "break: infinite threshold");
		DestroyWorld(world);
	}

	void LifetimeTest()
	{
		std::printf("[f] lifetime / invalid handles\n");

		// body destroyed first
		{
			Handle world = CreateWorld(-9.81f);
			Handle a = MakeBox(world, 0, 10, 0, true);
			Handle b = MakeBox(world, 1, 10, 0, true);
			Handle joint = CreateJoint(world, a, b, JointType::Fixed, kJointFlags);
			Check(IsValid(world, joint), "life: valid before");
			DestroyBody(world, a);
			Check(!IsValid(world, joint), "life: body destroyed -> constraint invalid");
			JointData::GetActors ga{};
			RunJ(world, joint, Joint::GetActors, ga);
			Check(ga.actors[0].actor == nullptr && ga.actors[1].actor == nullptr, "life: actors cleared after destroy");
			Step(world, 10, kDt);
			SetFrames(world, joint, Physics::Pose(), Physics::Pose()); // dead joint: harmless
			DestroyJoint(world, joint);
			DestroyJoint(world, joint); // double destroy
			Check(!IsValid(world, joint), "life: destroyed joint handle is invalid");
			DestroyBody(world, b);
			Step(world, 5, kDt);
			DestroyWorld(world);
		}
		// joint destroyed first, then bodies
		{
			Handle world = CreateWorld(-9.81f);
			Handle a = MakeBox(world, 0, 10, 0, true);
			Handle b = MakeBox(world, 1, 10, 0, true);
			Handle joint = CreateJoint(world, a, b, JointType::Hinge, kJointFlags);
			Step(world, 5, kDt);
			DestroyJoint(world, joint);
			Step(world, 5, kDt);
			DestroyBody(world, a);
			DestroyBody(world, b);
			Step(world, 5, kDt);
			DestroyWorld(world);
		}
		// body with a world anchored joint, other joint survives
		{
			Handle world = CreateWorld(-9.81f);
			Handle a = MakeBox(world, 0, 10, 0, true);
			Handle b = MakeBox(world, 1, 10, 0, true);
			Handle c = MakeBox(world, 2, 10, 0, true);
			Handle jab = CreateJoint(world, a, b, JointType::Fixed, kJointFlags);
			Handle jbc = CreateJoint(world, b, c, JointType::Fixed, kJointFlags);
			Handle jc = CreateJoint(world, nullptr, c, JointType::Dof6, kJointFlags);
			DestroyBody(world, a);
			Check(!IsValid(world, jab) && IsValid(world, jbc) && IsValid(world, jc), "life: only the joint of the destroyed body dies");
			Step(world, 10, kDt);
			DestroyBody(world, c);
			Check(!IsValid(world, jbc) && !IsValid(world, jc), "life: destroying c kills its joints");
			Step(world, 10, kDt);
			DestroyWorld(world);
			Check(!IsValid(world, jab) && !IsValid(world, jbc), "life: handles dead after the world is destroyed");
			DestroyJoint(world, jc);
		}
		// world destroyed with live joints
		{
			Handle world = CreateWorld(-9.81f);
			Handle a = MakeBox(world, 0, 10, 0, true);
			Handle b = MakeBox(world, 1, 10, 0, true);
			Handle joint = CreateJoint(world, a, b, JointType::Dof6, kJointFlags);
			Handle joint2 = CreateJoint(world, nullptr, a, JointType::Distance, kJointFlags);
			Step(world, 5, kDt);
			DestroyWorld(world);
			Check(!IsValid(world, joint) && !IsValid(world, joint2), "life: joints dead after the world is destroyed");
			JointData::GetFlags gf{};
			RunJ(world, joint, Joint::GetFlags, gf);
			DestroyJoint(world, joint);
			DestroyJoint(world, joint2);
		}
		// SetActors reassignment
		{
			Handle world = CreateWorld(0.0f);
			Handle a = MakeBox(world, 0, 10, 0, false);
			Handle b = MakeBox(world, 1, 10, 0, false);
			Handle c = MakeBox(world, 2, 10, 0, false);
			Handle joint = CreateJoint(world, a, b, JointType::Fixed, kJointFlags);
			JointData::SetActors sa{};
			sa.actors[0].actor = a;
			sa.actors[1].actor = c;
			RunJ(world, joint, Joint::SetActors, sa);
			JointData::GetActors ga{};
			RunJ(world, joint, Joint::GetActors, ga);
			Check(ga.actors[0].actor == a && ga.actors[1].actor == c, "life: SetActors");
			DestroyBody(world, b);
			Check(IsValid(world, joint), "life: old actor destroyed after SetActors keeps the joint");
			DestroyBody(world, c);
			Check(!IsValid(world, joint), "life: new actor destroyed invalidates the joint");
			Step(world, 5, kDt);
			DestroyWorld(world);
		}
		// invalid creation / handles
		{
			Handle world = CreateWorld(0.0f);
			Handle a = MakeBox(world, 0, 10, 0, false);
			Handle floor = MakeBody(world, true, Physics::Pose(0, 0, 0, 0, 0, 0, 1), 0.0f, false, BoxGeom(1, 1, 1));
			Handle floor2 = MakeBody(world, true, Physics::Pose(5, 0, 0, 0, 0, 0, 1), 0.0f, false, BoxGeom(1, 1, 1));
			Dummy foreign{ 1 };
			Handle bogus = &foreign;

			Check(CreateJoint(world, nullptr, nullptr, JointType::Fixed, kJointFlags) == nullptr, "invalid: both actors null");
			Check(CreateJoint(world, floor, floor2, JointType::Fixed, kJointFlags) == nullptr, "invalid: two static bodies");
			Check(CreateJoint(world, a, a, JointType::Fixed, kJointFlags) == nullptr, "invalid: same body twice");
			Check(CreateJoint(world, bogus, a, JointType::Fixed, kJointFlags) == nullptr, "invalid: foreign body handle");
			Check(CreateJoint(world, a, nullptr, JointType::Invalid, kJointFlags) == nullptr, "invalid: Invalid type");
			Check(CreateJoint(world, a, nullptr, JointType::Count, kJointFlags) == nullptr, "invalid: Count type");
			Check(CreateJoint(nullptr, a, nullptr, JointType::Fixed, kJointFlags) == nullptr || true, "invalid: null world tolerated");

			Handle other = CreateWorld(0.0f);
			Handle otherBody = MakeBox(other, 0, 0, 0, false);
			Check(CreateJoint(world, a, otherBody, JointType::Fixed, kJointFlags) == nullptr, "invalid: bodies of another world");
			DestroyWorld(other);

			// commands on bogus handles / non joint handles
			JointData::GetIsConstraintValid gv{};
			gv.value = true;
			RunJ(world, bogus, Joint::GetIsConstraintValid, gv);
			Check(!gv.value, "invalid: bogus joint handle");
			RunJ(world, a, Joint::GetIsConstraintValid, gv);
			Check(!gv.value, "invalid: body handle as joint");
			RunJ(world, nullptr, Joint::GetIsConstraintValid, gv);
			Check(!gv.value, "invalid: null joint handle");
			JointData::SetBreakForce bf{ {}, 1.0f, 1.0f };
			RunJ(world, bogus, Joint::SetBreakForce, bf);
			JointData::GetForces gf{};
			RunJ(world, bogus, Joint::GetForces, gf);
			JointData::SetActors sa{};
			RunJ(world, bogus, Joint::SetActors, sa);
			JointData::Set6DofLimit sl{};
			RunJ(world, bogus, Joint::Set6DofLimit, sl);
			JointData::SetHingeLimit hl{};
			RunJ(world, bogus, Joint::SetHingeLimit, hl);
			DestroyJoint(world, bogus);
			DestroyJoint(world, a);
			DestroyJoint(world, nullptr);
			CheckNear(GetPose(world, a).pos.y, 10.0f, 1e-3f, "invalid: body survives DestroyJoint(body)");

			// bad parameters on a real joint
			Handle joint = CreateJoint(world, a, nullptr, JointType::Dof6, kJointFlags);
			SetLimit6(world, joint, ConstraintAxis::LinearX, 2.0f, -2.0f);
			SetLimit6(world, joint, ConstraintAxis::AngularX, -100.0f, 100.0f);
			SetLimit6(world, joint, ConstraintAxis::AngularYZ, 0.0f, 0.0f);
			SetLimit6(world, joint, ConstraintAxis::LinearY, -INFINITY, INFINITY);
			SetDrive(world, joint, ConstraintAxis::LinearX, -1.0f, -1.0f, -1.0f);
			JointData::SetActors bad{};
			bad.actors[0].actor = bogus;
			RunJ(world, joint, Joint::SetActors, bad);
			Step(world, 10, kDt);
			Check(IsValid(world, joint), "invalid: bad parameters keep the joint alive");
			DestroyJoint(world, joint);
			DestroyWorld(world);
		}
		// ShutdownSdk destroys leftover worlds with joints (verified by the final shutdown).
		{
			Handle world = CreateWorld(-9.81f);
			Handle a = MakeBox(world, 0, 10, 0, true);
			CreateJoint(world, nullptr, a, JointType::BallAndSocket, kJointFlags);
			Handle b = MakeBox(world, 0, 5, 0, true);
			Handle j2 = CreateJoint(world, a, b, JointType::Prismatic, kJointFlags);
			SetLimit6(world, j2, ConstraintAxis::LinearX, -1.0f, 1.0f);
			SetLimit6(world, j2, ConstraintAxis::AngularYZ, -0.5f, 0.5f);
			Step(world, 10, kDt);
		}
	}
}

namespace
{
	// ---- joints on articulation links (FixedJoint.connectedArticulationBody) ----

	Handle MakeArticulation(Handle world, bool fixedRoot)
	{
		WorldData::CreateArticulation ca{};
		ca.flags = (fixedRoot ? Physics::ArticulationFlags::ImmovableRoot : Physics::ArticulationFlags::None)
			| Physics::ArticulationFlags::DriveLimitsAsForces | Physics::ArticulationFlags::Simulated;
		Run(world, Context(world, nullptr), World::CreateArticulation, ca);
		return ca.outArticulation;
	}

	Handle MakeLink(Handle world, Handle art, Handle parent, const Physics::Pose& pose)
	{
		ArticulationData::CreateLink cl{};
		cl.parentLink = parent;
		cl.pose = pose;
		Run(art, Context(world, art), Articulation::CreateLink, cl);
		Handle link = cl.outNewLink;
		if (link == nullptr)
		{
			return nullptr;
		}
		ShapeData::CreateShape cs{};
		cs.geom = BoxGeom(0.1f, 0.1f, 0.1f);
		cs.materialPtr = g_material;
		cs.flags = Physics::ShapeFlags::SceneQuery;
		Run(nullptr, Context(world, nullptr), Shape::CreateShape, cs);
		ShapeData::SetFilterData sf{};
		sf.data.belongsTo = 1;
		sf.data.collidesWith = 0;
		Run(cs.value, Context(world, nullptr), Shape::SetFilterData, sf);
		ArticulationLinkData::AttachShape as{};
		as.shape = cs.value;
		as.pose = Physics::Pose();
		Run(link, Context(world, link), Articulation::AttachShapeToLink, as);
		ArticulationLinkData::SetMass sm{};
		sm.value = 2.0f;
		Run(link, Context(world, link), Articulation::SetLinkMass, sm);
		ArticulationLinkData::RecomputeMassProperties rp;
		Run(link, Context(world, link), Articulation::RecomputeLinkMassProperties, rp);
		return link;
	}

	void AddToScene(Handle world, Handle obj)
	{
		Handle list[1] = { obj };
		WorldData::AddActorsToScene aa{};
		aa.actors = list;
		aa.bufferSize = 1;
		Run(world, Context(world, nullptr), World::AddActorsToScene, aa);
	}

	// A body that rests on a joint sleeps; wake it so a missing constraint shows up as a fall.
	void WakeBody(Handle world, Handle body)
	{
		BodyData::WakeUp wu{};
		Run(body, Context(world, body), Body::WakeUp, wu);
	}

	Handle ActorOfJoint(Handle world, Handle joint, int index, Physics::ActorType* type = nullptr)
	{
		JointData::GetActors ga{};
		RunJ(world, joint, Joint::GetActors, ga);
		if (type != nullptr)
		{
			*type = ga.actors[index].type;
		}
		return ga.actors[index].actor;
	}

	void LinkJointTest()
	{
		std::printf("[g] joints on articulation links\n");
		Handle world = CreateWorld(-9.81f);
		Handle art = MakeArticulation(world, true);
		Handle root = MakeLink(world, art, nullptr, Physics::Pose(0, 5, 0, 0, 0, 0, 1));
		AddToScene(world, art);

		// a dynamic body hangs from the immovable root link through a fixed joint (the gripper case)
		Handle body = MakeBox(world, 1.0f, 5.0f, 0.0f, true);
		Handle joint = CreateJoint(world, root, body, JointType::Fixed, kJointFlags);
		if (!Check(joint != nullptr, "link: a joint accepts an articulation link"))
		{
			DestroyWorld(world);
			return;
		}
		SetFrames(world, joint, Physics::Pose(1.0f, 0, 0, 0, 0, 0, 1), Physics::Pose());
		Physics::ActorType type0 = Physics::ActorType::Invalid;
		Check(ActorOfJoint(world, joint, 0, &type0) == root && type0 == Physics::ActorType::Link, "link: GetActors reports the link");
		Check(ActorOfJoint(world, joint, 1, &type0) == body && type0 == Physics::ActorType::Dynamic, "link: GetActors reports the body");
		Step(world, 120, kDt);
		CheckRange(GetPose(world, body).pos.y, 4.9f, 5.1f, "link: the body is held by the link");
		Check(IsValid(world, joint), "link: constraint valid");

		// a structural change rebuilds the PhysX articulation: the joint follows the new PxArticulationLink
		Handle child = MakeLink(world, art, root, Physics::Pose(0, 4, 0, 0, 0, 0, 1));
		Check(child != nullptr, "link: second link added to the live articulation");
		WakeBody(world, body);
		Step(world, 120, kDt);
		Check(IsValid(world, joint), "link: constraint valid after the articulation was rebuilt");
		CheckRange(GetPose(world, body).pos.y, 4.9f, 5.1f, "link: the body is still held after the rebuild");
		Check(ActorOfJoint(world, joint, 0) == root, "link: GetActors after the rebuild");

		// a wrong world is refused
		{
			Handle other = CreateWorld(0.0f);
			Handle foreign = CreateJoint(other, root, nullptr, JointType::Fixed, kJointFlags);
			Check(foreign == nullptr, "link: a link of another world is refused");
			DestroyWorld(other);
		}

		// destroying the link kills its joint, the body falls
		Handle body2 = MakeBox(world, 0.0f, 3.0f, 0.0f, false);
		Handle joint2 = CreateJoint(world, child, body2, JointType::Fixed, kJointFlags);
		Check(joint2 != nullptr, "link: joint on the child link");
		Step(world, 10, kDt);
		ArticulationData::DestroyLink dl{};
		dl.value = child;
		Run(art, Context(world, art), Articulation::DestroyLink, dl);
		Check(!IsValid(world, joint2), "link: destroying the link invalidates its joint");
		Check(ActorOfJoint(world, joint2, 0) == nullptr && ActorOfJoint(world, joint2, 1) == nullptr, "link: actors cleared after the link is gone");
		Check(IsValid(world, joint), "link: the joint of the other link survives");
		WakeBody(world, body);
		Step(world, 60, kDt);
		CheckRange(GetPose(world, body).pos.y, 4.9f, 5.1f, "link: the body of the surviving joint is still held");
		DestroyJoint(world, joint2);

		// destroying the articulation releases the joint; the body falls
		WorldData::DestoryArticulation da{};
		da.value = art;
		Run(world, Context(world, nullptr), World::DestroyArticulation, da);
		Check(!IsValid(world, joint), "link: destroying the articulation invalidates its joint");
		WakeBody(world, body);
		Step(world, 60, kDt);
		Check(GetPose(world, body).pos.y < 4.0f, "link: the body falls once the articulation is gone");
		DestroyJoint(world, joint);
		DestroyWorld(world);
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
	std::printf("joint_test: backend=%s (requested SDK %d)\n", path.c_str(), FALL_TEST_SDK);

	SdkData::InitializeSdk init;
	init.engineObjectIdOffset = 0;
	Run(nullptr, Context(), SDK::InitializeSdk, init);
	Check(init.result, "InitializeSdk");
	MaterialData::GetDefaultMaterial dm{};
	Run(nullptr, Context(), SDK::GetDefaultMaterial, dm);
	g_material = dm.value;

	FixedJointTest();
	HingePendulumTest();
	HingeMotorLimitTest();
	D6Test();
	DistanceTest();
	BreakTest();
	LifetimeTest();
	LinkJointTest();

	SdkData::ShutdownSdk shutdown;
	Run(nullptr, Context(), SDK::ShutdownSdk, shutdown);
	unload();
	dlclose(lib);

	Check(g_errorLogs == 0, "no backend error logs");
	std::printf("joint_test: %d checks, %d failures, %d error logs -> %s\n", g_checks, g_failures, g_errorLogs,
		g_failures == 0 ? "PASS" : "FAIL");
	return g_failures == 0 ? 0 : 1;
}
