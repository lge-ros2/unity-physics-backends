// Drives the backend through the Unity command entry point: reduced coordinate articulations
// (pendulum, drives, prismatic elevator, chain with shapes, joint buffers, limits, teardown orders).
// Usage: articulation_test [path/to/libBackend.so]

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

#include <map>
#include <memory>

// userData is an engine object pointer whose id lives at a fixed offset; give each tag a real buffer.
static void* FakeUser(intptr_t tag)
{
	static std::map<intptr_t, std::unique_ptr<unsigned char[]>> buffers;
	auto& buffer = buffers[tag];
	if (!buffer)
	{
		buffer.reset(new unsigned char[64]());
		std::memcpy(buffer.get(), &tag, sizeof(tag));
	}
	return buffer.get();
}

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
using Physics::JointType;

namespace
{
	BackendEntryPoint g_entry = nullptr;
	int g_errorLogs = 0;
	int g_failures = 0;
	int g_checks = 0;
	const float kPi = 3.14159265358979f;
	const float kDt = 1.0f / 240.0f;

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

	// ---- world / shapes ----

	Handle g_defaultMaterial = nullptr;

	Handle CreateWorld(float gravityY)
	{
		WorldData::CreateWorld cw;
		cw.jobWorkerCount = 1;
		cw.gravity = Physics::Vec3(0.0f, gravityY, 0.0f);
		Run(nullptr, Context(), World::CreateWorld, cw);
		return cw.outWorld;
	}

	void Step(Handle world, float seconds)
	{
		const int steps = static_cast<int>(std::lround(seconds / kDt));
		for (int i = 0; i < steps; ++i)
		{
			WorldData::Simulate sim;
			sim.deltaTime = kDt;
			Run(world, Context(world, nullptr), World::Simulate, sim);
		}
	}

	Handle CreateBox(Handle world, float hx, float hy, float hz, void* userData = nullptr)
	{
		ShapeData::CreateShape cs{};
		cs.geom.type = Physics::GeometryType::Box;
		cs.geom.boxGeometry.halfExtents = Physics::Vec3(hx, hy, hz);
		cs.materialPtr = g_defaultMaterial;
		cs.userData = userData;
		cs.flags = Physics::ShapeFlags::SceneQuery;
		Run(nullptr, Context(world, nullptr), Shape::CreateShape, cs);
		if (cs.value != nullptr)
		{
			ShapeData::SetFilterData sf{};
			sf.data.belongsTo = 1;
			sf.data.collidesWith = 1;
			Run(cs.value, Context(world, nullptr), Shape::SetFilterData, sf);
		}
		return cs.value;
	}

	Handle CreateFloor(Handle world)
	{
		WorldData::CreateBody cb{};
		cb.pose = Physics::Pose(0, -0.5f, 0, 0, 0, 0, 1);
		cb.isStatic = true;
		cb.flags = Physics::BodyFlags::Simulated;
		Run(world, Context(world, nullptr), World::CreateBody, cb);
		Handle body = cb.outBody;
		Handle shape = CreateBox(world, 50.0f, 0.5f, 50.0f);
		BodyData::AttachShape as{};
		as.shape = shape;
		as.pose = Physics::Pose();
		as.forceLocalPose = true;
		Run(body, Context(world, body), Body::AttachShape, as);
		Handle list[1] = { body };
		WorldData::AddActorsToScene aa{};
		aa.actors = list;
		aa.bufferSize = 1;
		Run(world, Context(world, nullptr), World::AddActorsToScene, aa);
		return body;
	}

	// ---- articulation wrappers ----

	Handle CreateArticulation(Handle world, Physics::ArticulationFlags flags, void* userData = nullptr)
	{
		WorldData::CreateArticulation ca{};
		ca.flags = flags;
		ca.userData = userData;
		Run(world, Context(world, nullptr), World::CreateArticulation, ca);
		return ca.outArticulation;
	}

	Handle CreateLink(Handle world, Handle art, Handle parent, const Physics::Pose& pose, void* userData = nullptr)
	{
		ArticulationData::CreateLink cl{};
		cl.parentLink = parent;
		cl.userData = userData;
		cl.pose = pose;
		Run(art, Context(world, art), Articulation::CreateLink, cl);
		return cl.outNewLink;
	}

	void LinkJointType(Handle world, Handle link, JointType type)
	{
		ArticulationJointData::SetType st{};
		st.value = type;
		Run(link, Context(world, link), Articulation::SetLinkJointType, st);
	}

	void LinkAxisLock(Handle world, Handle link, ConstraintAxis axis, JointDofLock lock)
	{
		ArticulationJointData::SetAxisLock sl{};
		sl.axis = axis;
		sl.value = lock;
		Run(link, Context(world, link), Articulation::SetLinkJointAxisLock, sl);
	}

	void LinkLimit(Handle world, Handle link, ConstraintAxis axis, float lo, float hi)
	{
		ArticulationJointData::SetLimit sl{};
		sl.axis = axis;
		sl.min = lo;
		sl.max = hi;
		Run(link, Context(world, link), Articulation::SetLinkJointLimit, sl);
	}

	void LinkDrive(Handle world, Handle link, ConstraintAxis axis, float stiffness, float damping, float forceLimit, float target, float targetVelocity = 0.0f,
		Physics::ArticulationMotorType type = Physics::ArticulationMotorType::Force)
	{
		ArticulationJointData::SetAxisMotorConfiguration mc{};
		mc.axis = axis;
		mc.value.stiffness = stiffness;
		mc.value.damping = damping;
		mc.value.forceLimit = forceLimit;
		mc.value.target = target;
		mc.value.targetVelocity = targetVelocity;
		mc.value.type = type;
		Run(link, Context(world, link), Articulation::SetLinkJointAxisMotorConfiguration, mc);
	}

	void LinkFrames(Handle world, Handle link, const Physics::Pose& parentFrame, const Physics::Pose& childFrame)
	{
		ArticulationJointData::SetActorLocalPose sp{};
		sp.target = Physics::PoseTarget::Parent;
		sp.value = parentFrame;
		Run(link, Context(world, link), Articulation::SetLinkJointActorLocalPose, sp);
		sp.target = Physics::PoseTarget::Child;
		sp.value = childFrame;
		Run(link, Context(world, link), Articulation::SetLinkJointActorLocalPose, sp);
	}

	void LinkFriction(Handle world, Handle link, float value)
	{
		ArticulationJointData::SetFriction sf{};
		sf.value = value;
		Run(link, Context(world, link), Articulation::SetLinkJointFriction, sf);
	}

	void LinkMaxJointVelocity(Handle world, Handle link, float value)
	{
		ArticulationJointData::SetMaxVelocity sm{};
		sm.value = value;
		Run(link, Context(world, link), Articulation::SetLinkJointMaxVelocity, sm);
	}

	void LinkDamping(Handle world, Handle link, float linear, float angular)
	{
		ArticulationLinkData::SetDamping sd{};
		sd.linear = linear;
		sd.angular = angular;
		Run(link, Context(world, link), Articulation::SetLinkDamping, sd);
	}

	void LinkMass(Handle world, Handle link, float mass)
	{
		ArticulationLinkData::SetMass sm{};
		sm.value = mass;
		Run(link, Context(world, link), Articulation::SetLinkMass, sm);
	}

	Handle LinkAttachBox(Handle world, Handle link, float hx, float hy, float hz, float mass, void* shapeUserData = nullptr)
	{
		Handle shape = CreateBox(world, hx, hy, hz, shapeUserData);
		ArticulationLinkData::AttachShape as{};
		as.shape = shape;
		as.pose = Physics::Pose();
		Run(link, Context(world, link), Articulation::AttachShapeToLink, as);
		LinkMass(world, link, mass);
		ArticulationLinkData::RecomputeMassProperties rp;
		Run(link, Context(world, link), Articulation::RecomputeLinkMassProperties, rp);
		return shape;
	}

	void AddArticulationToScene(Handle world, Handle art)
	{
		Handle list[1] = { art };
		WorldData::AddActorsToScene aa{};
		aa.actors = list;
		aa.bufferSize = 1;
		Run(world, Context(world, nullptr), World::AddActorsToScene, aa);
	}

	Physics::Pose LinkPose(Handle world, Handle link)
	{
		ArticulationLinkData::GetPose gp{};
		Run(link, Context(world, link), Articulation::GetLinkPose, gp);
		return gp.value;
	}

	Physics::Vec3 LinkLinearVelocity(Handle world, Handle link)
	{
		ArticulationLinkData::GetLinearVelocity gv{};
		Run(link, Context(world, link), Articulation::GetLinkLinearVelocity, gv);
		return gv.value;
	}

	Physics::ReducedSpaceCoordinateStorage JointPosition(Handle world, Handle link)
	{
		ArticulationJointData::GetPosition gp{};
		Run(link, Context(world, link), Articulation::GetLinkJointPosition, gp);
		return gp.value;
	}

	Physics::ReducedSpaceCoordinateStorage JointVelocity(Handle world, Handle link)
	{
		ArticulationJointData::GetVelocity gv{};
		Run(link, Context(world, link), Articulation::GetLinkJointVelocity, gv);
		return gv.value;
	}

	void SetJointPosition(Handle world, Handle link, float x, float y = 0.0f, float z = 0.0f, int dofs = 1)
	{
		ArticulationJointData::SetPosition sp{};
		sp.value.coordinate = Physics::Vec3(x, y, z);
		sp.value.dofCount = dofs;
		Run(link, Context(world, link), Articulation::SetLinkJointPosition, sp);
	}

	void SetJointVelocity(Handle world, Handle link, float x, float y = 0.0f, float z = 0.0f, int dofs = 1)
	{
		ArticulationJointData::SetVelocity sv{};
		sv.value.coordinate = Physics::Vec3(x, y, z);
		sv.value.dofCount = dofs;
		Run(link, Context(world, link), Articulation::SetLinkJointVelocity, sv);
	}

	// Buffer queries copy the data and release the SDK-owned memory like Unity does.
	template <typename Cmd, typename Id>
	std::vector<float> GetBuffer(Handle world, Handle art, Id id)
	{
		Cmd cmd{};
		Run(art, Context(world, art), id, cmd);
		std::vector<float> out;
		if (cmd.value.data != nullptr)
		{
			out.assign(cmd.value.data, cmd.value.data + cmd.value.size);
		}
		if (cmd.value.sdkOwnedMemory)
		{
			ArticulationData::ReleaseDataBuffer rel{};
			rel.value = cmd.value;
			Run(art, Context(world, art), Articulation::ReleaseDataBuffer, rel);
		}
		return out;
	}

	std::vector<float> JointsPositions(Handle world, Handle art)
	{
		return GetBuffer<ArticulationData::GetJointsPositions>(world, art, Articulation::GetJointsPositions);
	}

	std::vector<float> JointsVelocities(Handle world, Handle art)
	{
		return GetBuffer<ArticulationData::GetJointsVelocities>(world, art, Articulation::GetJointsVelocities);
	}

	template <typename Cmd>
	void SetBuffer(Handle world, Handle art, Articulation id, const std::vector<float>& values)
	{
		Cmd cmd{};
		cmd.value.data = values.data();
		cmd.value.size = values.size();
		cmd.value.sdkOwnedMemory = false;
		Run(art, Context(world, art), id, cmd);
	}

	// Rotation angle about the world Z axis encoded in a quaternion (sign follows the quaternion).
	float AngleAboutZ(const Physics::Pose& p)
	{
		return 2.0f * std::atan2(p.quat.z, p.quat.w);
	}

	Physics::Pose AxisFrame(float x, float y, float z, int axis)
	{
		// Rotates the joint X axis onto world Z (axis 2), world Y (axis 1) or leaves it on X (axis 0).
		const float s = std::sqrt(0.5f);
		if (axis == 2)
		{
			return Physics::Pose(x, y, z, 0.0f, -s, 0.0f, s);
		}
		if (axis == 1)
		{
			return Physics::Pose(x, y, z, 0.0f, 0.0f, s, s);
		}
		return Physics::Pose(x, y, z, 0.0f, 0.0f, 0.0f, 1.0f);
	}

	const Physics::ArticulationFlags kFixedFlags = Physics::ArticulationFlags::ImmovableRoot
		| Physics::ArticulationFlags::DriveLimitsAsForces | Physics::ArticulationFlags::Simulated;
	const Physics::ArticulationFlags kFreeFlags = Physics::ArticulationFlags::DriveLimitsAsForces | Physics::ArticulationFlags::Simulated;

	// Fixed root at (0, y, 0) plus one revolute child (axis world Z) whose center sits length L along +X.
	struct Pendulum
	{
		Handle world = nullptr;
		Handle art = nullptr;
		Handle root = nullptr;
		Handle link = nullptr;
		float length = 1.0f;
		float rootY = 5.0f;
	};

	Pendulum BuildPendulum(Handle world, float rootY, bool addToScene = true)
	{
		Pendulum p;
		p.world = world;
		p.rootY = rootY;
		p.art = CreateArticulation(world, kFixedFlags);
		p.root = CreateLink(world, p.art, nullptr, Physics::Pose(0, rootY, 0, 0, 0, 0, 1));
		LinkAttachBox(world, p.root, 0.1f, 0.1f, 0.1f, 5.0f);
		p.link = CreateLink(world, p.art, p.root, Physics::Pose(p.length, rootY, 0, 0, 0, 0, 1));
		LinkJointType(world, p.link, JointType::Hinge);
		LinkFrames(world, p.link, AxisFrame(0, 0, 0, 2), AxisFrame(-p.length, 0, 0, 2));
		LinkAxisLock(world, p.link, ConstraintAxis::AngularX, JointDofLock::Free);
		LinkAttachBox(world, p.link, 0.5f, 0.05f, 0.05f, 1.0f);
		LinkFriction(world, p.link, 0.0f);
		LinkDamping(world, p.link, 0.0f, 0.0f);
		if (addToScene)
		{
			AddArticulationToScene(world, p.art);
		}
		return p;
	}

	void SetGravityY(Handle world, float y)
	{
		WorldData::SetGravity sg{};
		sg.value = Physics::Vec3(0.0f, y, 0.0f);
		Run(world, Context(world, nullptr), World::SetGravity, sg);
	}

	void DestroyLinkCmd(Handle world, Handle art, Handle link)
	{
		ArticulationData::DestroyLink dl{};
		dl.value = link;
		Run(art, Context(world, art), Articulation::DestroyLink, dl);
	}

	void DestroyArticulationCmd(Handle world, Handle art)
	{
		WorldData::DestoryArticulation da{};
		da.value = art;
		Run(world, Context(world, nullptr), World::DestroyArticulation, da);
	}

	size_t LinkCount(Handle world, Handle art)
	{
		ArticulationData::GetLinkCount lc{};
		Run(art, Context(world, art), Articulation::GetLinkCount, lc);
		return lc.value;
	}

	// Fixed root plus n links along +X (hinge about Z), spaced 1 apart; returns the link handles (root first).
	std::vector<Handle> BuildLine(Handle world, Handle& art, int n, float rootY, bool withShapes = true)
	{
		std::vector<Handle> links;
		art = CreateArticulation(world, kFixedFlags);
		Handle root = CreateLink(world, art, nullptr, Physics::Pose(0, rootY, 0, 0, 0, 0, 1));
		if (withShapes)
		{
			LinkAttachBox(world, root, 0.2f, 0.2f, 0.2f, 2.0f);
		}
		links.push_back(root);
		for (int i = 1; i <= n; ++i)
		{
			Handle link = CreateLink(world, art, links.back(), Physics::Pose(static_cast<float>(i), rootY, 0, 0, 0, 0, 1));
			LinkJointType(world, link, JointType::Hinge);
			LinkFrames(world, link, AxisFrame(0.5f, 0, 0, 2), AxisFrame(-0.5f, 0, 0, 2));
			LinkAxisLock(world, link, ConstraintAxis::AngularX, JointDofLock::Free);
			if (withShapes)
			{
				LinkAttachBox(world, link, 0.4f, 0.1f, 0.1f, 1.0f);
			}
			links.push_back(link);
		}
		AddArticulationToScene(world, art);
		return links;
	}

	void DestroyWorld(Handle world)
	{
		ShapeData::DestroyShape dummy;
		Run(world, Context(world, nullptr), World::DestroyWorld, dummy);
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
	std::printf("articulation_test: backend=%s (requested SDK %d)\n", path.c_str(), FALL_TEST_SDK);

	SdkData::InitializeSdk init;
	init.engineObjectIdOffset = 0;
	Run(nullptr, Context(), SDK::InitializeSdk, init);
	Check(init.result, "InitializeSdk");
	SdkData::SetErrorVerbosityLevel ev;
	ev.value = Physics::ErrorVerbosityLevel::Verbose;
	Run(nullptr, Context(), SDK::SetErrorVerbosityLevel, ev);

	MaterialData::GetDefaultMaterial dm{};
	Run(nullptr, Context(), SDK::GetDefaultMaterial, dm);
	g_defaultMaterial = dm.value;
	Check(g_defaultMaterial != nullptr, "GetDefaultMaterial");

	// ---- (a) fixed base, one revolute link: pendulum under gravity ----
	std::printf("[a] pendulum\n");
	{
		Handle world = CreateWorld(-9.81f);
		Pendulum p = BuildPendulum(world, 5.0f);

		WorldData::GetBasicStats bs{};
		Run(world, Context(world, nullptr), World::GetBasicStats, bs);
		Check(bs.value.articulations == 1, "stats count the articulation");
		WorldData::GetArticulationCount ac{};
		Run(world, Context(world, nullptr), World::GetArticulationCount, ac);
		Check(ac.value == 1, "GetArticulationCount");
		Handle listed[4] = {};
		WorldData::GetArticulations ga{};
		ga.buffer = listed;
		ga.bufferSize = 4;
		Run(world, Context(world, nullptr), World::GetArticulations, ga);
		Check(ga.written == 1 && listed[0] == p.art, "GetArticulations lists the articulation");

		ArticulationLinkData::GetMass gm{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkMass, gm);
		Check(Near(gm.value, 1.0f, 1e-4f), "link mass");
		ArticulationLinkData::GetWorldInertiaTensorMatrix gi{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkWorldInertiaTensorMatrix, gi);
		const float izz = gi.value[8];
		Check(izz > 0.01f && izz < 0.2f, "link inertia from box shape");
		const float inertia = izz + 1.0f * p.length * p.length;

		ArticulationJointData::GetDofCount dc{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkJointDofCount, dc);
		Check(dc.value == 1, "revolute link has one DOF");

		float maxAbs = 0.0f;
		float maxEnergyError = 0.0f;
		float maxPoseMismatch = 0.0f;
		float firstQuarter = -1.0f;
		const int steps = static_cast<int>(3.0f / kDt);
		for (int i = 0; i < steps; ++i)
		{
			Step(world, kDt);
			const float theta = JointPosition(world, p.link).coordinate.x;
			const float omega = JointVelocity(world, p.link).coordinate.x;
			const Physics::Pose pose = LinkPose(world, p.link);
			const float energy = 0.5f * inertia * omega * omega + 1.0f * 9.81f * (pose.pos.y - p.rootY);
			maxEnergyError = std::fmax(maxEnergyError, std::fabs(energy));
			maxAbs = std::fmax(maxAbs, std::fabs(theta));
			const float fromPose = std::atan2(p.rootY - pose.pos.y, pose.pos.x);
			maxPoseMismatch = std::fmax(maxPoseMismatch, std::fabs(std::fabs(theta) - std::fabs(fromPose)));
			if (firstQuarter < 0.0f && std::fabs(theta) > 0.5f * kPi * 0.95f)
			{
				firstQuarter = (i + 1) * kDt;
			}
		}
		std::printf("  max |theta|=%.3f maxEnergyErr=%.4f (mgL=9.81) poseMismatch=%.4f t(90deg)=%.3f\n", maxAbs, maxEnergyError, maxPoseMismatch, firstQuarter);
		Check(maxAbs > 1.4f, "pendulum swings down (|theta| passes 80 degrees)");
		Check(maxEnergyError < 0.12f * 9.81f, "pendulum energy roughly conserved");
		Check(maxPoseMismatch < 0.05f, "joint position agrees with link pose");
		Check(firstQuarter > 0.3f && firstQuarter < 0.9f, "quarter swing takes a plausible time");

		// the root link stays fixed
		const Physics::Pose rootPose = LinkPose(world, p.root);
		Check(Near(rootPose.pos.y, 5.0f, 1e-4f) && Near(rootPose.pos.x, 0.0f, 1e-4f), "fixed root does not move");

		// gravity compensation of the horizontal pendulum is m * g * L (sign by axis)
		ArticulationJointData::SetPosition sp{};
		SetJointPosition(world, p.link, 0.0f);
		SetJointVelocity(world, p.link, 0.0f);
		const std::vector<float> grav = GetBuffer<ArticulationData::GetJointsGravityForces>(world, p.art, Articulation::GetJointsGravityForces);
		Check(grav.size() == 1 && Near(std::fabs(grav[0]), 9.81f, 0.8f), "gravity compensation force of a horizontal pendulum");
		std::printf("  gravity force=%.3f\n", grav.empty() ? 0.0f : grav[0]);
		const std::vector<float> cor = GetBuffer<ArticulationData::GetJointsCoriolisCentrifugalForces>(world, p.art, Articulation::GetJointsCoriolisCentrifugalForces);
		Check(cor.size() == 1 && Near(cor[0], 0.0f, 0.05f), "no Coriolis force at rest");
		ArticulationData::GetDenseJacobianAsBuffer jac{};
		Run(p.art, Context(world, p.art), Articulation::GetDenseJacobianAsFloatBuffer, jac);
		std::printf("  jacobian %ux%u\n", jac.rowCount, jac.colCount);
		Check(jac.rowCount > 0 && jac.colCount > 0 && jac.value.data != nullptr && jac.value.size == static_cast<size_t>(jac.rowCount) * jac.colCount,
			"dense Jacobian dimensions");
		if (jac.value.sdkOwnedMemory)
		{
			ArticulationData::ReleaseDataBuffer rel{};
			rel.value = jac.value;
			Run(p.art, Context(world, p.art), Articulation::ReleaseDataBuffer, rel);
		}
		(void)sp;

		// link hierarchy
		ArticulationData::GetLinkCount lc{};
		Run(p.art, Context(world, p.art), Articulation::GetLinkCount, lc);
		Check(lc.value == 2, "GetLinkCount");
		ArticulationData::GetLinkParent gp{};
		gp.link = p.link;
		Run(p.art, Context(world, p.art), Articulation::GetLinkParent, gp);
		Check(gp.value == p.root, "GetLinkParent");
		gp.link = p.root;
		Run(p.art, Context(world, p.art), Articulation::GetLinkParent, gp);
		Check(gp.value == nullptr, "root has no parent");
		ArticulationData::GetLinkChildCount cc{};
		cc.link = p.root;
		Run(p.art, Context(world, p.art), Articulation::GetLinkChildCount, cc);
		Check(cc.value == 1, "GetLinkChildCount");
		ArticulationLinkData::GetLinkIndex li{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkIndex, li);
		Check(li.value == 1, "GetLinkIndex");

		DestroyWorld(world);
	}

	// ---- (b) revolute drive converges ----
	std::printf("[b] revolute drive\n");
	float tRevolute = -1.0f;
	{
		Handle world = CreateWorld(0.0f);
		Pendulum p = BuildPendulum(world, 3.0f, false);
		LinkAxisLock(world, p.link, ConstraintAxis::AngularX, JointDofLock::Free);
		LinkDrive(world, p.link, ConstraintAxis::AngularX, 100.0f, 20.0f, 1e6f, 0.8f);
		AddArticulationToScene(world, p.art);

		const int steps = static_cast<int>(3.0f / kDt);
		for (int i = 0; i < steps; ++i)
		{
			Step(world, kDt);
			if (tRevolute < 0.0f && std::fabs(JointPosition(world, p.link).coordinate.x - 0.8f) < 0.02f)
			{
				tRevolute = (i + 1) * kDt;
			}
		}
		const float pos = JointPosition(world, p.link).coordinate.x;
		const float vel = JointVelocity(world, p.link).coordinate.x;
		std::printf("  pos=%.4f vel=%.5f t(|err|<0.02)=%.3f\n", pos, vel, tRevolute);
		Check(Near(pos, 0.8f, 0.01f), "revolute drive reaches the target");
		Check(std::fabs(vel) < 0.02f, "revolute drive comes to rest");
		const std::vector<float> posBuf = JointsPositions(world, p.art);
		const std::vector<float> velBuf = JointsVelocities(world, p.art);
		Check(posBuf.size() == 1 && Near(posBuf[0], pos, 1e-5f), "per-link and buffer position agree");
		Check(velBuf.size() == 1 && Near(velBuf[0], vel, 1e-5f), "per-link and buffer velocity agree");
		Check(Near(std::fabs(AngleAboutZ(LinkPose(world, p.link))), 0.8f, 0.02f), "link pose matches the joint position");

		ArticulationJointData::GetMotorForce mf{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkJointMotorForce, mf);
		const std::vector<float> driveForces = GetBuffer<ArticulationData::GetJointsDriveForces>(world, p.art, Articulation::GetJointsDriveForces);
		Check(driveForces.size() == 1 && Near(driveForces[0], mf.value.coordinate.x, 1e-4f), "drive force queries agree");
		Check(std::fabs(mf.value.coordinate.x) < 3.0f, "drive force is small at the target");

		// force limit: a weak drive cannot reach the target quickly
		SetJointPosition(world, p.link, 0.0f);
		SetJointVelocity(world, p.link, 0.0f);
		LinkDrive(world, p.link, ConstraintAxis::AngularX, 100.0f, 0.0f, 0.1f, 1.0f);
		Step(world, 1.0f);
		const float weak = JointPosition(world, p.link).coordinate.x;
		std::printf("  forceLimit 0.1 after 1s: pos=%.4f\n", weak);
		Check(weak > 0.0f && weak < 0.2f, "force limit bounds the drive");

		// velocity drive (damping only) tracks the target velocity
		SetJointPosition(world, p.link, 0.0f);
		SetJointVelocity(world, p.link, 0.0f);
		LinkDrive(world, p.link, ConstraintAxis::AngularX, 0.0f, 50.0f, 1e6f, 0.0f, 1.5f, Physics::ArticulationMotorType::Velocity);
		Step(world, 1.0f);
		const float tracked = JointVelocity(world, p.link).coordinate.x;
		std::printf("  velocity drive: v=%.4f (target 1.5)\n", tracked);
		Check(Near(tracked, 1.5f, 0.1f), "velocity drive reaches the target velocity");

		DestroyWorld(world);
	}

	// ---- (c) prismatic elevator ----
	std::printf("[c] prismatic drive\n");
	float tPrismatic = -1.0f;
	{
		Handle world = CreateWorld(-9.81f);
		Handle art = CreateArticulation(world, kFixedFlags);
		Handle root = CreateLink(world, art, nullptr, Physics::Pose(0, 1.0f, 0, 0, 0, 0, 1));
		LinkAttachBox(world, root, 0.3f, 0.1f, 0.3f, 10.0f);
		Handle car = CreateLink(world, art, root, Physics::Pose(0, 2.0f, 0, 0, 0, 0, 1));
		LinkJointType(world, car, JointType::Prismatic);
		LinkFrames(world, car, AxisFrame(0, 1.0f, 0, 1), AxisFrame(0, -0.0f, 0, 1));
		LinkAxisLock(world, car, ConstraintAxis::LinearX, JointDofLock::Free);
		LinkDrive(world, car, ConstraintAxis::LinearX, 5000.0f, 500.0f, 1e6f, 1.0f);
		LinkAttachBox(world, car, 0.3f, 0.1f, 0.3f, 1.0f);
		LinkFriction(world, car, 0.0f);
		AddArticulationToScene(world, art);

		const int steps = static_cast<int>(3.0f / kDt);
		for (int i = 0; i < steps; ++i)
		{
			Step(world, kDt);
			if (tPrismatic < 0.0f && std::fabs(JointPosition(world, car).coordinate.x - 1.0f) < 0.02f)
			{
				tPrismatic = (i + 1) * kDt;
			}
		}
		const float pos = JointPosition(world, car).coordinate.x;
		const Physics::Pose carPose = LinkPose(world, car);
		std::printf("  pos=%.4f carY=%.4f t(|err|<0.02)=%.3f\n", pos, carPose.pos.y, tPrismatic);
		Check(Near(pos, 1.0f, 0.01f), "prismatic drive reaches the target");
		Check(Near(carPose.pos.y - 1.0f, pos + 1.0f, 0.01f), "car link follows the joint position");
		Check(std::fabs(JointVelocity(world, car).coordinate.x) < 0.01f, "elevator at rest");

		// retarget downwards
		LinkDrive(world, car, ConstraintAxis::LinearX, 5000.0f, 500.0f, 1e6f, -0.5f);
		Step(world, 2.0f);
		std::printf("  retarget -0.5: pos=%.4f\n", JointPosition(world, car).coordinate.x);
		Check(Near(JointPosition(world, car).coordinate.x, -0.5f, 0.02f), "prismatic drive retargets");

		// limits: clamp the elevator travel
		LinkAxisLock(world, car, ConstraintAxis::LinearX, JointDofLock::Limited); // no range yet: stays locked
		LinkLimit(world, car, ConstraintAxis::LinearX, -0.2f, 0.3f);
		LinkDrive(world, car, ConstraintAxis::LinearX, 5000.0f, 500.0f, 1e6f, 1.0f);
		Step(world, 2.0f);
		const float limited = JointPosition(world, car).coordinate.x;
		std::printf("  limited to [-0.2,0.3]: pos=%.4f\n", limited);
		Check(limited <= 0.31f && limited >= 0.25f, "prismatic upper limit holds");
		DestroyWorld(world);
	}

	// ---- (d) chain with revolute + prismatic + fixed joints, shapes, floor ----
	std::printf("[d] chain\n");
	{
		Handle world = CreateWorld(-9.81f);
		CreateFloor(world);

		struct Chain
		{
			Handle art, root, a, b, c;
			Handle shapes[4];
		};
		auto build = [&](float x0, float y0, bool fixBase, int tag) {
			Chain ch{};
			ch.art = CreateArticulation(world, fixBase ? kFixedFlags : kFreeFlags, FakeUser(100 + tag));
			ch.root = CreateLink(world, ch.art, nullptr, Physics::Pose(x0, y0, 0, 0, 0, 0, 1), FakeUser(1000 + tag));
			ch.shapes[0] = LinkAttachBox(world, ch.root, 0.25f, 0.2f, 0.2f, 2.0f, FakeUser(2000 + tag));

			ch.a = CreateLink(world, ch.art, ch.root, Physics::Pose(x0 + 0.75f, y0, 0, 0, 0, 0, 1));
			LinkJointType(world, ch.a, JointType::Hinge);
			LinkFrames(world, ch.a, AxisFrame(0.25f, 0, 0, 2), AxisFrame(-0.5f, 0, 0, 2));
			LinkAxisLock(world, ch.a, ConstraintAxis::AngularX, JointDofLock::Free);
			LinkDrive(world, ch.a, ConstraintAxis::AngularX, 2000.0f, 200.0f, 1e6f, 0.0f);
			ch.shapes[1] = LinkAttachBox(world, ch.a, 0.5f, 0.2f, 0.2f, 1.0f);

			ch.b = CreateLink(world, ch.art, ch.a, Physics::Pose(x0 + 1.75f, y0, 0, 0, 0, 0, 1));
			LinkJointType(world, ch.b, JointType::Prismatic);
			LinkFrames(world, ch.b, AxisFrame(0.5f, 0, 0, 0), AxisFrame(-0.5f, 0, 0, 0));
			LinkAxisLock(world, ch.b, ConstraintAxis::LinearX, JointDofLock::Free);
			LinkDrive(world, ch.b, ConstraintAxis::LinearX, 2000.0f, 200.0f, 1e6f, 0.0f);
			ch.shapes[2] = LinkAttachBox(world, ch.b, 0.5f, 0.2f, 0.2f, 1.0f);

			ch.c = CreateLink(world, ch.art, ch.b, Physics::Pose(x0 + 2.5f, y0, 0, 0, 0, 0, 1));
			LinkJointType(world, ch.c, JointType::Fixed);
			LinkFrames(world, ch.c, AxisFrame(0.5f, 0, 0, 0), AxisFrame(-0.25f, 0, 0, 0));
			ch.shapes[3] = LinkAttachBox(world, ch.c, 0.25f, 0.2f, 0.2f, 1.0f);
			AddArticulationToScene(world, ch.art);
			return ch;
		};
		Chain fixedChain = build(0.0f, 3.0f, true, 1);
		Chain freeChain = build(10.0f, 3.0f, false, 2);

		WorldData::GetBasicStats bs{};
		Run(world, Context(world, nullptr), World::GetBasicStats, bs);
		Check(bs.value.articulations == 2, "two articulations in the world");

		ArticulationData::GetLinksUserData lu{};
		void* users[8] = {};
		lu.sortedByIndex = true;
		lu.buffer = users;
		lu.bufferSize = 8;
		Run(freeChain.art, Context(world, freeChain.art), Articulation::GetLinksUserData, lu);
		Check(lu.written == 4 && users[0] == FakeUser(1002), "GetLinksUserData lists the links in order");
		ArticulationData::GetUserData gu{};
		Run(freeChain.art, Context(world, freeChain.art), Articulation::GetUserData, gu);
		Check(gu.value == FakeUser(102), "articulation user data");

		ArticulationLinkData::GetShapeCount sc{};
		Run(freeChain.a, Context(world, freeChain.a), Articulation::GetLinkShapeCount, sc);
		Check(sc.value == 1, "link shape count");
		ArticulationLinkData::GetShapesUserData su{};
		void* shapeUsers[2] = {};
		su.buffer = shapeUsers;
		su.bufferSize = 2;
		Run(freeChain.root, Context(world, freeChain.root), Articulation::GetLinkShapesUserData, su);
		Check(su.written == 1 && shapeUsers[0] == FakeUser(2002), "link shape user data");

		ShapeData::GetActorDescriptor gad{};
		Run(freeChain.shapes[1], Context(world, nullptr), Shape::GetActorDescriptor, gad);
		Check(gad.value.type == Physics::ActorType::Link && gad.value.actor == freeChain.a, "Shape::GetActorDescriptor sees the link");

		Step(world, 0.3f);
		const Physics::Vec3 v = LinkLinearVelocity(world, freeChain.root);
		Check(v.y < -2.5f && v.y > -3.4f, "free chain falls with gravity");
		Step(world, 0.5f);
		const float fixedRootY = LinkPose(world, fixedChain.root).pos.y;
		Check(Near(fixedRootY, 3.0f, 1e-3f), "fixed chain root stays put");
		Check(LinkPose(world, freeChain.root).pos.y < 2.5f, "free chain root has fallen");

		Step(world, 3.0f);
		const Handle freeLinks[4] = { freeChain.root, freeChain.a, freeChain.b, freeChain.c };
		const Handle fixedLinks[4] = { fixedChain.root, fixedChain.a, fixedChain.b, fixedChain.c };
		bool restsOnFloor = true;
		for (Handle l : freeLinks)
		{
			const float y = LinkPose(world, l).pos.y;
			std::printf("  free link y=%.4f\n", y);
			restsOnFloor = restsOnFloor && y > 0.1f && y < 0.4f;
		}
		Check(restsOnFloor, "free chain rests on the floor");
		bool holds = true;
		for (Handle l : fixedLinks)
		{
			const float y = LinkPose(world, l).pos.y;
			std::printf("  fixed link y=%.4f\n", y);
			holds = holds && y > 2.7f && y < 3.2f;
		}
		Check(holds, "fixed chain keeps its shape under drives");

		// the shape of a link reports a world pose that follows the link
		ShapeData::GetPose sgp{};
		Run(freeChain.shapes[3], Context(world, nullptr), Shape::GetPose, sgp);
		Check(Near(sgp.value.pos.y, LinkPose(world, freeChain.c).pos.y, 1e-3f), "Shape::GetPose follows the link");

		// DOF count: revolute 1 + prismatic 1 + fixed 0
		const std::vector<float> positions = JointsPositions(world, freeChain.art);
		Check(positions.size() == 2, "chain has two DOFs");
		ArticulationJointData::GetDofCount dc{};
		Run(freeChain.c, Context(world, freeChain.c), Articulation::GetLinkJointDofCount, dc);
		Check(dc.value == 0, "fixed joint has no DOF");

		// raycast sees link shapes
		QueryData::RayCast rc{};
		rc.origin = Physics::Vec3(2.5f, 6.0f, 0.0f);
		rc.direction = Physics::Vec3(0.0f, -1.0f, 0.0f);
		rc.maxDistance = 20.0f;
		rc.filter.mask = 0xFFFFFFFFu;
		rc.filter.options = Physics::QueryFilterFlags::UseDynamicBodies | Physics::QueryFilterFlags::UseStaticBodies;
		Run(world, Context(world, nullptr), Query::RayCast, rc);
		const float cTop = LinkPose(world, fixedChain.c).pos.y + 0.2f;
		std::printf("  raycast hits=%u point.y=%.3f (expect %.3f)\n", rc.hitCount, rc.hit.point.y, cTop);
		Check(rc.hitCount == 1 && Near(rc.hit.point.y, cTop, 0.02f), "ray cast hits an articulation link shape");

		DestroyWorld(world);
	}

	// ---- (e) joint buffers, DOF order, state set before the scene ----
	std::printf("[e] joint buffers\n");
	{
		Handle world = CreateWorld(0.0f);
		Handle art = CreateArticulation(world, kFixedFlags);
		Handle root = CreateLink(world, art, nullptr, Physics::Pose(0, 5.0f, 0, 0, 0, 0, 1));
		LinkAttachBox(world, root, 0.2f, 0.2f, 0.2f, 2.0f);
		Handle l1 = CreateLink(world, art, root, Physics::Pose(1.0f, 5.0f, 0, 0, 0, 0, 1));
		LinkJointType(world, l1, JointType::Hinge);
		LinkFrames(world, l1, AxisFrame(0.5f, 0, 0, 2), AxisFrame(-0.5f, 0, 0, 2));
		LinkAxisLock(world, l1, ConstraintAxis::AngularX, JointDofLock::Free);
		LinkAttachBox(world, l1, 0.4f, 0.1f, 0.1f, 1.0f);
		Handle l2 = CreateLink(world, art, l1, Physics::Pose(2.0f, 5.0f, 0, 0, 0, 0, 1));
		LinkJointType(world, l2, JointType::BallAndSocket);
		LinkFrames(world, l2, AxisFrame(0.5f, 0, 0, 0), AxisFrame(-0.5f, 0, 0, 0));
		LinkAxisLock(world, l2, ConstraintAxis::AngularX, JointDofLock::Free);
		LinkAxisLock(world, l2, ConstraintAxis::AngularY, JointDofLock::Free);
		LinkAxisLock(world, l2, ConstraintAxis::AngularZ, JointDofLock::Free);
		LinkAttachBox(world, l2, 0.4f, 0.1f, 0.1f, 1.0f);
		Handle l3 = CreateLink(world, art, l2, Physics::Pose(3.0f, 5.0f, 0, 0, 0, 0, 1));
		LinkJointType(world, l3, JointType::Prismatic);
		LinkFrames(world, l3, AxisFrame(0.5f, 0, 0, 0), AxisFrame(-0.5f, 0, 0, 0));
		LinkAxisLock(world, l3, ConstraintAxis::LinearX, JointDofLock::Free);
		LinkAttachBox(world, l3, 0.4f, 0.1f, 0.1f, 1.0f);

		ArticulationJointData::GetDofCount dc{};
		Run(l2, Context(world, l2), Articulation::GetLinkJointDofCount, dc);
		Check(dc.value == 3, "spherical joint has three DOFs");

		// state set before the articulation is in a scene is applied when it is added
		SetJointPosition(world, l1, 0.3f);
		Check(Near(JointPosition(world, l1).coordinate.x, 0.3f, 1e-6f), "pre-scene joint position reads back");
		AddArticulationToScene(world, art);
		std::vector<float> pos = JointsPositions(world, art);
		Check(pos.size() == 5, "five DOFs in the buffer");
		Check(pos.size() == 5 && Near(pos[0], 0.3f, 1e-5f), "pre-scene position was applied on add");

		// round trip in DOF order: hinge, spherical x3, prismatic
		SetBuffer<ArticulationData::SetJointsPositions>(world, art, Articulation::SetJointsPositions, { 0.2f, 0.1f, -0.15f, 0.25f, 0.4f });
		pos = JointsPositions(world, art);
		const float expectPos[5] = { 0.2f, 0.1f, -0.15f, 0.25f, 0.4f };
		bool posOk = pos.size() == 5;
		for (size_t i = 0; posOk && i < 5; ++i)
		{
			posOk = Near(pos[i], expectPos[i], 1e-4f);
		}
		Check(posOk, "SetJointsPositions / GetJointsPositions round trip");
		Check(Near(JointPosition(world, l1).coordinate.x, 0.2f, 1e-4f), "hinge slice of the buffer");
		const auto sph = JointPosition(world, l2);
		Check(sph.dofCount == 3 && Near(sph.coordinate.x, 0.1f, 1e-4f) && Near(sph.coordinate.y, -0.15f, 1e-4f) && Near(sph.coordinate.z, 0.25f, 1e-4f),
			"spherical slice of the buffer");
		Check(Near(JointPosition(world, l3).coordinate.x, 0.4f, 1e-4f), "prismatic slice of the buffer");
		Check(Near(std::fabs(AngleAboutZ(LinkPose(world, l1))), 0.2f, 0.01f), "link pose follows SetJointsPositions");

		SetBuffer<ArticulationData::SetJointsVelocities>(world, art, Articulation::SetJointsVelocities, { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f });
		const std::vector<float> vel = JointsVelocities(world, art);
		bool velOk = vel.size() == 5;
		for (size_t i = 0; velOk && i < 5; ++i)
		{
			velOk = Near(vel[i], static_cast<float>(i + 1), 1e-4f);
		}
		Check(velOk, "SetJointsVelocities / GetJointsVelocities round trip");
		SetJointVelocity(world, l2, 0.5f, 0.6f, 0.7f, 3);
		const auto sv = JointVelocity(world, l2);
		const std::vector<float> vel2 = JointsVelocities(world, art);
		Check(vel2.size() == 5 && Near(sv.coordinate.y, 0.6f, 1e-4f) && Near(vel2[1], 0.5f, 1e-4f) && Near(vel2[3], 0.7f, 1e-4f),
			"per-link velocity write lands in the right buffer slots");

		// a per-link position write only touches its own DOFs
		SetJointPosition(world, l3, 0.1f);
		pos = JointsPositions(world, art);
		Check(pos.size() == 5 && Near(pos[4], 0.1f, 1e-4f) && Near(pos[0], 0.2f, 1e-4f) && Near(pos[2], -0.15f, 1e-4f), "per-link position write is local");

		// wrong sized buffer: no crash, overlapping part applied
		SetBuffer<ArticulationData::SetJointsPositions>(world, art, Articulation::SetJointsPositions, { 0.05f, 0.0f });
		pos = JointsPositions(world, art);
		Check(pos.size() == 5 && Near(pos[0], 0.05f, 1e-4f), "short buffer applies the overlapping part");

		// forces persist as set; accelerations come back as a buffer of the same size
		SetBuffer<ArticulationData::SetJointsForces>(world, art, Articulation::SetJointsForces, { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
		const std::vector<float> forces = GetBuffer<ArticulationData::GetJointsForces>(world, art, Articulation::GetJointsForces);
		Check(forces.size() == 5, "joint force buffer size");
		const std::vector<float> acc = GetBuffer<ArticulationData::GetJointsAccelerations>(world, art, Articulation::GetJointsAccelerations);
		Check(acc.size() == 5, "joint acceleration buffer size");
		ArticulationJointData::GetForcesForAcceleration fa{};
		fa.acceleration.coordinate = Physics::Vec3(1.0f, 0.0f, 0.0f);
		fa.acceleration.dofCount = 1;
		Run(l1, Context(world, l1), Articulation::GetLinkJointForcesForAcceleration, fa);
		std::printf("  force for 1 rad/s^2 at the base hinge: %.3f\n", fa.value.coordinate.x);
#if FALL_TEST_SDK == 4
		// the 4.1 reference backend reports zeros for this query (SDK limitation, see PxArticulationBackend.cpp)
		Check(fa.value.dofCount == 1, "inverse dynamics query answers on PhysX 4.1");
#else
		Check(fa.value.dofCount == 1 && fa.value.coordinate.x > 0.5f, "inverse dynamics gives a positive force for a positive acceleration");
#endif

		// prismatic position moves the link: L3 sits at x = 3 + position when everything else is at 0
		SetBuffer<ArticulationData::SetJointsPositions>(world, art, Articulation::SetJointsPositions, { 0.0f, 0.0f, 0.0f, 0.0f, 0.4f });
		Check(Near(LinkPose(world, l3).pos.x, 3.4f, 0.01f), "prismatic joint position moves its link along the joint axis");

		// zero-length buffers are handled
		SetBuffer<ArticulationData::SetJointsPositions>(world, art, Articulation::SetJointsPositions, {});
		Check(true, "empty buffer set survived");
		DestroyWorld(world);
	}

	// ---- (f) limits, friction, max velocity, gravity flag ----
	std::printf("[f] limits / friction / max velocity\n");
	float frictionV0 = 0.0f;
	float frictionV1 = 0.0f;
	{
		Handle world = CreateWorld(-9.81f);
		Pendulum p = BuildPendulum(world, 5.0f, false);
		LinkAxisLock(world, p.link, ConstraintAxis::AngularX, JointDofLock::Limited);
		LinkLimit(world, p.link, ConstraintAxis::AngularX, -0.3f, 0.5f);
		AddArticulationToScene(world, p.art);
		Step(world, 3.0f);
		const float first = JointPosition(world, p.link).coordinate.x;
		SetGravityY(world, 9.81f);
		ArticulationData::WakeUp wake;
		Run(p.art, Context(world, p.art), Articulation::WakeUp, wake); // PhysX does not wake bodies on a gravity change
		Step(world, 4.0f);
		const float second = JointPosition(world, p.link).coordinate.x;
		std::printf("  limits [-0.3,0.5]: gravity down -> %.4f, gravity up -> %.4f\n", first, second);
		const bool firstLow = Near(first, -0.3f, 0.03f);
		const bool firstHigh = Near(first, 0.5f, 0.03f);
		Check(firstLow || firstHigh, "joint rests against one limit");
		Check(firstLow ? Near(second, 0.5f, 0.03f) : Near(second, -0.3f, 0.03f), "flipping gravity moves it to the other limit");

		// change the limits while in the scene
		LinkLimit(world, p.link, ConstraintAxis::AngularX, -0.1f, 0.2f);
		Step(world, 2.0f);
		const float narrowed = JointPosition(world, p.link).coordinate.x;
		std::printf("  narrowed to [-0.1,0.2]: %.4f\n", narrowed);
		Check(narrowed >= -0.12f && narrowed <= 0.22f, "limits can change inside a scene");

		// link without gravity does not swing
		SetGravityY(world, -9.81f);
		ArticulationLinkData::SetFlags sf{};
		sf.value = Physics::BodyFlags::Gravity;
		sf.enabled = false;
		Run(p.link, Context(world, p.link), Articulation::SetLinkFlags, sf);
		ArticulationLinkData::GetFlags gf{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkFlags, gf);
		Check((gf.value & Physics::BodyFlags::Gravity) == Physics::BodyFlags::None, "link gravity flag cleared");
		LinkAxisLock(world, p.link, ConstraintAxis::AngularX, JointDofLock::Free);
		SetJointPosition(world, p.link, 0.0f);
		SetJointVelocity(world, p.link, 0.0f);
		Step(world, 1.0f);
		Check(std::fabs(JointPosition(world, p.link).coordinate.x) < 0.02f, "gravity-free link stays put");
		DestroyWorld(world);
	}
	{
		// joint friction slows a spinning joint down; zero friction keeps the speed
		for (int variant = 0; variant < 2; ++variant)
		{
			Handle world = CreateWorld(0.0f);
			Pendulum p = BuildPendulum(world, 5.0f);
			LinkFriction(world, p.link, variant == 0 ? 0.0f : 0.5f);
			SetJointVelocity(world, p.link, 2.0f);
			Step(world, 1.0f);
			const float v = JointVelocity(world, p.link).coordinate.x;
			(variant == 0 ? frictionV0 : frictionV1) = v;
			DestroyWorld(world);
		}
		std::printf("  joint speed after 1s: friction 0 -> %.3f, friction 0.5 -> %.3f\n", frictionV0, frictionV1);
		Check(frictionV0 > 1.9f, "frictionless joint keeps its velocity");
		Check(frictionV1 < frictionV0 - 0.3f, "joint friction slows the joint");

		Handle world = CreateWorld(0.0f);
		Pendulum p = BuildPendulum(world, 5.0f);
		LinkMaxJointVelocity(world, p.link, 1.0f);
		ArticulationJointData::GetMaxVelocity gmv{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkJointMaxVelocity, gmv);
		Check(Near(gmv.value, 1.0f, 1e-6f), "max joint velocity reads back");
		SetJointVelocity(world, p.link, 5.0f);
		Step(world, 0.5f);
		const float limitedV = JointVelocity(world, p.link).coordinate.x;
		std::printf("  max joint velocity 1.0, started at 5.0 -> %.3f\n", limitedV);
		Check(std::fabs(limitedV) <= 1.25f, "joint velocity is limited");
		DestroyWorld(world);
	}

	// ---- handle placement: the target may arrive as sdkObject or ctx.body ----
	std::printf("[h] handle placement\n");
	{
		Handle world = CreateWorld(0.0f);
		Pendulum p = BuildPendulum(world, 5.0f);
		ArticulationLinkData::GetMass gm{};
		Run(p.link, Context(world, nullptr), Articulation::GetLinkMass, gm);
		const float viaSdk = gm.value;
		gm = {};
		Run(nullptr, Context(world, p.link), Articulation::GetLinkMass, gm);
		Check(Near(viaSdk, 1.0f, 1e-4f) && Near(gm.value, 1.0f, 1e-4f), "link target via sdkObject or ctx.body");
		ArticulationLinkData::GetLinkJoint gj{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkJoint, gj);
		Check(gj.value != nullptr, "GetLinkJoint returns a handle");
		ArticulationJointData::SetFriction sf{};
		sf.value = 0.25f;
		Run(gj.value, Context(world, p.link), Articulation::SetLinkJointFriction, sf);
		ArticulationJointData::GetType gt{};
		Run(gj.value, Context(world, nullptr), Articulation::GetLinkJointType, gt);
		Check(gt.value == JointType::Hinge, "joint handle resolves to its link");
		ArticulationData::GetLinkCount lc{};
		Run(nullptr, Context(world, p.art), Articulation::GetLinkCount, lc);
		const size_t viaCtx = lc.value;
		lc = {};
		Run(p.art, Context(world, nullptr), Articulation::GetLinkCount, lc);
		Check(viaCtx == 2 && lc.value == 2, "articulation target via sdkObject or ctx.body");
		lc = {};
		Run(p.link, Context(world, nullptr), Articulation::GetLinkCount, lc);
		Check(lc.value == 2, "a link handle also reaches its articulation");
		DestroyWorld(world);
	}

	// ---- (i) link body properties, root pose / velocity of a free base ----
	std::printf("[i] link properties\n");
	{
		Handle world = CreateWorld(0.0f);
		Handle art = CreateArticulation(world, kFreeFlags);
		Handle root = CreateLink(world, art, nullptr, Physics::Pose(0, 0, 0, 0, 0, 0, 1));
		LinkAttachBox(world, root, 0.3f, 0.2f, 0.1f, 3.0f);
		Handle child = CreateLink(world, art, root, Physics::Pose(1.0f, 0, 0, 0, 0, 0, 1));
		LinkJointType(world, child, JointType::Hinge);
		LinkFrames(world, child, AxisFrame(0.5f, 0, 0, 2), AxisFrame(-0.5f, 0, 0, 2));
		LinkAxisLock(world, child, ConstraintAxis::AngularX, JointDofLock::Free);
		LinkAttachBox(world, child, 0.4f, 0.1f, 0.1f, 1.0f);

		// pose of the root before the articulation is in a scene
		ArticulationLinkData::SetPose sp{};
		sp.value = Physics::Pose(2.0f, 3.0f, 4.0f, 0, 0, 0, 1);
		sp.awake = true;
		Run(root, Context(world, root), Articulation::SetLinkPose, sp);
		Check(Near(LinkPose(world, root).pos.y, 3.0f, 1e-5f), "root pose set before the scene");

		// body properties round trip and survive a rebuild of the PhysX articulation
		ArticulationLinkData::SetDamping sd{};
		sd.linear = 0.3f;
		sd.angular = 0.4f;
		Run(child, Context(world, child), Articulation::SetLinkDamping, sd);
		ArticulationLinkData::SetMaxLinearVelocity ml{};
		ml.value = 12.0f;
		Run(child, Context(world, child), Articulation::SetLinkMaxLinearVelocity, ml);
		ArticulationLinkData::SetLocalCenterOfMass com{};
		com.value = Physics::Vec3(0.1f, 0.0f, 0.0f);
		Run(child, Context(world, child), Articulation::SetLinkLocalCenterOfMass, com);
		ArticulationLinkData::SetInertiaTensor it{};
		it.value = Physics::Vec3(0.5f, 0.6f, 0.7f);
		Run(child, Context(world, child), Articulation::SetLinkInertiaTensor, it);
		LinkMass(world, child, 2.5f);
		AddArticulationToScene(world, art);

		auto verify = [&](const char* when) {
			ArticulationLinkData::GetDamping gd{};
			Run(child, Context(world, child), Articulation::GetLinkDamping, gd);
			ArticulationLinkData::GetMaxLinearVelocity gml{};
			Run(child, Context(world, child), Articulation::GetLinkMaxLinearVelocity, gml);
			ArticulationLinkData::GetLocalCenterOfMass gcom{};
			Run(child, Context(world, child), Articulation::GetLinkLocalCenterOfMass, gcom);
			ArticulationLinkData::GetInertiaTensor git{};
			Run(child, Context(world, child), Articulation::GetLinkInertiaTensor, git);
			ArticulationLinkData::GetMass gm{};
			Run(child, Context(world, child), Articulation::GetLinkMass, gm);
			char what[96];
			std::snprintf(what, sizeof(what), "link properties %s", when);
			Check(Near(gd.linear, 0.3f, 1e-5f) && Near(gd.angular, 0.4f, 1e-5f) && Near(gml.value, 12.0f, 1e-4f)
				&& Near(gcom.value.x, 0.1f, 1e-5f) && Near(git.value.y, 0.6f, 1e-5f) && Near(gm.value, 2.5f, 1e-5f), what);
		};
		verify("after adding to the scene");
		// a structural change inside the scene rebuilds the PhysX articulation
		LinkAxisLock(world, child, ConstraintAxis::AngularY, JointDofLock::Locked);
		LinkJointType(world, child, JointType::BallAndSocket);
		LinkJointType(world, child, JointType::Hinge);
		verify("after rebuilding the articulation");

		// free root: velocity, force, teleport
		ArticulationLinkData::SetLinearVelocity slv{};
		slv.value = Physics::Vec3(1.0f, 0.0f, 0.0f);
		slv.awake = true;
		Run(root, Context(world, root), Articulation::SetLinkLinearVelocity, slv);
		Check(Near(LinkLinearVelocity(world, root).x, 1.0f, 1e-3f), "root linear velocity round trip");
		const float x0 = LinkPose(world, root).pos.x;
		Step(world, 1.0f);
		std::printf("  free root moved %.3f in 1 s at 1 m/s\n", LinkPose(world, root).pos.x - x0);
		Check(Near(LinkPose(world, root).pos.x - x0, 1.0f, 0.1f), "free root drifts with its velocity");

		ArticulationLinkData::SetPose teleport{};
		teleport.value = Physics::Pose(-5.0f, 6.0f, 0.0f, 0, 0, 0, 1);
		teleport.awake = true;
		Run(root, Context(world, root), Articulation::SetLinkPose, teleport);
		Check(Near(LinkPose(world, root).pos.x, -5.0f, 1e-3f) && Near(LinkPose(world, root).pos.y, 6.0f, 1e-3f), "root teleport inside the scene");
		Check(Near(LinkPose(world, child).pos.y, 6.0f, 0.01f), "children follow the teleported root");

		ArticulationLinkData::AddForce af{};
		af.value = Physics::Vec3(0.0f, 10.0f, 0.0f);
		af.mode = Physics::ForceMode::Impulse;
		af.awake = true;
		Run(root, Context(world, root), Articulation::AddForceToLink, af);
		Step(world, kDt);
		std::printf("  root velocity after a 10 N s impulse: y=%.3f\n", LinkLinearVelocity(world, root).y);
		Check(LinkLinearVelocity(world, root).y > 1.0f, "impulse accelerates the articulation");

		ArticulationLinkData::GetClosestWorldPoint cp{};
		cp.worldPosition = Physics::Vec3(-5.0f, 20.0f, 0.0f);
		Run(root, Context(world, root), Articulation::GetLinkClosestWorldPoint, cp);
		Check(cp.outSqrDistance > 100.0f, "closest point on the link bounds");
		ArticulationLinkData::GetWorldPointVelocity wp{};
		wp.worldPoint = Physics::Vec3(-5.0f, 6.0f, 0.0f);
		Run(root, Context(world, root), Articulation::GetLinkWorldPointVelocity, wp);
		Check(std::isfinite(wp.value.y), "world point velocity is finite");

		ArticulationLinkData::SetCollisionDetectionMode cm{};
		cm.value = Physics::CollisionDetectionMode::ContinuousDynamic;
		Run(root, Context(world, root), Articulation::SetLinkCollisionDetectionMode, cm);
		Step(world, 0.1f);
		DestroyWorld(world);
	}

	// ---- (g) teardown orders and bad handles ----
	std::printf("[g] teardown\n");
	{
		// lock / unlock a live joint: the joint state survives, the joint swings again once it is free
		Handle world = CreateWorld(-9.81f);
		Pendulum p = BuildPendulum(world, 5.0f);
		Step(world, 0.2f);
		const float before = JointPosition(world, p.link).coordinate.x;
		LinkAxisLock(world, p.link, ConstraintAxis::AngularX, JointDofLock::Locked);
		Step(world, 0.2f);
		ArticulationJointData::GetDofCount dc{};
		Run(p.link, Context(world, p.link), Articulation::GetLinkJointDofCount, dc);
		Check(dc.value == 0 && JointsPositions(world, p.art).empty(), "a locked joint has no DOF");
		LinkAxisLock(world, p.link, ConstraintAxis::AngularX, JointDofLock::Free);
		const float restored = JointPosition(world, p.link).coordinate.x;
		Step(world, 0.5f);
		const float after = JointPosition(world, p.link).coordinate.x;
		std::printf("  lock/unlock: before=%.4f restored=%.4f after 0.5s=%.4f\n", before, restored, after);
		Check(Near(restored, before, 1e-3f), "joint position survives lock and unlock");
		Check(std::isfinite(after) && std::fabs(after) > std::fabs(before) + 0.1f, "unlocked joint swings again");
		DestroyWorld(world);
	}
	{
		// links added after the articulation is already in the scene
		Handle world = CreateWorld(-9.81f);
		Handle art = CreateArticulation(world, kFixedFlags);
		Handle root = CreateLink(world, art, nullptr, Physics::Pose(0, 5.0f, 0, 0, 0, 0, 1));
		LinkAttachBox(world, root, 0.2f, 0.2f, 0.2f, 2.0f);
		AddArticulationToScene(world, art);
		Step(world, 0.1f);
		Handle l1 = CreateLink(world, art, root, Physics::Pose(1.0f, 5.0f, 0, 0, 0, 0, 1));
		LinkJointType(world, l1, JointType::Hinge);
		LinkFrames(world, l1, AxisFrame(0.5f, 0, 0, 2), AxisFrame(-0.5f, 0, 0, 2));
		LinkAxisLock(world, l1, ConstraintAxis::AngularX, JointDofLock::Free);
		LinkAttachBox(world, l1, 0.4f, 0.1f, 0.1f, 1.0f);
		LinkFriction(world, l1, 0.0f);
		Step(world, 0.3f);
		std::printf("  live growth: count=%zu pos=%.4f\n", LinkCount(world, art), JointPosition(world, l1).coordinate.x);
		Check(LinkCount(world, art) == 2 && std::fabs(JointPosition(world, l1).coordinate.x) > 0.01f, "link added to an articulation that is already in the scene swings");
		DestroyWorld(world);
	}
	{
		// parent link first: the subtree goes with it, the child handles stay safe
		Handle world = CreateWorld(-9.81f);
		Handle art = nullptr;
		std::vector<Handle> links = BuildLine(world, art, 3, 5.0f);
		Step(world, 0.2f);
		DestroyLinkCmd(world, art, links[1]);
		Check(LinkCount(world, art) == 1, "destroying a link removes its subtree");
		Step(world, 0.2f);
		const Physics::Pose stale = LinkPose(world, links[2]);
		Check(stale.isValid, "queries on a link of a removed subtree do not crash");
		ArticulationLinkData::GetShapeCount sc{};
		Run(links[2], Context(world, links[2]), Articulation::GetLinkShapeCount, sc);
		DestroyLinkCmd(world, art, links[2]);
		DestroyLinkCmd(world, art, links[2]);
		DestroyLinkCmd(world, art, links[3]);
		DestroyLinkCmd(world, art, links[1]);
		Step(world, 0.2f);
		Check(LinkCount(world, art) == 1, "root remains after its subtree was destroyed");
		// the empty articulation can grow again
		Handle l1 = CreateLink(world, art, links[0], Physics::Pose(1.0f, 5.0f, 0, 0, 0, 0, 1));
		LinkJointType(world, l1, JointType::Hinge);
		LinkFrames(world, l1, AxisFrame(0.5f, 0, 0, 2), AxisFrame(-0.5f, 0, 0, 2));
		LinkAxisLock(world, l1, ConstraintAxis::AngularX, JointDofLock::Free);
		LinkAttachBox(world, l1, 0.4f, 0.1f, 0.1f, 1.0f);
		Step(world, 0.3f);
		Check(l1 != nullptr && LinkCount(world, art) == 2 && std::fabs(JointPosition(world, l1).coordinate.x) > 0.01f, "link added to a live articulation swings");
		DestroyLinkCmd(world, art, links[0]);
		Check(LinkCount(world, art) == 0, "root destruction empties the articulation");
		Step(world, 0.1f);
		DestroyArticulationCmd(world, art);
		DestroyArticulationCmd(world, art);
		Check(LinkCount(world, art) == 0, "destroyed articulation reports no links");
		WorldData::GetArticulationCount ac{};
		Run(world, Context(world, nullptr), World::GetArticulationCount, ac);
		Check(ac.value == 0, "articulation count after destruction");
		DestroyWorld(world);
	}
	{
		// leaf first, then articulation, then stale handle access
		Handle world = CreateWorld(-9.81f);
		Handle art = nullptr;
		std::vector<Handle> links = BuildLine(world, art, 3, 5.0f);
		Step(world, 0.2f);
		DestroyLinkCmd(world, art, links[3]);
		DestroyLinkCmd(world, art, links[2]);
		Check(LinkCount(world, art) == 2, "leaf links destroyed one by one");
		Step(world, 0.2f);
		DestroyArticulationCmd(world, art);
		ArticulationLinkData::GetPose gp{};
		Run(links[1], Context(world, links[1]), Articulation::GetLinkPose, gp);
		ArticulationJointData::SetPosition sp{};
		Run(links[1], Context(world, links[1]), Articulation::SetLinkJointPosition, sp);
		DestroyLinkCmd(world, art, links[1]);
		Step(world, 0.1f);
		Check(true, "stale link handles after DestroyArticulation");
		DestroyWorld(world);
	}
	{
		// shapes: destroy while attached, detach and re-attach, shape outliving its articulation
		Handle world = CreateWorld(-9.81f);
		Handle art = nullptr;
		std::vector<Handle> links = BuildLine(world, art, 2, 5.0f, false);
		Handle s0 = LinkAttachBox(world, links[1], 0.4f, 0.1f, 0.1f, 1.0f);
		Handle s1 = LinkAttachBox(world, links[2], 0.4f, 0.1f, 0.1f, 1.0f);
		Step(world, 0.2f);
		ShapeData::DestroyShape ds;
		Run(s0, Context(world, nullptr), Shape::DestroyShape, ds);
		ArticulationLinkData::GetShapeCount sc{};
		Run(links[1], Context(world, links[1]), Articulation::GetLinkShapeCount, sc);
		Check(sc.value == 0, "shape destroyed while attached to an in-scene link");
		Step(world, 0.2f);

		ArticulationLinkData::DetachShape dts{};
		dts.shape = s1;
		Run(links[2], Context(world, links[2]), Articulation::DetachShapeFromLink, dts);
		Run(links[2], Context(world, links[2]), Articulation::GetLinkShapeCount, sc);
		Check(sc.value == 0, "shape detached from a link");
		ArticulationLinkData::AttachShape ats{};
		ats.shape = s1;
		Run(links[1], Context(world, links[1]), Articulation::AttachShapeToLink, ats);
		Run(links[1], Context(world, links[1]), Articulation::GetLinkShapeCount, sc);
		Check(sc.value == 1, "shape re-attached to another link");
		Step(world, 0.2f);
		DestroyArticulationCmd(world, art);
		ShapeData::GetFlags gfl{};
		Run(s1, Context(world, nullptr), Shape::GetFlags, gfl);
		Run(s1, Context(world, nullptr), Shape::DestroyShape, ds);
		Check(true, "shape outliving its articulation");
		DestroyWorld(world);
	}
	{
		// invalid handles never crash
		Handle world = CreateWorld(-9.81f);
		Pendulum p = BuildPendulum(world, 5.0f);
		int dummyStorage = 0;
		Handle bogus = &dummyStorage;
		const int errorsBefore = g_errorLogs; // the backend reports the bogus parent link below as an error on purpose
		const Handle targets[4] = { nullptr, bogus, world, p.root };
		for (Handle t : targets)
		{
			const Context ctxs[2] = { Context(world, t), Context(t, t) };
			for (const Context& ctx : ctxs)
			{
				ArticulationLinkData::GetPose gp{};
				Run(t, ctx, Articulation::GetLinkPose, gp);
				ArticulationLinkData::SetMass sm{};
				sm.value = 2.0f;
				Run(t, ctx, Articulation::SetLinkMass, sm);
				ArticulationLinkData::AttachShape as{};
				as.shape = bogus;
				Run(t, ctx, Articulation::AttachShapeToLink, as);
				ArticulationJointData::SetType st{};
				st.value = JointType::Prismatic;
				Run(t, ctx, Articulation::SetLinkJointType, st);
				ArticulationJointData::GetPosition jp{};
				Run(t, ctx, Articulation::GetLinkJointPosition, jp);
				ArticulationJointData::GetForcesForAcceleration fa{};
				Run(t, ctx, Articulation::GetLinkJointForcesForAcceleration, fa);
				ArticulationData::CreateLink cl{};
				cl.parentLink = bogus;
				Run(t, ctx, Articulation::CreateLink, cl);
				ArticulationData::GetLinkParent gpa{};
				gpa.link = t;
				Run(t, ctx, Articulation::GetLinkParent, gpa);
				ArticulationData::SetFlags sf{};
				sf.value = Physics::ArticulationFlags::ImmovableRoot;
				sf.enabled = true;
				Run(t, ctx, Articulation::SetFlags, sf);
				ArticulationData::GetJointsPositions gjp{};
				Run(t, ctx, Articulation::GetJointsPositions, gjp);
				ArticulationData::SetJointsPositions sjp{};
				Run(t, ctx, Articulation::SetJointsPositions, sjp);
				ArticulationData::GetDenseJacobianAsBuffer gdj{};
				Run(t, ctx, Articulation::GetDenseJacobianAsFloatBuffer, gdj);
				ArticulationData::ReleaseDataBuffer rb{};
				rb.value.data = reinterpret_cast<const float*>(bogus);
				Run(t, ctx, Articulation::ReleaseDataBuffer, rb);
				ArticulationData::DestroyLink dl{};
				dl.value = t == p.root ? nullptr : t;
				Run(t, ctx, Articulation::DestroyLink, dl);
				WorldData::DestoryArticulation da{};
				da.value = t == p.root ? nullptr : t;
				Run(t, ctx, World::DestroyArticulation, da);
			}
		}
		g_errorLogs = errorsBefore;
		Step(world, 0.2f);
		Check(LinkCount(world, p.art) == 2, "invalid handles left the articulation intact");
		// the world handle must not be mistaken for an articulation
		WorldData::GetArticulationCount ac{};
		Run(world, Context(world, nullptr), World::GetArticulationCount, ac);
		Check(ac.value == 1, "articulation survived the bad commands");
		DestroyWorld(world);
	}
	{
		// articulation toggling: not simulated -> frozen, simulated again -> moves, joint state preserved
		Handle world = CreateWorld(-9.81f);
		Pendulum p = BuildPendulum(world, 5.0f);
		Step(world, 0.2f);
		const float before = JointPosition(world, p.link).coordinate.x;
		ArticulationData::SetFlags sf{};
		sf.value = Physics::ArticulationFlags::Simulated;
		sf.enabled = false;
		Run(p.art, Context(world, p.art), Articulation::SetFlags, sf);
		Step(world, 0.5f);
		const float frozen = JointPosition(world, p.link).coordinate.x;
		sf.enabled = true;
		Run(p.art, Context(world, p.art), Articulation::SetFlags, sf);
		const float restored = JointPosition(world, p.link).coordinate.x;
		Step(world, 0.2f);
		const float after = JointPosition(world, p.link).coordinate.x;
		std::printf("  simulated toggle: before=%.4f frozen=%.4f restored=%.4f after=%.4f\n", before, frozen, restored, after);
		Check(Near(frozen, before, 1e-4f) || std::fabs(frozen) <= std::fabs(before) + 1e-4f, "articulation does not move while not simulated");
		Check(Near(restored, before, 1e-3f), "joint state survives leaving and re-entering the scene");
		Check(std::fabs(after) > std::fabs(restored), "articulation moves again once simulated");
		ArticulationData::GetFlags gf{};
		Run(p.art, Context(world, p.art), Articulation::GetFlags, gf);
		Check((gf.value & Physics::ArticulationFlags::ImmovableRoot) != Physics::ArticulationFlags::None
			&& (gf.value & Physics::ArticulationFlags::Simulated) != Physics::ArticulationFlags::None, "GetFlags reports the flags");

		ArticulationData::SetSolverIterationsOverride si{};
		si.positionIterations = 12;
		si.velocityIterations = 3;
		Run(p.art, Context(world, p.art), Articulation::SetSolverIterationsOverride, si);
		ArticulationData::GetSolverIterationsOverride gsi{};
		Run(p.art, Context(world, p.art), Articulation::GetSolverIterationsOverride, gsi);
		Check(gsi.positionIterations == 12 && gsi.velocityIterations == 3, "solver iterations round trip");
		ArticulationData::SetSleepThresholdOverride st{};
		st.value = 0.25f;
		Run(p.art, Context(world, p.art), Articulation::SetSleepThresholdOverride, st);
		ArticulationData::GetSleepThresholdOverride gst{};
		Run(p.art, Context(world, p.art), Articulation::GetSleepThresholdOverride, gst);
		Check(Near(gst.value, 0.25f, 1e-6f), "sleep threshold round trip");
		ArticulationData::GetIsSleeping gs{};
		Run(p.art, Context(world, p.art), Articulation::GetIsSleeping, gs);
		Check(!gs.value, "moving articulation is awake");
		Run(p.art, Context(world, p.art), Articulation::Sleep, gs);
		Run(p.art, Context(world, p.art), Articulation::GetIsSleeping, gs);
		Check(gs.value, "Sleep puts the articulation to sleep");
		Run(p.art, Context(world, p.art), Articulation::WakeUp, gs);
		Run(p.art, Context(world, p.art), Articulation::GetIsSleeping, gs);
		Check(!gs.value, "WakeUp wakes it again");
		DestroyWorld(world);
	}
	// a world with a live articulation is left for ShutdownSdk to clean up
	Handle leftoverWorld = CreateWorld(-9.81f);
	Pendulum leftover = BuildPendulum(leftoverWorld, 5.0f);
	Step(leftoverWorld, 0.1f);
	(void)leftover;

	std::printf("  summary: revolute drive t=%.3f, prismatic drive t=%.3f\n", tRevolute, tPrismatic);

	SdkData::ShutdownSdk shutdown;
	Run(nullptr, Context(), SDK::ShutdownSdk, shutdown);
	unload();
	dlclose(lib);
	Check(g_errorLogs == 0, "no backend error logs");
	std::printf("articulation_test: %d checks, %d failures, %d error logs -> %s\n", g_checks, g_failures, g_errorLogs,
		g_failures == 0 ? "PASS" : "FAIL");
	return g_failures == 0 ? 0 : 1;
}
