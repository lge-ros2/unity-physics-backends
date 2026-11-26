// Collision mesh cooking and heightfield checks, driven through the Unity command entry point.
// Usage: mesh_test [path/to/libBackend.so]

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
	int g_warnLogs = 0;
	int g_failures = 0;
	int g_checks = 0;

	void LogFn(UnityLogType type, const char* message, const char*, int)
	{
		if (type == kUnityLogTypeError)
		{
			++g_errorLogs;
			std::printf("  [backend error] %s\n", message);
		}
		else if (type == kUnityLogTypeWarning)
		{
			++g_warnLogs;
			if (std::getenv("MESH_TEST_VERBOSE") != nullptr)
			{
				std::printf("  [backend warning] %s\n", message);
			}
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

	const float kDt = 1.0f / 60.0f;
	const Physics::BodyFlags kDynamicFlags = Physics::BodyFlags::Gravity | Physics::BodyFlags::Simulated;
	Handle g_material = nullptr;

	// ---- thin command wrappers ----

	Handle CreateWorld()
	{
		WorldData::CreateWorld cw;
		cw.jobWorkerCount = 1;
		cw.gravity = Physics::Vec3(0.0f, -9.81f, 0.0f);
		Run(nullptr, Context(), World::CreateWorld, cw);
		return cw.outWorld;
	}

	void Simulate(Handle world, int steps)
	{
		for (int i = 0; i < steps; ++i)
		{
			WorldData::Simulate sim;
			sim.deltaTime = kDt;
			Run(world, Context(world, nullptr), World::Simulate, sim);
		}
	}

	Handle CreateBody(Handle world, bool isStatic, const Physics::Vec3& pos, Physics::BodyFlags flags = kDynamicFlags)
	{
		WorldData::CreateBody cb{};
		cb.pose = Physics::Pose(pos.x, pos.y, pos.z, 0, 0, 0, 1);
		cb.mass = isStatic ? 0.0f : 1.0f;
		cb.angularDamping = 0.05f;
		cb.flags = isStatic ? Physics::BodyFlags::Simulated : flags;
		cb.isStatic = isStatic;
		Run(world, Context(world, nullptr), World::CreateBody, cb);
		return cb.outBody;
	}

	Handle CreateShape(Handle world, const Physics::ShapeGeometry& geom)
	{
		ShapeData::CreateShape cs{};
		cs.geom = geom;
		cs.materialPtr = g_material;
		cs.flags = Physics::ShapeFlags::SceneQuery;
		Run(nullptr, Context(world, nullptr), Shape::CreateShape, cs);
		if (cs.value != nullptr)
		{
			ShapeData::SetFilterData sf{};
			sf.data.belongsTo = 1;
			sf.data.collidesWith = 0xFFFFFFFFu;
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

	void SetBodyFlag(Handle world, Handle body, Physics::BodyFlags flag, bool enabled)
	{
		BodyData::SetFlags sf{};
		sf.value = flag;
		sf.enabled = enabled;
		Run(body, Context(world, body), Body::SetFlags, sf);
	}

	Physics::Vec3 Position(Handle world, Handle body)
	{
		BodyData::GetPose gp{};
		Run(body, Context(world, body), Body::GetPose, gp);
		return gp.value.pos;
	}

	void DestroyShape(Handle world, Handle body, Handle shape)
	{
		ShapeData::DestroyShape ds;
		Run(shape, Context(world, body), Shape::DestroyShape, ds);
	}

	void DestroyBody(Handle world, Handle body)
	{
		WorldData::DestroyBody db{};
		db.value = body;
		Run(world, Context(world, nullptr), World::DestroyBody, db);
	}

	void DestroyWorld(Handle world)
	{
		ShapeData::DestroyShape ds; // any empty command; DestroyWorld carries no payload
		Run(world, Context(world, nullptr), World::DestroyWorld, ds);
	}

	// ---- geometry helpers ----

	Physics::ShapeGeometry BoxGeom(float h)
	{
		Physics::ShapeGeometry g = {};
		g.type = Physics::GeometryType::Box;
		g.boxGeometry.halfExtents = Physics::Vec3(h, h, h);
		return g;
	}

	Physics::ShapeGeometry SphereGeom(float r)
	{
		Physics::ShapeGeometry g = {};
		g.type = Physics::GeometryType::Sphere;
		g.sphereGeometry.radius = r;
		return g;
	}

	Physics::ShapeGeometry MeshGeom(Physics::GeometryType type, Handle mesh, const Physics::Vec3& scale = Physics::Vec3(1.0f, 1.0f, 1.0f))
	{
		Physics::ShapeGeometry g = {};
		g.type = type;
		g.meshGeometry.mesh = mesh;
		g.meshGeometry.scale = scale;
		return g;
	}

	Physics::ShapeGeometry TerrainGeom(Handle field, const Physics::Vec3& scale)
	{
		Physics::ShapeGeometry g = {};
		g.type = Physics::GeometryType::Terrain;
		g.heightFieldGeometry.heightField = field;
		g.heightFieldGeometry.scale = scale;
		return g;
	}

	// ---- cooking helpers ----

	struct CookResult
	{
		int calls = 0;
		Physics::CookingError status = Physics::CookingError::Success;
		Handle mesh = nullptr;
		std::vector<uint8_t> stream;
	};

	void OnMeshReady(Physics::CookingError result, const char*, void* context, Handle mesh)
	{
		auto* r = static_cast<CookResult*>(context);
		++r->calls;
		r->status = result;
		r->mesh = mesh;
	}

	void OnStreamReady(Physics::CookingError result, const char*, void* context, void* stream, size_t size)
	{
		auto* r = static_cast<CookResult*>(context);
		++r->calls;
		r->status = result;
		r->stream.assign(static_cast<uint8_t*>(stream), static_cast<uint8_t*>(stream) + (stream != nullptr ? size : 0));
	}

	struct MeshData
	{
		std::vector<float> vertices; // xyz
		std::vector<uint32_t> indices;
	};

	MeshData QuadMesh(float half)
	{
		MeshData m;
		m.vertices = { -half, 0, -half, -half, 0, half, half, 0, half, half, 0, -half };
		m.indices = { 0, 1, 3, 1, 2, 3 };
		return m;
	}

	MeshData CubeMesh(float h)
	{
		MeshData m;
		for (int i = 0; i < 8; ++i)
		{
			m.vertices.push_back((i & 1) ? h : -h);
			m.vertices.push_back((i & 2) ? h : -h);
			m.vertices.push_back((i & 4) ? h : -h);
		}
		return m;
	}

	Physics::CollisionMeshCookingDescriptor MakeDescriptor(const MeshData& m, bool convex, std::vector<uint16_t>& indices16, bool use16)
	{
		Physics::CollisionMeshCookingDescriptor d;
		d.vertexStream = const_cast<float*>(m.vertices.data());
		d.vertexElementCount = static_cast<uint32_t>(m.vertices.size() / 3);
		d.vertexElementSize = 12;
		d.convex = convex;
		d.options = Physics::CollisionMeshProcessingOptions::OptimizeForRuntimePerformance
			| Physics::CollisionMeshProcessingOptions::CleanupInputMesh | Physics::CollisionMeshProcessingOptions::WeldVertices;
		if (!m.indices.empty())
		{
			d.indexElementCount = static_cast<uint32_t>(m.indices.size());
			if (use16)
			{
				indices16.assign(m.indices.begin(), m.indices.end());
				d.indexStream = indices16.data();
				d.indexElementSize = 2;
			}
			else
			{
				d.indexStream = const_cast<uint32_t*>(m.indices.data());
				d.indexElementSize = 4;
			}
		}
		return d;
	}

	Handle CookMesh(const MeshData& m, bool convex, bool use16 = false, Physics::CookingError* status = nullptr)
	{
		std::vector<uint16_t> idx16;
		ShapeData::CookCollisionMesh cook{};
		cook.descriptor = MakeDescriptor(m, convex, idx16, use16);
		CookResult result;
		cook.dataContext = &result;
		cook.onMeshReadyCallback = &OnMeshReady;
		Run(nullptr, Context(), Shape::CookCollisionMesh, cook);
		Check(result.calls == 1, "cook callback invoked exactly once");
		if (status != nullptr)
		{
			*status = result.status;
		}
		return result.mesh;
	}

	std::vector<uint8_t> CookStream(const MeshData& m, bool convex)
	{
		std::vector<uint16_t> idx16;
		ShapeData::CookCollisionMeshStream cook{};
		cook.descriptor = MakeDescriptor(m, convex, idx16, false);
		CookResult result;
		cook.dataContext = &result;
		cook.onStreamReadyCallback = &OnStreamReady;
		Run(nullptr, Context(), Shape::CookCollisionMeshStream, cook);
		Check(result.calls == 1, "stream callback invoked exactly once");
		return result.stream;
	}

	Handle MeshFromStream(std::vector<uint8_t>& stream, bool convex)
	{
		ShapeData::CreateCollisionMeshFromStream cs{};
		cs.stream = stream.data();
		cs.streamSize = stream.size();
		cs.convex = convex;
		Run(nullptr, Context(), Shape::CreateCollisionMeshFromStream, cs);
		return cs.outCollisionMesh;
	}

	void DestroyMesh(Handle mesh, bool convex)
	{
		ShapeData::DestroyCollisionMesh dm{};
		dm.collisionMesh = mesh;
		dm.convex = convex;
		Run(nullptr, Context(), Shape::DestroyCollisionMesh, dm);
	}

	// ---- heightfield helpers ----

	Handle CreateHeightField(const std::vector<int16_t>& heights, int resolution, const std::vector<uint8_t>* holes = nullptr)
	{
		ShapeData::CreateHeightField ch;
		ch.descriptor.heights = heights.data();
		ch.descriptor.heightsCount = static_cast<int>(heights.size());
		ch.descriptor.resolution = resolution;
		if (holes != nullptr)
		{
			ch.descriptor.holes = holes->data();
			ch.descriptor.holesCount = static_cast<int>(holes->size());
		}
		Run(nullptr, Context(), Shape::CreateHeightField, ch);
		return static_cast<Handle>(ch.heightField);
	}

	void DestroyHeightField(Handle field)
	{
		ShapeData::DestroyHeightField dh;
		dh.heightField = field;
		Run(nullptr, Context(), Shape::DestroyHeightField, dh);
	}

	void UpdateHeightField(Handle field, const std::vector<int16_t>& heights, int resolution, int x0, int z0, int w, int h)
	{
		ShapeData::UpdateHeightFieldRegion up{};
		up.descriptor.heights = heights.data();
		up.descriptor.heightsCount = static_cast<int>(heights.size());
		up.descriptor.resolution = resolution;
		up.heightField = field;
		up.xBase = x0;
		up.yBase = z0;
		up.width = w;
		up.height = h;
		Run(nullptr, Context(), Shape::UpdateHeightFieldRegion, up);
	}

	// Static body + shape added to the scene.
	Handle AddStatic(Handle world, const Physics::ShapeGeometry& geom, Handle* shapeOut = nullptr)
	{
		Handle body = CreateBody(world, true, Physics::Vec3(0, 0, 0));
		Handle shape = CreateShape(world, geom);
		if (shape != nullptr)
		{
			AttachShape(world, body, shape);
		}
		AddToScene(world, body);
		if (shapeOut != nullptr)
		{
			*shapeOut = shape;
		}
		return body;
	}

	// Dynamic body with one shape, added to the scene.
	Handle AddDynamic(Handle world, const Physics::ShapeGeometry& geom, const Physics::Vec3& pos, Handle* shapeOut = nullptr)
	{
		Handle body = CreateBody(world, false, pos);
		Handle shape = CreateShape(world, geom);
		if (shape != nullptr)
		{
			AttachShape(world, body, shape);
		}
		Recompute(world, body);
		AddToScene(world, body);
		if (shapeOut != nullptr)
		{
			*shapeOut = shape;
		}
		return body;
	}

	struct ExtractResult
	{
		int calls = 0;
		Physics::SDKCollisionMeshData data = {};
		std::vector<uint32_t> indices;
	};

	void OnExtract(void* context, Physics::SDKCollisionMeshData data)
	{
		auto* r = static_cast<ExtractResult*>(context);
		++r->calls;
		r->data = data;
		for (size_t i = 0; i < data.indexCount; ++i)
		{
			uint32_t v = 0;
			std::memcpy(&v, static_cast<const uint8_t*>(data.indices) + i * data.indexStride, data.indexStride);
			r->indices.push_back(data.indexStride == 2 ? (v & 0xFFFFu) : v);
		}
	}

	ExtractResult Extract(Handle sdkObject, const Context& ctx)
	{
		ExtractResult result;
		ShapeData::ExtractCollisionMeshData ex{};
		ex.ctx = &result;
		ex.callback = &OnExtract;
		Run(sdkObject, ctx, Shape::ExtractCollisionMeshData, ex);
		return result;
	}

	bool IndicesInRange(const ExtractResult& r)
	{
		for (uint32_t i : r.indices)
		{
			if (i >= r.data.vertexCount)
			{
				return false;
			}
		}
		return true;
	}

	// ---- scenarios ----

	// Drops a box and a sphere on a static shape and checks where they come to rest.
	void DropOnFloor(const char* label, Handle world, float floorTopY)
	{
		char text[128];
		Handle box = AddDynamic(world, BoxGeom(0.5f), Physics::Vec3(-2.0f, floorTopY + 3.0f, 1.0f));
		Handle sphere = AddDynamic(world, SphereGeom(0.5f), Physics::Vec3(2.0f, floorTopY + 3.0f, -1.0f));
		Simulate(world, 240);
		const Physics::Vec3 pb = Position(world, box);
		const Physics::Vec3 ps = Position(world, sphere);
		std::printf("  %s: box y=%.4f sphere y=%.4f (expect %.3f)\n", label, pb.y, ps.y, floorTopY + 0.5f);
		std::snprintf(text, sizeof(text), "%s: box rests on the floor", label);
		Check(Near(pb.y, floorTopY + 0.5f, 0.04f) && Near(pb.x, -2.0f, 0.3f), text);
		std::snprintf(text, sizeof(text), "%s: sphere rests on the floor", label);
		Check(Near(ps.y, floorTopY + 0.5f, 0.04f), text);
	}

	void TestTriangleMeshFloor(Handle world, Handle* meshOut)
	{
		std::printf("(a) triangle mesh floor\n");
		const MeshData quad = QuadMesh(10.0f);
		Physics::CookingError status = Physics::CookingError::Failed;
		Handle mesh = CookMesh(quad, false, false, &status);
		Check(mesh != nullptr && status == Physics::CookingError::Success, "triangle mesh cooked");

		Handle shape = nullptr;
		AddStatic(world, MeshGeom(Physics::GeometryType::TriangleMesh, mesh), &shape);
		Check(shape != nullptr, "triangle mesh shape created");

		ShapeData::GetGeometryType ggt{};
		Run(shape, Context(world, nullptr), Shape::GetGeometryType, ggt);
		Check(ggt.value == Physics::GeometryType::TriangleMesh, "GetGeometryType = TriangleMesh");
		ShapeData::GetGeometry gg{};
		Run(shape, Context(world, nullptr), Shape::GetGeometry, gg);
		Check(gg.value.type == Physics::GeometryType::TriangleMesh && gg.value.meshGeometry.mesh == mesh, "GetGeometry returns the mesh handle");
		ShapeData::GetWorldBounds gwb{};
		Run(shape, Context(world, nullptr), Shape::GetWorldBounds, gwb);
		Check(Near(gwb.extents.x, 10.0f, 0.1f) && Near(gwb.extents.z, 10.0f, 0.1f) && gwb.extents.y < 0.1f, "triangle mesh world bounds");

		DropOnFloor("trimesh floor", world, 0.0f);

		// scaling the mesh geometry through SetGeometry doubles the floor size
		ShapeData::SetGeometry sg{};
		sg.value = MeshGeom(Physics::GeometryType::TriangleMesh, mesh, Physics::Vec3(2.0f, 1.0f, 2.0f));
		Run(shape, Context(world, nullptr), Shape::SetGeometry, sg);
		Run(shape, Context(world, nullptr), Shape::GetWorldBounds, gwb);
		Check(Near(gwb.extents.x, 20.0f, 0.2f), "SetGeometry rescales the mesh");

		// triangle indices map back to the source mesh
		for (uint32_t i = 0; i < 2; ++i)
		{
			ShapeData::RemapTriangleIndexToSourceIndex rm{};
			rm.physicsMeshTriangleIndex = i;
			rm.outSourceMeshTriangleIndex = 99;
			Run(shape, Context(world, nullptr), Shape::RemapTriangleIndexToSourceIndex, rm);
			Check(rm.outSourceMeshTriangleIndex < 2, "remapped triangle index is a valid source index");
		}
		*meshOut = mesh;
	}

	void TestConvex(Handle world)
	{
		std::printf("(b) convex mesh\n");
		Handle floor = CreateBody(world, true, Physics::Vec3(0, 0, 0));
		Handle floorShape = CreateShape(world, BoxGeom(0.5f));
		ShapeData::SetGeometry sg{};
		Physics::ShapeGeometry big = {};
		big.type = Physics::GeometryType::Box;
		big.boxGeometry.halfExtents = Physics::Vec3(50.0f, 0.5f, 50.0f);
		sg.value = big;
		Run(floorShape, Context(world, nullptr), Shape::SetGeometry, sg);
		AttachShape(world, floor, floorShape);
		AddToScene(world, floor);
		// floor top surface at y = 0.5

		const MeshData cube = CubeMesh(0.5f);
		Physics::CookingError status = Physics::CookingError::Failed;
		Handle mesh = CookMesh(cube, true, false, &status);
		Check(mesh != nullptr && status != Physics::CookingError::Failed, "convex mesh cooked");

		Handle shapeA = nullptr;
		Handle bodyA = AddDynamic(world, MeshGeom(Physics::GeometryType::ConvexMesh, mesh), Physics::Vec3(-3.0f, 4.0f, 0.0f), &shapeA);
		Handle shapeB = nullptr;
		Handle bodyB = AddDynamic(world, MeshGeom(Physics::GeometryType::ConvexMesh, mesh, Physics::Vec3(1.0f, 2.0f, 1.0f)),
			Physics::Vec3(3.0f, 5.0f, 0.0f), &shapeB);
		Check(shapeA != nullptr && shapeB != nullptr, "convex shapes created");

		ShapeData::GetGeometryType ggt{};
		Run(shapeA, Context(world, nullptr), Shape::GetGeometryType, ggt);
		Check(ggt.value == Physics::GeometryType::ConvexMesh, "GetGeometryType = ConvexMesh");

		Simulate(world, 300);
		const Physics::Vec3 pa = Position(world, bodyA);
		const Physics::Vec3 pb = Position(world, bodyB);
		std::printf("  convex cube y=%.4f stretched y=%.4f (expect 1.000 / 1.500)\n", pa.y, pb.y);
		Check(Near(pa.y, 1.0f, 0.05f), "convex cube rests on the box floor");
		Check(Near(pb.y, 1.5f, 0.05f), "scaled convex mesh rests with the scaled half height");

		// converting the dynamic body to kinematic and back must not trip PhysX
		SetBodyFlag(world, bodyA, Physics::BodyFlags::Kinematic, true);
		SetBodyFlag(world, bodyA, Physics::BodyFlags::Kinematic, false);
		Simulate(world, 10);

		// convex data extraction: 8 vertices, 12 triangles
		ExtractResult ex = Extract(shapeA, Context(world, nullptr));
		Check(ex.calls == 1 && ex.data.vertexCount == 8 && ex.data.indexCount == 36 && IndicesInRange(ex), "convex ExtractCollisionMeshData");

		DestroyMesh(mesh, true);
		Simulate(world, 5); // shapes keep the destroyed mesh alive
		Check(true, "simulating with a destroyed (still referenced) convex mesh");
		DestroyShape(world, bodyA, shapeA);
		DestroyShape(world, bodyB, shapeB);
		DestroyBody(world, bodyA);
		DestroyBody(world, bodyB);
		DestroyBody(world, floor);
		DestroyShape(world, nullptr, floorShape);
	}

	void TestStreams()
	{
		std::printf("(c) stream round trip\n");
		Handle world = CreateWorld();
		const MeshData quad = QuadMesh(10.0f);
		std::vector<uint8_t> stream = CookStream(quad, false);
		Check(stream.size() > 24, "triangle stream produced");

		Handle mesh = MeshFromStream(stream, false);
		Check(mesh != nullptr, "mesh created from stream");
		AddStatic(world, MeshGeom(Physics::GeometryType::TriangleMesh, mesh));
		DropOnFloor("stream floor", world, 0.0f);

		// stream cooked as convex must not load as triangle mesh and vice versa
		std::vector<uint8_t> convexStream = CookStream(CubeMesh(0.5f), true);
		Check(!convexStream.empty(), "convex stream produced");
		Handle convex = MeshFromStream(convexStream, true);
		Check(convex != nullptr, "convex mesh created from stream");
		Check(MeshFromStream(convexStream, false) == nullptr, "convex stream rejected as triangle mesh");

		// damaged streams
		const int warningsBefore = g_warnLogs;
		std::vector<uint8_t> bad = stream;
		bad.back() ^= 0x5A;
		Check(MeshFromStream(bad, false) == nullptr, "corrupted payload rejected");
		bad = stream;
		bad[0] ^= 0xFF;
		Check(MeshFromStream(bad, false) == nullptr, "wrong magic rejected");
		bad = stream;
		bad[4] ^= 0x7F; // format version
		Check(MeshFromStream(bad, false) == nullptr, "wrong format version rejected");
		bad = stream;
		bad[6] ^= 0x01; // PhysX major version
		Check(MeshFromStream(bad, false) == nullptr, "stream from another PhysX version rejected");
		bad.assign(stream.begin(), stream.begin() + 10);
		Check(MeshFromStream(bad, false) == nullptr, "truncated header rejected");
		bad.assign(stream.begin(), stream.end() - 7);
		Check(MeshFromStream(bad, false) == nullptr, "truncated payload rejected");
		bad.clear();
		ShapeData::CreateCollisionMeshFromStream empty{};
		Run(nullptr, Context(), Shape::CreateCollisionMeshFromStream, empty);
		Check(empty.outCollisionMesh == nullptr, "null stream rejected");
		Check(g_warnLogs >= warningsBefore + 6, "rejections are logged as warnings");

		DestroyMesh(mesh, false);
		DestroyMesh(convex, true);
		DestroyWorld(world);
	}

	void TestExtract()
	{
		std::printf("(d) extract / remap\n");
		const MeshData quad = QuadMesh(10.0f);
		for (int use16 = 0; use16 < 2; ++use16)
		{
			Handle mesh = CookMesh(quad, false, use16 != 0);
			ExtractResult ex = Extract(mesh, Context());
			Check(ex.calls == 1 && ex.data.vertexCount == 4 && ex.data.indexCount == 6 && IndicesInRange(ex),
				"triangle ExtractCollisionMeshData by mesh handle");
			Check(ex.data.vertexStride == 12 && (ex.data.indexStride == 2 || ex.data.indexStride == 4), "extract strides");
			DestroyMesh(mesh, false);
		}

		// welding merges duplicated vertices (6 -> 4)
		MeshData dup;
		dup.vertices = { -1, 0, -1, -1, 0, 1, 1, 0, -1, -1, 0, 1, 1, 0, 1, 1, 0, -1 };
		dup.indices = { 0, 1, 2, 3, 4, 5 };
		Handle mesh = CookMesh(dup, false);
		ExtractResult ex = Extract(mesh, Context());
		Check(ex.data.vertexCount == 4 && ex.data.indexCount == 6, "welded mesh keeps 4 vertices / 2 triangles");
		DestroyMesh(mesh, false);

		// invalid input fails through the callback without crashing
		MeshData badIndices = QuadMesh(1.0f);
		badIndices.indices[2] = 77;
		Physics::CookingError status = Physics::CookingError::Success;
		Handle failed = CookMesh(badIndices, false, false, &status);
		Check(failed == nullptr && status == Physics::CookingError::Failed, "out of range index fails cooking");
		MeshData tooFew;
		tooFew.vertices = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
		failed = CookMesh(tooFew, true, false, &status);
		Check(failed == nullptr && status == Physics::CookingError::Failed, "degenerate convex input fails cooking");

		ExtractResult none = Extract(reinterpret_cast<Handle>(0x1234), Context());
		Check(none.calls == 1 && none.data.vertexCount == 0, "extract with an unknown handle reports empty data");
	}

	// resolution 8, rows are x, columns are z; heights z-major as delivered by Unity
	std::vector<int16_t> FlatHeights(int resolution, int16_t value)
	{
		return std::vector<int16_t>(static_cast<size_t>(resolution) * resolution, value);
	}

	void TestHeightField()
	{
		std::printf("(e) heightfield\n");
		const int res = 8;
		const Physics::Vec3 scale(2.0f, 0.001f, 2.0f); // 14 x 14 m, 1000 -> 1 m
		Handle world = CreateWorld();

		Handle field = CreateHeightField(FlatHeights(res, 1000), res);
		Check(field != nullptr, "heightfield created");
		Handle terrainShape = nullptr;
		AddStatic(world, TerrainGeom(field, scale), &terrainShape);
		Check(terrainShape != nullptr, "terrain shape created");

		ShapeData::GetGeometryType ggt{};
		Run(terrainShape, Context(world, nullptr), Shape::GetGeometryType, ggt);
		Check(ggt.value == Physics::GeometryType::Terrain, "GetGeometryType = Terrain");
		ShapeData::GetWorldBounds gwb{};
		Run(terrainShape, Context(world, nullptr), Shape::GetWorldBounds, gwb);
		Check(Near(gwb.extents.x, 7.0f, 0.1f) && Near(gwb.center.x, 7.0f, 0.1f) && Near(gwb.center.y, 1.0f, 0.05f), "terrain world bounds");

		Handle sphere = AddDynamic(world, SphereGeom(0.5f), Physics::Vec3(7.0f, 4.0f, 7.0f));
		Handle box = AddDynamic(world, BoxGeom(0.5f), Physics::Vec3(3.0f, 4.0f, 10.0f));
		Simulate(world, 240);
		const Physics::Vec3 ps = Position(world, sphere);
		const Physics::Vec3 pb = Position(world, box);
		std::printf("  flat terrain: sphere y=%.4f box y=%.4f (expect 1.5)\n", ps.y, pb.y);
		Check(Near(ps.y, 1.5f, 0.04f), "sphere rests on the flat heightfield");
		Check(Near(pb.y, 1.5f, 0.04f), "box rests on the flat heightfield");

		// raise a 3x3 plateau (samples 2..4) with a compact region buffer
		const Handle fallsAfter = AddDynamic(world, SphereGeom(0.5f), Physics::Vec3(6.0f, 9.0f, 6.0f));
		UpdateHeightField(field, std::vector<int16_t>(9, 3000), res, 2, 2, 3, 3);
		Simulate(world, 240);
		const Physics::Vec3 pp = Position(world, fallsAfter);
		std::printf("  plateau (region buffer): sphere y=%.4f (expect 3.5)\n", pp.y);
		Check(Near(pp.y, 3.5f, 0.05f), "UpdateHeightFieldRegion with a region buffer is reflected");

		// full grid buffer variant: plateau goes up to 5
		std::vector<int16_t> full = FlatHeights(res, 1000);
		for (int z = 2; z < 5; ++z)
		{
			for (int x = 2; x < 5; ++x)
			{
				full[static_cast<size_t>(z) * res + x] = 5000;
			}
		}
		UpdateHeightField(field, full, res, 2, 2, 3, 3);
		const Handle second = AddDynamic(world, SphereGeom(0.5f), Physics::Vec3(6.0f, 9.0f, 6.0f));
		Simulate(world, 240);
		const Physics::Vec3 p2 = Position(world, second);
		std::printf("  plateau (full grid): sphere y=%.4f (expect 5.5)\n", p2.y);
		Check(Near(p2.y, 5.5f, 0.05f), "UpdateHeightFieldRegion with a full grid is reflected");

		// invalid updates are ignored
		UpdateHeightField(field, std::vector<int16_t>(9, 0), res, 7, 7, 3, 3);
		UpdateHeightField(field, std::vector<int16_t>(5, 0), res, 0, 0, 3, 3);
		UpdateHeightField(reinterpret_cast<Handle>(0x40), std::vector<int16_t>(9, 0), res, 0, 0, 3, 3);
		Check(true, "invalid heightfield updates survived");
		DestroyWorld(world);
		DestroyHeightField(field);

		// slope rising along +x: 1 m per sample, a sphere rolls toward -x and keeps its z
		world = CreateWorld();
		std::vector<int16_t> slope(static_cast<size_t>(res) * res);
		for (int z = 0; z < res; ++z)
		{
			for (int x = 0; x < res; ++x)
			{
				slope[static_cast<size_t>(z) * res + x] = static_cast<int16_t>(x * 1000);
			}
		}
		Handle slopeField = CreateHeightField(slope, res);
		AddStatic(world, TerrainGeom(slopeField, scale));
		Handle roller = AddDynamic(world, SphereGeom(0.5f), Physics::Vec3(10.0f, 6.5f, 7.0f));
		Simulate(world, 90);
		const Physics::Vec3 pr = Position(world, roller);
		std::printf("  slope: sphere x=%.3f y=%.3f z=%.3f (start x=10 y=6.5 z=7)\n", pr.x, pr.y, pr.z);
		Check(pr.x < 8.5f, "sphere rolls down toward -x on the slope");
		Check(Near(pr.z, 7.0f, 0.4f), "sphere keeps its z while rolling down the slope");
		DestroyWorld(world);
		DestroyHeightField(slopeField);

		// cell diagonal: the tess flag is set on every sample (as in the built-in backend). In a single cell with
		// corners (0,0)=0, (1,0)=1000, (0,1)=1000, (1,1)=0 the flag decides which diagonal splits it, so the
		// height at the cell center is 0 m with the flag set (the diagonal joins the two zero corners) and 1 m without.
		world = CreateWorld();
		const std::vector<int16_t> saddle = { 0, 1000, 1000, 0 };
		Handle saddleField = CreateHeightField(saddle, 2);
		Handle saddleShape = nullptr;
		AddStatic(world, TerrainGeom(saddleField, scale), &saddleShape);
		QueryData::RayCastAgainstShape ray{};
		ray.origin = Physics::Vec3(1.0f, 10.0f, 1.0f);
		ray.direction = Physics::Vec3(0.0f, -1.0f, 0.0f);
		ray.filterOptions = Physics::QueryFilterFlags::UseDynamicBodies | Physics::QueryFilterFlags::UseStaticBodies;
		ray.maxDistance = 100.0f;
		Run(saddleShape, Context(world, nullptr), Query::RayCastAgainstShape, ray);
		std::printf("  tess flag: cell center height %.3f\n", 10.0f - ray.hit.distance);
		Check(ray.hitCount == 1 && Near(10.0f - ray.hit.distance, 0.0f, 0.01f), "heightfield cell diagonal follows the tess flag");
		DestroyWorld(world);
		DestroyHeightField(saddleField);

		// holes: cell (x = 1, z = 4) -> world x 2..4, z 8..10 (hole mask is (res-1)^2, z-major)
		world = CreateWorld();
		std::vector<uint8_t> holes(static_cast<size_t>(res - 1) * (res - 1), 1);
		holes[static_cast<size_t>(4) * (res - 1) + 1] = 0;
		Handle holeField = CreateHeightField(FlatHeights(res, 1000), res, &holes);
		AddStatic(world, TerrainGeom(holeField, scale));
		Handle falling = AddDynamic(world, SphereGeom(0.5f), Physics::Vec3(3.0f, 4.0f, 9.0f));
		Handle resting = AddDynamic(world, SphereGeom(0.5f), Physics::Vec3(9.0f, 4.0f, 3.0f));
		Simulate(world, 120);
		std::printf("  hole: sphere over hole y=%.3f, sphere beside it y=%.3f\n", Position(world, falling).y, Position(world, resting).y);
		Check(Position(world, falling).y < -2.0f, "sphere falls through the heightfield hole");
		Check(Near(Position(world, resting).y, 1.5f, 0.05f), "sphere beside the hole is supported");
		DestroyWorld(world);
		DestroyHeightField(holeField);

		// invalid creation
		Check(CreateHeightField(std::vector<int16_t>(4, 0), 1) == nullptr, "resolution 1 rejected");
		Check(CreateHeightField(std::vector<int16_t>(10, 0), 8) == nullptr, "too few samples rejected");
	}

	void TestLifetime()
	{
		std::printf("(f) destruction order\n");
		const MeshData quad = QuadMesh(10.0f);

		// mesh destroyed first, the shape keeps working
		{
			Handle world = CreateWorld();
			Handle mesh = CookMesh(quad, false);
			Handle shape = nullptr;
			Handle body = AddStatic(world, MeshGeom(Physics::GeometryType::TriangleMesh, mesh), &shape);
			DestroyMesh(mesh, false);
			DestroyMesh(mesh, false); // double destroy
			Handle ball = AddDynamic(world, SphereGeom(0.5f), Physics::Vec3(0, 3, 0));
			Simulate(world, 240);
			Check(Near(Position(world, ball).y, 0.5f, 0.05f), "destroyed mesh stays alive for the shape that uses it");
			ShapeData::GetGeometry gg{};
			Run(shape, Context(world, nullptr), Shape::GetGeometry, gg);
			Check(gg.value.type == Physics::GeometryType::TriangleMesh, "GetGeometry on a shape whose mesh was destroyed");
			Check(CreateShape(world, MeshGeom(Physics::GeometryType::TriangleMesh, mesh)) == nullptr, "destroyed mesh handle rejected by CreateShape");
			Check(Extract(shape, Context(world, nullptr)).data.vertexCount == 4, "extract from a shape whose mesh was destroyed");
			DestroyShape(world, body, shape);
			DestroyBody(world, body);
			DestroyWorld(world);
		}

		// world destroyed with live mesh shapes, then shape and mesh
		{
			Handle world = CreateWorld();
			Handle mesh = CookMesh(quad, false);
			Handle shape = nullptr;
			AddStatic(world, MeshGeom(Physics::GeometryType::TriangleMesh, mesh), &shape);
			DestroyWorld(world);
			DestroyShape(world, nullptr, shape);
			DestroyMesh(mesh, false);
			Check(true, "world, shape and mesh destroyed in that order");
		}

		// foreign handles
		{
			Handle fake = reinterpret_cast<Handle>(0x1234);
			DestroyMesh(fake, false);
			DestroyMesh(nullptr, true);
			DestroyHeightField(fake);
			Handle world = CreateWorld();
			Check(CreateShape(world, MeshGeom(Physics::GeometryType::ConvexMesh, fake)) == nullptr, "foreign convex handle rejected");
			Check(CreateShape(world, MeshGeom(Physics::GeometryType::TriangleMesh, nullptr)) == nullptr, "null mesh handle rejected");
			Check(CreateShape(world, TerrainGeom(fake, Physics::Vec3(1, 1, 1))) == nullptr, "foreign heightfield handle rejected");
			// a convex handle must not be accepted as a triangle mesh
			Handle convex = CookMesh(CubeMesh(0.5f), true);
			Check(CreateShape(world, MeshGeom(Physics::GeometryType::TriangleMesh, convex)) == nullptr, "convex handle rejected as triangle mesh");
			DestroyMesh(convex, true);
			DestroyWorld(world);
		}

		// dynamic body: triangle mesh and heightfield shapes cannot simulate there, the body must not crash or log errors
		{
			Handle world = CreateWorld();
			Handle floor = AddStatic(world, BoxGeom(0.5f));
			(void)floor;
			Handle mesh = CookMesh(quad, false);
			Handle field = CreateHeightField(FlatHeights(8, 0), 8);
			Handle body = CreateBody(world, false, Physics::Vec3(0, 5, 0));
			Handle meshShape = CreateShape(world, MeshGeom(Physics::GeometryType::TriangleMesh, mesh));
			Handle fieldShape = CreateShape(world, TerrainGeom(field, Physics::Vec3(1, 1, 1)));
			Handle boxShape = CreateShape(world, BoxGeom(0.5f));
			AttachShape(world, body, meshShape);
			AttachShape(world, body, fieldShape);
			AttachShape(world, body, boxShape);
			Recompute(world, body);
			AddToScene(world, body);
			Simulate(world, 30);
			ShapeData::GetFlags gf{};
			Run(meshShape, Context(world, body), Shape::GetFlags, gf);
			Check((gf.flags & Physics::ShapeFlags::SceneQuery) != Physics::ShapeFlags::None, "mesh shape on a dynamic body stays queryable");

			// kinematic bodies may carry the mesh shapes as simulation shapes
			SetBodyFlag(world, body, Physics::BodyFlags::Kinematic, true);
			SetBodyFlag(world, body, Physics::BodyFlags::Kinematic, false);
			Simulate(world, 10);

			// a trigger flag on a mesh shape is refused without errors
			ShapeData::SetFlags sf{};
			sf.flags = Physics::ShapeFlags::Trigger;
			Run(meshShape, Context(world, body), Shape::SetFlags, sf);
			Simulate(world, 5);

			DestroyMesh(mesh, false);
			DestroyHeightField(field);
			DestroyBody(world, body);
			DestroyShape(world, nullptr, meshShape);
			DestroyShape(world, nullptr, fieldShape);
			DestroyShape(world, nullptr, boxShape);
			DestroyWorld(world);
			Check(true, "dynamic body with mesh and heightfield shapes");
		}

		// heightfield: destroy before the shape, double destroy
		{
			Handle world = CreateWorld();
			Handle field = CreateHeightField(FlatHeights(8, 500), 8);
			Handle shape = nullptr;
			Handle body = AddStatic(world, TerrainGeom(field, Physics::Vec3(1, 0.001f, 1)), &shape);
			DestroyHeightField(field);
			DestroyHeightField(field);
			Simulate(world, 5);
			UpdateHeightField(field, FlatHeights(8, 1), 8, 0, 0, 8, 8); // destroyed handle: ignored
			DestroyShape(world, body, shape);
			DestroyBody(world, body);
			DestroyWorld(world);
			Check(true, "heightfield destroyed before its shape");
		}

		// objects left alive at shutdown are released by the SDK shutdown
		Handle leakedMesh = CookMesh(quad, false);
		Handle leakedField = CreateHeightField(FlatHeights(8, 0), 8);
		Check(leakedMesh != nullptr && leakedField != nullptr, "meshes left for shutdown");
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
	std::printf("mesh_test: backend=%s (requested SDK %d)\n", path.c_str(), FALL_TEST_SDK);

	SdkData::InitializeSdk init;
	init.engineObjectIdOffset = 0;
	Run(nullptr, Context(), SDK::InitializeSdk, init);
	Check(init.result, "InitializeSdk");
	MaterialData::GetDefaultMaterial dm{};
	Run(nullptr, Context(), SDK::GetDefaultMaterial, dm);
	g_material = dm.value;

	{
		Handle world = CreateWorld();
		Handle mesh = nullptr;
		TestTriangleMeshFloor(world, &mesh);
		DestroyMesh(mesh, false);
		DestroyWorld(world);
	}
	{
		Handle world = CreateWorld();
		TestConvex(world);
		DestroyWorld(world);
	}
	TestStreams();
	TestExtract();
	TestHeightField();
	TestLifetime();

	SdkData::ShutdownSdk shutdown;
	Run(nullptr, Context(), SDK::ShutdownSdk, shutdown);
	unload();
	dlclose(lib);

	Check(g_errorLogs == 0, "no backend error logs");
	std::printf("mesh_test: %d checks, %d failures, %d error logs, %d warnings -> %s\n", g_checks, g_failures, g_errorLogs, g_warnLogs,
		g_failures == 0 ? "PASS" : "FAIL");
	return g_failures == 0 ? 0 : 1;
}
