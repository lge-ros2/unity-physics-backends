#pragma once

// Collision mesh / heightfield objects behind the Unity mesh handles. A handle is the address of a
// MeshObj / HeightFieldObj, registered in the handle registry (Kind::Mesh / Kind::HeightField).
//
// Lifetime: shapes pin the objects they reference. DestroyCollisionMesh / DestroyHeightField only mark an
// object destroyed; the PhysX object is released once the last shape lets go (so a destroyed mesh stays
// valid for the shapes that still use it, and the handle address is never reused while they do).
// All reference bookkeeping is guarded by one mutex that is never held while a world lock is taken.

#include "physx/common/PxObjects.h"

#include <vector>

namespace backend::pxw
{
	struct MeshObj
	{
		physx::PxConvexMesh* convex = nullptr;
		physx::PxTriangleMesh* triangle = nullptr;
		uint32_t refs = 0;
		bool destroyed = false;
	};

	struct HeightFieldObj
	{
		physx::PxHeightField* field = nullptr;
		uint32_t resolution = 0;
		uint32_t refs = 0;
		bool destroyed = false;
		std::vector<ShapeObj*> shapes; // shapes that use the field (their geometry is refreshed after an update)
	};

	// Pins a live (not destroyed) mesh of the requested geometry type. Null for stale, foreign or destroyed handles.
	MeshObj* AcquireMesh(Handle h, Physics::GeometryType type);
	HeightFieldObj* AcquireHeightField(Handle h);
	// Mesh behind a handle, destroyed-but-still-referenced included. Does not pin.
	MeshObj* FindMesh(Handle h);
	HeightFieldObj* FindHeightField(Handle h);

	void ReleaseMesh(MeshObj* mesh);
	void LinkHeightFieldShape(HeightFieldObj* field, ShapeObj* shape);
	void ReleaseHeightField(HeightFieldObj* field, ShapeObj* shape);
	// Drops the mesh / heightfield references held by a shape.
	void ReleaseShapeGeometryRefs(ShapeObj& shape);

	// Frees every remaining mesh and heightfield. Must run after all shapes are gone and before PxPhysics is released.
	void ReleaseAllMeshes();

	void RegisterMeshHandlers();
}
