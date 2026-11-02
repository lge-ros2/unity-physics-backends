#pragma once

// Cooking entry points whose PhysX API differs between SDKs: 4.1 owns a PxCooking object, 5.x exposes
// free functions. The implementation lives in physx/v4 or physx/v5 (selected by PHYSX_SDK in CMake).

#include "PxCompat.h"

#include <PxPhysicsAPI.h>

namespace backend::cooking
{
	struct Options
	{
		bool optimizeForRuntime = false; // BVH leaf size: runtime query speed vs. cooking speed
		bool cleanup = false; // remove degenerate / duplicate triangles (keeps the remap table up to date)
		bool weld = false; // merge vertices closer than weldTolerance (implies cleanup)
		float weldTolerance = 1e-4f;
	};

	// Cooking parameters derived from the options; identical field layout in 4.1 and 5.x.
	inline physx::PxCookingParams MakeParams(const physx::PxTolerancesScale& scale, const Options& options)
	{
		physx::PxCookingParams params(scale);
		params.suppressTriangleMeshRemapTable = false; // RemapTriangleIndexToSourceIndex needs it
		params.midphaseDesc = physx::PxMeshMidPhase::eBVH34;
		params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = options.optimizeForRuntime ? 4 : 15;
		params.meshPreprocessParams = physx::PxMeshPreprocessingFlags();
		if (options.weld)
		{
			params.meshPreprocessParams |= physx::PxMeshPreprocessingFlag::eWELD_VERTICES;
			params.meshWeldTolerance = options.weldTolerance;
		}
		else if (!options.cleanup)
		{
			params.meshPreprocessParams |= physx::PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;
		}
		return params;
	}

	// Serialize a cooked mesh into the stream (PhysX binary format of the linked SDK). False on failure.
	bool CookTriangleMesh(const physx::PxTriangleMeshDesc& desc, const Options& options, physx::PxOutputStream& stream);
	bool CookConvexMesh(const physx::PxConvexMeshDesc& desc, const Options& options, physx::PxOutputStream& stream,
		bool* polygonLimitReached);

	// Cooks a heightfield and inserts it into PxPhysics. Null on failure.
	physx::PxHeightField* CreateHeightField(const physx::PxHeightFieldDesc& desc);

	// Releases cooking state (4.1: the PxCooking object). Call before PxPhysics / PxFoundation are released.
	void Shutdown();
}
