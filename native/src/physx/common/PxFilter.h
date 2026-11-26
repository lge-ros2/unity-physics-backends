#pragma once

#include "physx/common/PxObjects.h"

namespace backend::pxw
{
	// Physics::FilterData <-> PxFilterData (the 16 byte layout maps 1:1 onto word0..word3).
	physx::PxFilterData EncodeFilter(const Physics::FilterData& data);
	Physics::FilterData DecodeFilter(const physx::PxFilterData& data);

	// Simulation filter data of a shape: its FilterData with the pair flags of Shape::SetSupportedMessages merged in.
	// Shapes on articulation links additionally carry the articulation's collision group.
	physx::PxFilterData EncodeShapeFilter(const ShapeObj& shape);
	// Nonzero group id for a new articulation.
	// Returns 0 (no group) only when all 65535 ids are live.
	uint16_t NextArticulationCollisionGroup();
	// Gives the id of a destroyed articulation back (call from the articulation destroy path; 0 and unknown ids are ignored).
	void ReleaseArticulationCollisionGroup(uint16_t group);
	// Pushes EncodeShapeFilter onto the PxShape; re-filters the shape's pairs when the pair flags changed.
	void UpdateShapeSimulationFilter(ShapeObj& shape);

	// Layer rule shared by the simulation filter shader.
	bool LayersCollide(const Physics::FilterData& a, const Physics::FilterData& b);

	// PxSimulationFilterShader implementation.
	physx::PxFilterFlags SimulationFilterShader(
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize);

	// Physics.IgnoreCollision pairs (global, keyed by the collider ignore ids).
	bool IsPairIgnored(uint16_t id0, uint16_t id1);
	void SetPairIgnored(uint16_t id0, uint16_t id1, bool ignored);
	// Removes every pair that references id. Returns true when something was removed.
	bool CleanupIgnoredPairs(uint16_t id);
	void ClearIgnoredPairs();
}
