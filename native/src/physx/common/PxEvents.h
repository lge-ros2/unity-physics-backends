#pragma once

// Simulation events: contact pairs, trigger enter/exit and joint breaks of the last step.
// Everything here runs under the owning world's lock domain (callbacks fire inside fetchResults of Simulate).

#include "physx/common/PxObjects.h"

namespace backend::pxw
{
	// Creates the per-world event collector and installs it as the scene's PxSimulationEventCallback.
	void AttachWorldEvents(WorldObj& world);
	// Frees the collector (before the scene is released).
	void DetachWorldEvents(WorldObj& world);

	// Called right after PxScene::fetchResults: the removed-object id tables are only valid for one step.
	void FinishStepEvents(WorldObj& world);

	// Removal hooks (before the PhysX object is detached / released): PhysX reports pairs lost through
	// removal with dangling pointers, so the engine ids are captured here to report the Exit / lost touch.
	void NotifyShapeRemoved(const ShapeObj& shape);
	void NotifyBodyRemoved(const BodyObj& body);

	// Same for objects that are not Body / Shape handles (articulation links): call before the PhysX object is
	// detached or released while it is in the scene. engineUserData is the user data given at creation.
	void NotifyPxShapeRemoved(WorldObj& world, const physx::PxShape* shape, Handle handle, const void* engineUserData);
	void NotifyPxActorRemoved(WorldObj& world, const physx::PxActor* actor, Handle handle, const void* engineUserData);

	// Entry for the joint implementation: queues a break event for the next GetLastSimulationStepResults.
	void ReportJointBreak(WorldObj& world, const Physics::JointBreakEvent& event);

	void RegisterEventHandlers();
}
