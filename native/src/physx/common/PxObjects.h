#pragma once

// Handle objects backing the Unity SDKObjectHandle values of the PhysX backend, plus the handle registry.
// A handle is the address of one of the structs below; it is only dereferenced after the registry
// confirmed it, so foreign handles (e.g. stub handles) never crash.

#include "PhysicsCommands.h"
#include "PhysicsFilterData.h"
#include "PhysicsMaterialCommandData.h"
#include "PhysicsPose.h"
#include "PhysicsShapeTypes.h"
#include "PhysicsBodyTypes.h"

#include "PxCompat.h"

#include <PxPhysicsAPI.h>

#include <memory>
#include <mutex>
#include <vector>

namespace backend::pxw
{
	using Handle = Physics::SDKObjectHandle;
	using Context = PhysicsCommands::Context;
	using Command = PhysicsCommands::Command;

	enum class Kind : uint8_t
	{
		World,
		Body,
		Shape,
		Material,
		Mesh,
		HeightField,
		Articulation,
		Link,
		LinkJoint,
		Joint
	};

	// One lock domain: a world, or the "loose" domain for shapes and materials that belong to no world.
	struct WorldState
	{
		std::mutex mutex;
	};

	struct BodyObj;
	struct LinkObj;
	struct ArticulationObj;
	struct ShapeObj;
	struct MeshObj;
	struct HeightFieldObj;
	struct WorldEvents;
	struct JointObj;

	struct WorldObj
	{
		physx::PxScene* scene = nullptr;
		physx::PxDefaultCpuDispatcher* dispatcher = nullptr;
		std::shared_ptr<WorldState> state;
		void* userData = nullptr;
		uint32_t scratchChunkCount = 0;
		WorldEvents* events = nullptr; // PxEvents.cpp
		std::vector<BodyObj*> bodies;
		std::vector<ArticulationObj*> articulations;
		std::vector<JointObj*> joints;
	};

	struct BodyObj
	{
		physx::PxRigidActor* actor = nullptr;
		physx::PxRigidDynamic* dynamic = nullptr; // null for static bodies
		WorldObj* world = nullptr;
		void* userData = nullptr;
		bool isStatic = false;
		bool inScene = false;
		Physics::BodyFlags flags = Physics::BodyFlags::None;
		Physics::CollisionDetectionMode ccdMode = Physics::CollisionDetectionMode::Discrete;
		float mass = 1.0f;
		float linearDamping = 0.0f;
		float angularDamping = 0.05f;
		float sleepThreshold = 0.005f;
		float maxDepenetrationVelocity = 10.0f;
		float maxLinearVelocity = 1e16f; // PhysX default; refreshed from the actor on creation
		float maxAngularVelocity = 7.0f;
		uint32_t positionIterations = 6;
		uint32_t velocityIterations = 1;
		Physics::Pose kinematicTarget;
		std::vector<ShapeObj*> shapes;
		std::vector<JointObj*> joints; // joints attached to this body
	};

	struct ShapeObj
	{
		physx::PxShape* shape = nullptr;
		BodyObj* owner = nullptr;
		LinkObj* ownerLink = nullptr; // set instead of owner while attached to an articulation link
		void* userData = nullptr;
		Physics::ShapeGeometry geometry = {};
		MeshObj* mesh = nullptr; // pinned convex / triangle mesh (geometry.meshGeometry.mesh)
		HeightFieldObj* heightField = nullptr; // pinned heightfield (geometry.heightFieldGeometry.heightField)
		Physics::ShapeFlags flags = Physics::ShapeFlags::None;
		Physics::FilterData filter;
		uint16_t supportedByLayer[32] = {}; // Shape::SetSupportedMessages: ShapePairEventFlags per layer
	};

	struct MaterialObj
	{
		physx::PxMaterial* material = nullptr;
		void* userData = nullptr;
		Physics::MaterialCombineMode frictionCombine = Physics::MaterialCombineMode::Average;
		Physics::MaterialCombineMode bounceCombine = Physics::MaterialCombineMode::Average;
	};

	// ---- registry ----

	// Registers a handle. ws is the lock domain the object currently lives in.
	void Register(Handle h, Kind kind, void* obj, const std::shared_ptr<WorldState>& ws);
	void Unregister(Handle h);
	// Moves a handle to another lock domain (shape attach/detach). Caller holds both domains.
	void Rebind(Handle h, const std::shared_ptr<WorldState>& ws);

	const std::shared_ptr<WorldState>& LooseState();

	// First candidate that is registered with the given kind (null if none). Does not lock the object.
	Handle Pick(Kind kind, Handle c0, Handle c1 = nullptr, Handle c2 = nullptr);
	bool IsKind(Handle h, Kind kind);
	std::vector<Handle> AllHandles(Kind kind);
	// Object pointer of a registered handle, without locking (only valid while the caller holds its domain).
	void* Lookup(Handle h, Kind kind);

	struct Target
	{
		Kind kind;
		Handle c0;
		Handle c1 = nullptr;
	};

	// Resolves one or two targets and locks their domains (ordered by address, so concurrent
	// multi-target locks cannot deadlock). Not reentrant: never open a Lock while holding one.
	class Lock
	{
	public:
		explicit Lock(Target t0);
		Lock(Target t0, Target t1);
		~Lock();
		Lock(const Lock&) = delete;
		Lock& operator=(const Lock&) = delete;

		explicit operator bool() const { return m_ok; }
		Handle handle(int i = 0) const { return m_handle[i]; }
		template <typename T>
		T* get(int i = 0) const { return static_cast<T*>(m_obj[i]); }

	private:
		void Acquire(const Target* targets, int count);
		void Unlock();

		bool m_ok = false;
		Handle m_handle[2] = { nullptr, nullptr };
		void* m_obj[2] = { nullptr, nullptr };
		std::shared_ptr<WorldState> m_locked[2];
		int m_lockedCount = 0;
	};

	// ---- conversions ----

	physx::PxTransform ToPx(const Physics::Pose& pose);
	Physics::Pose FromPx(const physx::PxTransform& t);
	inline physx::PxVec3 ToPx(const Physics::Vec3& v) { return physx::PxVec3(v.x, v.y, v.z); }
	inline Physics::Vec3 FromPx(const physx::PxVec3& v) { return Physics::Vec3(v.x, v.y, v.z); }

	// Unity hands shapes of static bodies a world pose (unless forceLocalPose); PhysX wants it relative to the actor.
	inline physx::PxTransform ToLocalShapePose(const BodyObj* owner, const Physics::Pose& pose, bool forceLocal)
	{
		const physx::PxTransform p = ToPx(pose);
		return (owner != nullptr && owner->isStatic && !forceLocal) ? owner->actor->getGlobalPose().transformInv(p) : p;
	}

	// ---- shared helpers implemented across the module ----

	// Handlers that own the destruction order; callers hold the owning domain.
	void DestroyShapeLocked(Handle h, ShapeObj* shape);
	void DestroyBodyLocked(Handle h, BodyObj* body);
	void DestroyWorldLocked(Handle h, WorldObj* world);

	// Pushes BodyFlags (gravity / kinematic / simulated) and the CCD mode onto the PhysX actor.
	void ApplyBodyFlags(BodyObj& body, Physics::BodyFlags flags);

	// Triangle mesh / heightfield shapes can only simulate on static or kinematic bodies; elsewhere they stay query-only.
	// Re-evaluates the shape's PhysX flags as if it were owned by owner (null = loose shape).
	void ReapplyShapeFlags(ShapeObj& shape, const BodyObj* owner);
	// Same for a shape that is (about to be) attached to an articulation link (links are never kinematic).
	void ReapplyShapeFlagsForLink(ShapeObj& shape);

	// Articulation module (PxArticulation.cpp). Callers hold the owning world domain.
	void DestroyArticulationLocked(Handle h, ArticulationObj* articulation);
	// Detaches a shape from its articulation link (shape becomes loose).
	void DetachShapeFromLinkLocked(ShapeObj* shape);
	// Adds a registered articulation handle to its world's scene. False if h is not an articulation of that world.
	bool AddArticulationToSceneLocked(WorldObj* world, Handle h);
	// Appends the user data of every articulation-link shape whose layer matches the mask.
	void CollectLinkShapesUserData(const WorldObj& world, uint32_t layerMask, std::vector<void*>& out);
	// Re-evaluates cached filter decisions of articulation links carrying one of the ignore ids.
	void RefilterLinkShapes(WorldObj& world, uint16_t id0, uint16_t id1);
	// After every step: advances the wrap offsets of the joint positions (see LinkObj::unwrapOffset).
	void UpdateJointUnwrap(WorldObj& world);
	// Handle of the actor a shape is attached to (body or link), null when loose.
	Handle ShapeOwnerHandle(const ShapeObj& shape);
	// Engine user data of a registered link handle (does not lock a domain).
	void* LinkUserData(Handle link);
	// Re-evaluates the mesh-restricted shapes of a body that is (about to be) kinematic or dynamic.
	void UpdateMeshShapeSimulation(BodyObj& body, bool kinematic);
	// Pushes the shape's stored geometry into PhysX again (after a heightfield was modified).
	void RefreshShapeGeometry(ShapeObj& shape);

	MaterialObj* DefaultMaterial();
	bool ImprovedPatchFrictionEnabled();
	void SetImprovedPatchFrictionEnabled(bool enabled);
	uint16_t EngineObjectIdOffset();
	void SetEngineObjectIdOffset(uint16_t offset);
	void ReleaseAllMaterials();

	template <typename T>
	void ZeroCommand(Command& c)
	{
		static_cast<T&>(c) = T{};
	}

	// Per-module registration (called from RegisterPhysXWorldHandlers).
	void RegisterSdkHandlers();
	void RegisterWorldHandlers();
	void RegisterBodyHandlers();
	void RegisterShapeHandlers();
	void RegisterQueryHandlers();
	void RegisterArticulationHandlers();
}
