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
		Material
	};

	// One lock domain: a world, or the "loose" domain for shapes and materials that belong to no world.
	struct WorldState
	{
		std::mutex mutex;
	};

	struct BodyObj;
	struct ShapeObj;

	struct WorldObj
	{
		physx::PxScene* scene = nullptr;
		physx::PxDefaultCpuDispatcher* dispatcher = nullptr;
		std::shared_ptr<WorldState> state;
		void* userData = nullptr;
		uint32_t scratchChunkCount = 0;
		std::vector<BodyObj*> bodies;
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
		float maxLinearVelocity = 100.0f;
		float maxAngularVelocity = 7.0f;
		uint32_t positionIterations = 6;
		uint32_t velocityIterations = 1;
		Physics::Pose kinematicTarget;
		std::vector<ShapeObj*> shapes;
	};

	struct ShapeObj
	{
		physx::PxShape* shape = nullptr;
		BodyObj* owner = nullptr;
		void* userData = nullptr;
		Physics::ShapeGeometry geometry = {};
		Physics::ShapeFlags flags = Physics::ShapeFlags::None;
		Physics::FilterData filter;
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

	// ---- shared helpers implemented across the module ----

	// Handlers that own the destruction order; callers hold the owning domain.
	void DestroyShapeLocked(Handle h, ShapeObj* shape);
	void DestroyBodyLocked(Handle h, BodyObj* body);
	void DestroyWorldLocked(Handle h, WorldObj* world);

	// Pushes BodyFlags (gravity / kinematic / simulated) and the CCD mode onto the PhysX actor.
	void ApplyBodyFlags(BodyObj& body, Physics::BodyFlags flags);

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
}
