#include "physx/common/PxObjects.h"

#include "physx/common/PhysXSdk.h"
#include "physx/common/PxMesh.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <unordered_map>

namespace backend::pxw
{
	namespace
	{
		struct Entry
		{
			Kind kind;
			void* obj;
			std::shared_ptr<WorldState> ws;
		};

		std::mutex g_regMutex;
		std::unordered_map<Handle, Entry> g_reg;

		std::mutex g_defaultMaterialMutex;
		MaterialObj* g_defaultMaterial = nullptr;
		std::atomic<bool> g_improvedPatchFriction{ false };
		std::atomic<uint16_t> g_engineObjectIdOffset{ 0 };
	}

	// ---- registry ----

	void Register(Handle h, Kind kind, void* obj, const std::shared_ptr<WorldState>& ws)
	{
		std::lock_guard<std::mutex> g(g_regMutex);
		g_reg[h] = Entry{ kind, obj, ws };
	}

	void Unregister(Handle h)
	{
		std::lock_guard<std::mutex> g(g_regMutex);
		g_reg.erase(h);
	}

	void Rebind(Handle h, const std::shared_ptr<WorldState>& ws)
	{
		std::lock_guard<std::mutex> g(g_regMutex);
		const auto it = g_reg.find(h);
		if (it != g_reg.end())
		{
			it->second.ws = ws;
		}
	}

	const std::shared_ptr<WorldState>& LooseState()
	{
		static const std::shared_ptr<WorldState> state = std::make_shared<WorldState>();
		return state;
	}

	Handle Pick(Kind kind, Handle c0, Handle c1, Handle c2)
	{
		std::lock_guard<std::mutex> g(g_regMutex);
		for (Handle h : { c0, c1, c2 })
		{
			if (h == nullptr)
			{
				continue;
			}
			const auto it = g_reg.find(h);
			if (it != g_reg.end() && it->second.kind == kind)
			{
				return h;
			}
		}
		return nullptr;
	}

	bool IsKind(Handle h, Kind kind)
	{
		return Pick(kind, h) != nullptr;
	}

	std::vector<Handle> AllHandles(Kind kind)
	{
		std::vector<Handle> out;
		std::lock_guard<std::mutex> g(g_regMutex);
		for (const auto& kv : g_reg)
		{
			if (kv.second.kind == kind)
			{
				out.push_back(kv.first);
			}
		}
		return out;
	}

	void* Lookup(Handle h, Kind kind)
	{
		std::lock_guard<std::mutex> g(g_regMutex);
		const auto it = g_reg.find(h);
		return (it != g_reg.end() && it->second.kind == kind) ? it->second.obj : nullptr;
	}

	// ---- Lock ----

	Lock::Lock(Target t0)
	{
		Acquire(&t0, 1);
	}

	Lock::Lock(Target t0, Target t1)
	{
		const Target targets[2] = { t0, t1 };
		Acquire(targets, 2);
	}

	Lock::~Lock()
	{
		Unlock();
	}

	void Lock::Unlock()
	{
		for (int i = m_lockedCount - 1; i >= 0; --i)
		{
			m_locked[i]->mutex.unlock();
			m_locked[i].reset();
		}
		m_lockedCount = 0;
	}

	void Lock::Acquire(const Target* targets, int count)
	{
		// Optimistic: snapshot the owning domains, lock them, then confirm nothing moved in between.
		for (int attempt = 0; attempt < 64; ++attempt)
		{
			std::shared_ptr<WorldState> ws[2];
			Handle h[2] = { nullptr, nullptr };
			void* obj[2] = { nullptr, nullptr };

			{
				std::lock_guard<std::mutex> g(g_regMutex);
				for (int i = 0; i < count; ++i)
				{
					bool found = false;
					for (Handle cand : { targets[i].c0, targets[i].c1 })
					{
						if (cand == nullptr)
						{
							continue;
						}
						const auto it = g_reg.find(cand);
						if (it != g_reg.end() && it->second.kind == targets[i].kind)
						{
							h[i] = cand;
							obj[i] = it->second.obj;
							ws[i] = it->second.ws;
							found = true;
							break;
						}
					}
					if (!found)
					{
						return;
					}
				}
			}

			WorldState* first = ws[0].get();
			WorldState* second = count > 1 ? ws[1].get() : nullptr;
			std::shared_ptr<WorldState> order[2];
			int orderCount = 0;
			if (second == nullptr || second == first)
			{
				order[orderCount++] = ws[0];
			}
			else if (first < second)
			{
				order[orderCount++] = ws[0];
				order[orderCount++] = ws[1];
			}
			else
			{
				order[orderCount++] = ws[1];
				order[orderCount++] = ws[0];
			}

			for (int i = 0; i < orderCount; ++i)
			{
				order[i]->mutex.lock();
				m_locked[i] = order[i];
				m_lockedCount = i + 1;
			}

			bool missing = false;
			bool moved = false;
			{
				std::lock_guard<std::mutex> g(g_regMutex);
				for (int i = 0; i < count; ++i)
				{
					const auto it = g_reg.find(h[i]);
					if (it == g_reg.end())
					{
						missing = true;
						break;
					}
					if (it->second.ws != ws[i])
					{
						moved = true;
					}
				}
			}

			if (!missing && !moved)
			{
				for (int i = 0; i < count; ++i)
				{
					m_handle[i] = h[i];
					m_obj[i] = obj[i];
				}
				m_ok = true;
				return;
			}

			Unlock();
			if (missing)
			{
				return;
			}
		}
	}

	// ---- conversions ----

	physx::PxTransform ToPx(const Physics::Pose& pose)
	{
		physx::PxQuat q(pose.quat.x, pose.quat.y, pose.quat.z, pose.quat.w);
		const float mag = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
		if (!(mag > 1e-6f) || !std::isfinite(mag))
		{
			q = physx::PxQuat(physx::PxIdentity);
		}
		else
		{
			q = physx::PxQuat(q.x / mag, q.y / mag, q.z / mag, q.w / mag);
		}
		return physx::PxTransform(physx::PxVec3(pose.pos.x, pose.pos.y, pose.pos.z), q);
	}

	Physics::Pose FromPx(const physx::PxTransform& t)
	{
		return Physics::Pose(t.p.x, t.p.y, t.p.z, t.q.x, t.q.y, t.q.z, t.q.w);
	}

	// ---- destruction (caller holds the owning domain) ----

	namespace
	{
		void DetachFromOwner(ShapeObj* shape)
		{
			BodyObj* owner = shape->owner;
			if (owner == nullptr)
			{
				return;
			}
			owner->actor->detachShape(*shape->shape);
			owner->shapes.erase(std::remove(owner->shapes.begin(), owner->shapes.end(), shape), owner->shapes.end());
			shape->owner = nullptr;
			Rebind(shape, LooseState());
		}
	}

	void DestroyShapeLocked(Handle h, ShapeObj* shape)
	{
		DetachFromOwner(shape);
		shape->shape->release();
		ReleaseShapeGeometryRefs(*shape);
		Unregister(h);
		delete shape;
	}

	void DestroyBodyLocked(Handle h, BodyObj* body)
	{
		// Releasing the actor removes it from the scene and detaches its shapes.
		body->actor->release();
		for (ShapeObj* shape : body->shapes)
		{
			shape->owner = nullptr;
			Rebind(shape, LooseState());
		}
		body->shapes.clear();

		if (body->world != nullptr)
		{
			auto& list = body->world->bodies;
			list.erase(std::remove(list.begin(), list.end(), body), list.end());
		}
		Unregister(h);
		delete body;
	}

	void DestroyWorldLocked(Handle h, WorldObj* world)
	{
		const std::vector<BodyObj*> bodies = world->bodies;
		for (BodyObj* body : bodies)
		{
			DestroyBodyLocked(body, body);
		}
		if (world->scene != nullptr)
		{
			world->scene->release();
		}
		if (world->dispatcher != nullptr)
		{
			world->dispatcher->release();
		}
		Unregister(h);
		delete world;
	}

	// ---- globals ----

	MaterialObj* DefaultMaterial()
	{
		std::lock_guard<std::mutex> g(g_defaultMaterialMutex);
		if (g_defaultMaterial == nullptr)
		{
			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (physics == nullptr)
			{
				return nullptr;
			}
			auto* m = new MaterialObj();
			m->material = physics->createMaterial(0.6f, 0.6f, 0.0f);
			if (m->material == nullptr)
			{
				delete m;
				return nullptr;
			}
			compat::ApplyImprovedPatchFriction(*m->material, g_improvedPatchFriction.load());
			Register(m, Kind::Material, m, LooseState());
			g_defaultMaterial = m;
		}
		return g_defaultMaterial;
	}

	void ReleaseAllMaterials()
	{
		std::lock_guard<std::mutex> g(g_defaultMaterialMutex);
		for (Handle h : AllHandles(Kind::Material))
		{
			auto* m = static_cast<MaterialObj*>(Lookup(h, Kind::Material));
			Unregister(h);
			if (m != nullptr)
			{
				m->material->release();
				delete m;
			}
		}
		g_defaultMaterial = nullptr;
	}

	bool ImprovedPatchFrictionEnabled()
	{
		return g_improvedPatchFriction.load();
	}

	void SetImprovedPatchFrictionEnabled(bool enabled)
	{
		g_improvedPatchFriction.store(enabled);
	}

	uint16_t EngineObjectIdOffset()
	{
		return g_engineObjectIdOffset.load();
	}

	void SetEngineObjectIdOffset(uint16_t offset)
	{
		g_engineObjectIdOffset.store(offset);
	}
}
