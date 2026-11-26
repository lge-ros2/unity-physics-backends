#include "physx/common/PxFilter.h"
#include "physx/common/PxArticulation.h"

#include <atomic>
#include <cstring>
#include <set>
#include <shared_mutex>
#include <utility>

namespace backend::pxw
{
	static_assert(sizeof(Physics::FilterData) == sizeof(physx::PxFilterData), "FilterData must map onto PxFilterData");
	static_assert(sizeof(physx::PxFilterData) == 16, "PxFilterData is expected to be 4 x 32 bit");

	namespace
	{
		std::shared_mutex g_ignoreMutex;
		std::set<std::pair<uint16_t, uint16_t>> g_ignored;
		std::atomic<size_t> g_ignoredCount{ 0 };

		std::pair<uint16_t, uint16_t> MakeKey(uint16_t a, uint16_t b)
		{
			return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
		}
	}

	physx::PxFilterData EncodeFilter(const Physics::FilterData& data)
	{
		physx::PxFilterData out;
		std::memcpy(static_cast<void*>(&out), &data, sizeof(out));
		return out;
	}

	Physics::FilterData DecodeFilter(const physx::PxFilterData& data)
	{
		Physics::FilterData out;
		std::memcpy(static_cast<void*>(&out), &data, sizeof(out));
		return out;
	}

	bool LayersCollide(const Physics::FilterData& a, const Physics::FilterData& b)
	{
		const bool aSeesB = (a.collidesWith & b.belongsTo) != 0;
		const bool bSeesA = (b.collidesWith & a.belongsTo) != 0;

		// Layer override: the shape with the higher priority decides alone; a shape without an
		// override ranks below any override. Equal rank: both sides must agree.
		// TODO: verify against Unity's Include/Exclude Layers semantics (priority ties, exclude-wins).
		const int rankA = a.hasOverride ? static_cast<int>(a.layerOverridePriority) : -1;
		const int rankB = b.hasOverride ? static_cast<int>(b.layerOverridePriority) : -1;
		if (rankA > rankB)
		{
			return aSeesB;
		}
		if (rankB > rankA)
		{
			return bSeesA;
		}
		return aSeesB && bSeesA;
	}

	physx::PxFilterFlags SimulationFilterShader(
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		physx::PxPairFlags& pairFlags, const void*, physx::PxU32)
	{
		const Physics::FilterData f0 = DecodeFilter(filterData0);
		const Physics::FilterData f1 = DecodeFilter(filterData1);

		if (f0.hasCollisionDisabled || f1.hasCollisionDisabled)
		{
			return physx::PxFilterFlag::eSUPPRESS;
		}
		if (!LayersCollide(f0, f1))
		{
			return physx::PxFilterFlag::eSUPPRESS;
		}
		// padding marks a link shape whose ignorePairID slot holds the articulation collision group instead.
		const bool group0 = f0.padding != 0;
		const bool group1 = f1.padding != 0;
		if (group0 && group1 && f0.ignorePairID == f1.ignorePairID)
		{
			return physx::PxFilterFlag::eSUPPRESS;
		}
		if (!group0 && !group1
			&& f0.ignorePairID != Physics::FilterData::invalidIgnorePairID
			&& f1.ignorePairID != Physics::FilterData::invalidIgnorePairID
			&& IsPairIgnored(f0.ignorePairID, f1.ignorePairID))
		{
			return physx::PxFilterFlag::eSUPPRESS;
		}

		if (physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
			return physx::PxFilterFlag::eDEFAULT;
		}

		// ShapePairEventFlags maps bit for bit onto PxPairFlag. Physics (solve + detect) always stays on;
		// the notify / velocity bits come from either shape. Contact modification is not supported.
		constexpr uint16_t kUnsupported = static_cast<uint16_t>(Physics::ShapePairEventFlags::ModifyContacts)
			| static_cast<uint16_t>(Physics::ShapePairEventFlags::NextFree);
		const uint16_t requested = (static_cast<uint16_t>(f0.pairFlags) | static_cast<uint16_t>(f1.pairFlags)) & ~kUnsupported;
		const physx::PxPairFlags notify(requested);
		pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT | physx::PxPairFlag::eDETECT_CCD_CONTACT | notify;
		if (notify & (physx::PxPairFlag::eNOTIFY_TOUCH_FOUND | physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS))
		{
			// Pre-solver velocities give the engine the relative velocity of the impact.
			pairFlags |= physx::PxPairFlag::ePRE_SOLVER_VELOCITY;
		}
		return physx::PxFilterFlag::eDEFAULT;
	}

	physx::PxFilterData EncodeShapeFilter(const ShapeObj& shape)
	{
		Physics::FilterData data = shape.filter;
		uint16_t supported = 0;
		for (uint16_t flags : shape.supportedByLayer)
		{
			supported |= flags;
		}
		data.pairFlags = static_cast<Physics::ShapePairEventFlags>(static_cast<uint16_t>(data.pairFlags) | supported);
		// Unity's Physics.IgnoreCollision reaches custom backends with ignore ids that are always 0, so CLOiSim-style
		// "ignore every pair inside a model" cannot be expressed per pair; links of one articulation never collide instead.
		// FilterData() leaves padding uninitialized, so it is always written here.
		data.padding = 0;
		if (shape.ownerLink != nullptr && shape.ownerLink->art != nullptr && shape.ownerLink->art->collisionGroup != 0)
		{
			data.padding = 1;
			data.ignorePairID = shape.ownerLink->art->collisionGroup;
		}
		return EncodeFilter(data);
	}

	void UpdateShapeSimulationFilter(ShapeObj& shape)
	{
		const physx::PxFilterData oldData = shape.shape->getSimulationFilterData();
		const physx::PxFilterData newData = EncodeShapeFilter(shape);
		if (std::memcmp(&oldData, &newData, sizeof(newData)) == 0)
		{
			return;
		}
		shape.shape->setSimulationFilterData(newData);

		// Existing pairs keep their old pair flags until they are filtered again.
		const bool pairFlagsChanged = DecodeFilter(oldData).pairFlags != DecodeFilter(newData).pairFlags;
		const BodyObj* owner = shape.owner;
		if (pairFlagsChanged && owner != nullptr && owner->inScene && owner->world != nullptr && owner->world->scene != nullptr)
		{
			physx::PxShape* px = shape.shape;
			owner->world->scene->resetFiltering(*owner->actor, &px, 1);
		}
	}

	bool IsPairIgnored(uint16_t id0, uint16_t id1)
	{
		if (g_ignoredCount.load(std::memory_order_relaxed) == 0)
		{
			return false;
		}
		std::shared_lock<std::shared_mutex> lock(g_ignoreMutex);
		return g_ignored.count(MakeKey(id0, id1)) != 0;
	}

	void SetPairIgnored(uint16_t id0, uint16_t id1, bool ignored)
	{
		std::unique_lock<std::shared_mutex> lock(g_ignoreMutex);
		if (ignored)
		{
			g_ignored.insert(MakeKey(id0, id1));
		}
		else
		{
			g_ignored.erase(MakeKey(id0, id1));
		}
		g_ignoredCount.store(g_ignored.size(), std::memory_order_relaxed);
	}

	bool CleanupIgnoredPairs(uint16_t id)
	{
		std::unique_lock<std::shared_mutex> lock(g_ignoreMutex);
		bool removed = false;
		for (auto it = g_ignored.begin(); it != g_ignored.end();)
		{
			if (it->first == id || it->second == id)
			{
				it = g_ignored.erase(it);
				removed = true;
			}
			else
			{
				++it;
			}
		}
		g_ignoredCount.store(g_ignored.size(), std::memory_order_relaxed);
		return removed;
	}

	uint16_t NextArticulationCollisionGroup()
	{
		static std::atomic<uint32_t> next{ 0 };
		uint16_t group = 0;
		while (group == 0)
		{
			group = static_cast<uint16_t>(next.fetch_add(1, std::memory_order_relaxed) + 1);
		}
		return group;
	}

	void ClearIgnoredPairs()
	{
		std::unique_lock<std::shared_mutex> lock(g_ignoreMutex);
		g_ignored.clear();
		g_ignoredCount.store(0, std::memory_order_relaxed);
	}
}
