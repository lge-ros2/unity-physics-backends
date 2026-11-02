#include "core/Dispatch.h"
#include "physx/common/PxFilter.h"
#include "physx/common/PxObjects.h"

#include "PhysicsQueryCommandData.h"

#include <cmath>
#include <cstring>

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace qd = PhysicsCommands::QueryData;

	namespace
	{
		// Layer mask + trigger gate; runs before the exact intersection test.
		class RayFilter : public compat::QueryFilterCallbackBase
		{
		public:
			RayFilter(uint32_t mask, bool useTriggers)
				: m_mask(mask), m_useTriggers(useTriggers)
			{
			}

			physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData&, const physx::PxShape* shape,
				const physx::PxRigidActor*, physx::PxHitFlags&) override
			{
				if (shape == nullptr)
				{
					return physx::PxQueryHitType::eNONE;
				}
				if (!m_useTriggers && (shape->getFlags() & physx::PxShapeFlag::eTRIGGER_SHAPE))
				{
					return physx::PxQueryHitType::eNONE;
				}
				const Physics::FilterData filter = DecodeFilter(shape->getQueryFilterData());
				return (filter.belongsTo & m_mask) != 0 ? physx::PxQueryHitType::eBLOCK : physx::PxQueryHitType::eNONE;
			}

		private:
			uint32_t m_mask;
			bool m_useTriggers;
		};

		// userData of a shape points at the engine object whose EngineObjectId sits at a fixed offset.
		Physics::EngineObjectId ReadEngineObjectId(const physx::PxShape* shape)
		{
			Physics::EngineObjectId id;
			const auto* obj = shape != nullptr ? static_cast<const ShapeObj*>(shape->userData) : nullptr;
			if (obj != nullptr && obj->userData != nullptr)
			{
				std::memcpy(&id, static_cast<const uint8_t*>(obj->userData) + EngineObjectIdOffset(), sizeof(id));
			}
			return id;
		}

		void RayCast(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::RayCast&>(c);
			cmd.hit = Physics::QueryHit::Invalid();
			cmd.hitCount = 0;

			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock)
			{
				return;
			}

			using Opt = Physics::QueryFilterFlags;
			const bool dynamic = (cmd.filter.options & Opt::UseDynamicBodies) != Opt::None;
			const bool statics = (cmd.filter.options & Opt::UseStaticBodies) != Opt::None;
			const bool triggers = (cmd.filter.options & Opt::UseTriggerShapes) != Opt::None;
			if (!dynamic && !statics)
			{
				return;
			}

			physx::PxVec3 dir = ToPx(cmd.direction);
			const float length = dir.magnitude();
			if (!(length > 1e-6f) || !(cmd.maxDistance > 0.0f) || !std::isfinite(cmd.maxDistance))
			{
				return;
			}
			dir *= 1.0f / length;

			physx::PxQueryFilterData filterData;
			filterData.flags = physx::PxQueryFlag::ePREFILTER;
			if (dynamic)
			{
				filterData.flags |= physx::PxQueryFlag::eDYNAMIC;
			}
			if (statics)
			{
				filterData.flags |= physx::PxQueryFlag::eSTATIC;
			}

			RayFilter filter(static_cast<uint32_t>(cmd.filter.mask), triggers);
			physx::PxRaycastBuffer buffer;
			const physx::PxHitFlags hitFlags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL
				| physx::PxHitFlag::eFACE_INDEX | physx::PxHitFlag::eUV;
			if (!lock.get<WorldObj>()->scene->raycast(ToPx(cmd.origin), dir, cmd.maxDistance, buffer, hitFlags, filterData, &filter)
				|| !buffer.hasBlock)
			{
				return;
			}

			const physx::PxRaycastHit& hit = buffer.block;
			cmd.hit.point = FromPx(hit.position);
			cmd.hit.normal = FromPx(hit.normal);
			cmd.hit.faceID = hit.faceIndex;
			cmd.hit.distance = hit.distance;
			cmd.hit.uv[0] = hit.u;
			cmd.hit.uv[1] = hit.v;
			cmd.hit.userData = ReadEngineObjectId(hit.shape);
			cmd.hitCount = 1;
		}
	}

	void RegisterQueryHandlers()
	{
		Bind(pc::Query::RayCast, &RayCast);
	}
}
