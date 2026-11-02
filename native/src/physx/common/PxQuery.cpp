#include "core/Dispatch.h"
#include "physx/common/PxFilter.h"
#include "physx/common/PxObjects.h"

#include "PhysicsQueryCommandData.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>

// Scene queries. Every handler gathers its hits while the world domain is locked, converts them into
// plain data, releases the lock and only then calls back into the caller (collector callbacks may issue
// further backend commands, and Lock is not reentrant).

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace qd = PhysicsCommands::QueryData;

	namespace
	{
		using Opt = Physics::QueryFilterFlags;

		constexpr float kMinExtent = 1e-4f;
		constexpr float kMaxDistance = 1e7f; // Mathf.Infinity is clamped to this
		constexpr physx::PxU32 kTouchBatch = 64;

		bool Has(Opt value, Opt flag)
		{
			return (value & flag) != Opt::None;
		}

		float Clamp(float v)
		{
			return std::isfinite(v) ? std::max(v, kMinExtent) : kMinExtent;
		}

		void WarnUnsupportedGeometry(int type)
		{
			static std::atomic<uint32_t> s_warned{ 0 };
			const uint32_t bit = 1u << (static_cast<uint32_t>(type) & 31u);
			if ((s_warned.fetch_or(bit) & bit) == 0)
			{
				BACKEND_WARNF("Query with geometry type %d is not supported by the PhysX backend yet; reported as no hit", type);
			}
		}

		// Query shapes given by value (Overlap / ShapeCast): box, sphere and capsule only.
		struct QueryGeometry
		{
			physx::PxBoxGeometry box;
			physx::PxSphereGeometry sphere;
			physx::PxCapsuleGeometry capsule;

			// Null (with a one-time warning) for unsupported geometry types.
			const physx::PxGeometry* Build(const Physics::ShapeGeometry& g)
			{
				switch (g.type)
				{
				case Physics::GeometryType::Box:
					box = physx::PxBoxGeometry(Clamp(g.boxGeometry.halfExtents.x), Clamp(g.boxGeometry.halfExtents.y),
						Clamp(g.boxGeometry.halfExtents.z));
					return &box;
				case Physics::GeometryType::Sphere:
					sphere = physx::PxSphereGeometry(Clamp(g.sphereGeometry.radius));
					return &sphere;
				case Physics::GeometryType::Capsule:
				{
					const float radius = Clamp(g.capsuleGeometry.radius);
					const float halfHeight = Clamp((g.capsuleGeometry.height - 2.0f * radius) * 0.5f);
					capsule = physx::PxCapsuleGeometry(radius, halfHeight);
					return &capsule;
				}
				default:
					WarnUnsupportedGeometry(static_cast<int>(g.type));
					return nullptr;
				}
			}
		};

		// Decoded Physics::QueryFilterData / QueryFilterFlags.
		struct QueryParams
		{
			QueryParams(Opt options, uint32_t layerMask)
				: mask(layerMask)
				, dynamic(Has(options, Opt::UseDynamicBodies))
				, statics(Has(options, Opt::UseStaticBodies))
				, triggers(Has(options, Opt::UseTriggerShapes))
				, backfaces(Has(options, Opt::AllowTriangleMeshBackfaceHits))
				, multipleMeshHits(Has(options, Opt::AllowMultipleTriangleMeshHits))
				, earlyOut(Has(options, Opt::AllowEarlyOutOnFirstHit))
			{
			}

			explicit QueryParams(const Physics::QueryFilterData& f)
				: QueryParams(f.options, static_cast<uint32_t>(f.mask))
			{
			}

			bool Empty() const { return !dynamic && !statics; }

			bool AcceptsBody(bool isStatic) const { return isStatic ? statics : dynamic; }

			// Trigger gate + layer mask (the layer decides on the shape's belongsTo bits).
			bool AcceptsShape(const physx::PxShape& shape, bool checkMask) const
			{
				if (!triggers && (shape.getFlags() & physx::PxShapeFlag::eTRIGGER_SHAPE))
				{
					return false;
				}
				return !checkMask || (DecodeFilter(shape.getQueryFilterData()).belongsTo & mask) != 0;
			}

			physx::PxHitFlags MeshHitFlags() const
			{
				physx::PxHitFlags flags;
				if (backfaces)
				{
					flags |= physx::PxHitFlag::eMESH_BOTH_SIDES;
				}
				if (multipleMeshHits)
				{
					flags |= physx::PxHitFlag::eMESH_MULTIPLE;
				}
				return flags;
			}

			uint32_t mask;
			bool dynamic;
			bool statics;
			bool triggers;
			bool backfaces;
			bool multipleMeshHits;
			bool earlyOut;
		};

		enum class Collect
		{
			Closest, // closest blocking hit only
			Any,     // first hit found (PxQueryFlag::eANY_HIT)
			All      // every hit
		};

		struct HitRec
		{
			Physics::QueryHit hit;
			const physx::PxShape* shape = nullptr;
		};

		// Pre-filter: dynamic/static/trigger/layer gates and the optional "own actor" exclusion.
		// Post-filter: drops hits that already overlap at the start (distance 0) as Unity does for casts.
		class SceneFilter : public compat::QueryFilterCallbackBase
		{
		public:
			SceneFilter(const QueryParams& params, physx::PxQueryHitType::Enum hitType, const physx::PxRigidActor* exclude,
				bool discardInitialOverlap)
				: m_params(params), m_hitType(hitType), m_exclude(exclude), m_discardInitial(discardInitialOverlap)
			{
			}

			physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData&, const physx::PxShape* shape,
				const physx::PxRigidActor* actor, physx::PxHitFlags&) override
			{
				if (shape == nullptr || (m_exclude != nullptr && actor == m_exclude))
				{
					return physx::PxQueryHitType::eNONE;
				}
				return m_params.AcceptsShape(*shape, true) ? m_hitType : physx::PxQueryHitType::eNONE;
			}

		protected:
			physx::PxQueryHitType::Enum OnPostFilter(const physx::PxQueryHit& hit, const physx::PxShape* shape) override
			{
				if (m_discardInitial && static_cast<const physx::PxLocationHit&>(hit).hadInitialOverlap())
				{
					const bool meshBackface = m_params.backfaces && shape != nullptr
						&& compat::ShapeGeometryType(*shape) == physx::PxGeometryType::eTRIANGLEMESH;
					if (!meshBackface)
					{
						return physx::PxQueryHitType::eNONE;
					}
				}
				return m_hitType;
			}

		private:
			const QueryParams& m_params;
			physx::PxQueryHitType::Enum m_hitType;
			const physx::PxRigidActor* m_exclude;
			bool m_discardInitial;
		};

		// Collects every touch plus the blocking hit of a query into one vector.
		template <typename Hit>
		struct HitGather : physx::PxHitCallback<Hit>
		{
			HitGather() : physx::PxHitCallback<Hit>(m_buffer, kTouchBatch) {}

			physx::PxAgain processTouches(const Hit* buffer, physx::PxU32 count) override
			{
				hits.insert(hits.end(), buffer, buffer + count);
				return true;
			}

			void Finish()
			{
				if (this->hasBlock)
				{
					hits.push_back(this->block);
				}
			}

			std::vector<Hit> hits;

		private:
			Hit m_buffer[kTouchBatch];
		};

		physx::PxQueryFilterData MakeFilterData(const QueryParams& p, bool postFilter, bool anyHit)
		{
			physx::PxQueryFilterData data;
			data.flags = physx::PxQueryFlag::ePREFILTER;
			if (p.dynamic)
			{
				data.flags |= physx::PxQueryFlag::eDYNAMIC;
			}
			if (p.statics)
			{
				data.flags |= physx::PxQueryFlag::eSTATIC;
			}
			if (postFilter)
			{
				data.flags |= physx::PxQueryFlag::ePOSTFILTER;
			}
			if (anyHit)
			{
				data.flags |= physx::PxQueryFlag::eANY_HIT;
			}
			return data;
		}

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

		const ShapeObj* ShapeObjOf(const physx::PxShape* shape)
		{
			return shape != nullptr ? static_cast<const ShapeObj*>(shape->userData) : nullptr;
		}

		HitRec MakeHit(const physx::PxLocationHit& hit, const physx::PxShape* shape, float u, float v)
		{
			HitRec rec;
			rec.shape = shape;
			rec.hit.point = FromPx(hit.position);
			rec.hit.normal = FromPx(hit.normal);
			rec.hit.faceID = hit.faceIndex;
			rec.hit.distance = std::max(hit.distance, 0.0f);
			rec.hit.uv[0] = u;
			rec.hit.uv[1] = v;
			rec.hit.userData = ReadEngineObjectId(shape);
			return rec;
		}

		void SortByDistance(std::vector<HitRec>& hits)
		{
			std::stable_sort(hits.begin(), hits.end(), [](const HitRec& a, const HitRec& b) { return a.hit.distance < b.hit.distance; });
		}

		// Keeps the closest hit per shape (sorted ascending by distance first).
		void KeepClosestPerShape(std::vector<HitRec>& hits)
		{
			SortByDistance(hits);
			std::vector<HitRec> unique;
			for (const HitRec& rec : hits)
			{
				const bool seen = std::any_of(unique.begin(), unique.end(), [&](const HitRec& u) { return u.shape == rec.shape; });
				if (!seen)
				{
					unique.push_back(rec);
				}
			}
			hits.swap(unique);
		}

		// Unit direction + clamped distance. A zero distance is only valid for casts that keep initial overlaps.
		bool PrepareCast(const Physics::Vec3& direction, float maxDistance, bool allowZeroDistance, physx::PxVec3& unitDir, float& distance)
		{
			if (std::isnan(maxDistance) || maxDistance < 0.0f || (maxDistance == 0.0f && !allowZeroDistance))
			{
				return false;
			}
			const physx::PxVec3 dir = ToPx(direction);
			const float length = dir.magnitude();
			if (!(length > 1e-6f) || !std::isfinite(length))
			{
				return false;
			}
			unitDir = dir * (1.0f / length);
			distance = std::min(maxDistance, kMaxDistance);
			return true;
		}

		physx::PxScene* SceneOf(const Lock& lock)
		{
			const auto* world = lock.get<WorldObj>();
			return world != nullptr ? world->scene : nullptr;
		}

		physx::PxTransform ShapeWorldPose(const physx::PxShape& shape)
		{
			const physx::PxRigidActor* actor = shape.getActor();
			return actor != nullptr ? actor->getGlobalPose() * shape.getLocalPose() : shape.getLocalPose();
		}

		// ---- scene casts (caller holds the world domain) ----

		void CastRay(physx::PxScene& scene, const physx::PxVec3& origin, const physx::PxVec3& dir, float distance,
			const QueryParams& p, Collect mode, std::vector<HitRec>& out)
		{
			SceneFilter filter(p, mode == Collect::All ? physx::PxQueryHitType::eTOUCH : physx::PxQueryHitType::eBLOCK, nullptr, true);
			HitGather<physx::PxRaycastHit> gather;
			const physx::PxHitFlags hitFlags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL
				| physx::PxHitFlag::eFACE_INDEX | physx::PxHitFlag::eUV | p.MeshHitFlags();
			scene.raycast(origin, dir, distance, gather, hitFlags, MakeFilterData(p, true, mode == Collect::Any), &filter);
			gather.Finish();
			for (const physx::PxRaycastHit& hit : gather.hits)
			{
				out.push_back(MakeHit(hit, hit.shape, hit.u, hit.v));
			}
		}

		void CastSweep(physx::PxScene& scene, const physx::PxGeometry& geometry, const physx::PxTransform& pose,
			const physx::PxVec3& dir, float distance, const QueryParams& p, Collect mode, bool keepInitialOverlaps,
			const physx::PxRigidActor* exclude, std::vector<HitRec>& out)
		{
			SceneFilter filter(p, mode == Collect::All ? physx::PxQueryHitType::eTOUCH : physx::PxQueryHitType::eBLOCK, exclude,
				!keepInitialOverlaps);
			HitGather<physx::PxSweepHit> gather;
			const physx::PxHitFlags hitFlags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL | physx::PxHitFlag::eFACE_INDEX;
			scene.sweep(geometry, pose, dir, distance, gather, hitFlags,
				MakeFilterData(p, !keepInitialOverlaps, mode == Collect::Any), &filter);
			gather.Finish();
			for (const physx::PxSweepHit& hit : gather.hits)
			{
				out.push_back(MakeHit(hit, hit.shape, 0.0f, 0.0f));
			}
		}

		// Shapes touched by the geometry; Any stops at the first one.
		void CastOverlap(physx::PxScene& scene, const physx::PxGeometry& geometry, const physx::PxTransform& pose,
			const QueryParams& p, Collect mode, std::vector<const physx::PxShape*>& out)
		{
			SceneFilter filter(p, physx::PxQueryHitType::eTOUCH, nullptr, false);
			HitGather<physx::PxOverlapHit> gather;
			scene.overlap(geometry, pose, gather, MakeFilterData(p, false, mode == Collect::Any), &filter);
			gather.Finish();
			for (const physx::PxOverlapHit& hit : gather.hits)
			{
				out.push_back(hit.shape);
			}
		}

		void Deliver(Physics::QueryHitResultCollectorCallback collector, void* context, std::vector<HitRec>& hits)
		{
			if (collector == nullptr || hits.empty())
			{
				return;
			}
			std::vector<Physics::QueryHit> batch;
			batch.reserve(hits.size());
			for (const HitRec& rec : hits)
			{
				batch.push_back(rec.hit);
			}
			collector(context, batch.data(), batch.size());
		}

		void ResetHit(Physics::QueryHit& hit, uint32_t& count)
		{
			hit = Physics::QueryHit::Invalid();
			count = 0;
		}

		void SetClosest(std::vector<HitRec>& hits, Physics::QueryHit& hit, uint32_t& count)
		{
			if (hits.empty())
			{
				return;
			}
			const auto closest = std::min_element(hits.begin(), hits.end(),
				[](const HitRec& a, const HitRec& b) { return a.hit.distance < b.hit.distance; });
			hit = closest->hit;
			count = 1;
		}

		bool SweepSupported(physx::PxGeometryType::Enum type)
		{
			return type == physx::PxGeometryType::eSPHERE || type == physx::PxGeometryType::eCAPSULE
				|| type == physx::PxGeometryType::eBOX || type == physx::PxGeometryType::eCONVEXMESH;
		}

		bool IsComplexGeometry(physx::PxGeometryType::Enum type)
		{
			return type == physx::PxGeometryType::eTRIANGLEMESH || type == physx::PxGeometryType::eHEIGHTFIELD
				|| type == physx::PxGeometryType::ePLANE;
		}

		// ---- scene-wide commands ----

		void RayCast(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::RayCast&>(c);
			ResetHit(cmd.hit, cmd.hitCount);

			const QueryParams params(cmd.filter);
			physx::PxVec3 dir;
			float distance = 0.0f;
			if (params.Empty() || !PrepareCast(cmd.direction, cmd.maxDistance, false, dir, distance))
			{
				return;
			}

			std::vector<HitRec> hits;
			{
				Lock lock(Target{ Kind::World, sdkObject, ctx.world });
				physx::PxScene* scene = lock ? SceneOf(lock) : nullptr;
				if (scene == nullptr)
				{
					return;
				}
				CastRay(*scene, ToPx(cmd.origin), dir, distance, params, params.earlyOut ? Collect::Any : Collect::Closest, hits);
			}
			SetClosest(hits, cmd.hit, cmd.hitCount);
		}

		void RayCastWithCollector(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::RaycastWithCollector&>(c);

			const QueryParams params(cmd.filter);
			physx::PxVec3 dir;
			float distance = 0.0f;
			if (cmd.collector == nullptr || params.Empty() || !PrepareCast(cmd.direction, cmd.maxDistance, false, dir, distance))
			{
				return;
			}

			std::vector<HitRec> hits;
			{
				Lock lock(Target{ Kind::World, sdkObject, ctx.world });
				physx::PxScene* scene = lock ? SceneOf(lock) : nullptr;
				if (scene == nullptr)
				{
					return;
				}
				CastRay(*scene, ToPx(cmd.origin), dir, distance, params, params.earlyOut ? Collect::Any : Collect::All, hits);
			}
			SortByDistance(hits);
			Deliver(cmd.collector, cmd.dataContext, hits);
		}

		void Overlap(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::Overlap&>(c);
			cmd.shapeUserData = nullptr;

			const QueryParams params(cmd.filter);
			QueryGeometry builder;
			const physx::PxGeometry* geometry = params.Empty() ? nullptr : builder.Build(cmd.geometry);
			if (geometry == nullptr)
			{
				return;
			}

			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			physx::PxScene* scene = lock ? SceneOf(lock) : nullptr;
			if (scene == nullptr)
			{
				return;
			}
			std::vector<const physx::PxShape*> shapes;
			CastOverlap(*scene, *geometry, ToPx(cmd.pose), params, Collect::Any, shapes);
			if (!shapes.empty())
			{
				const ShapeObj* shape = ShapeObjOf(shapes.front());
				cmd.shapeUserData = shape != nullptr ? shape->userData : nullptr;
			}
		}

		void OverlapWithCollector(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::OverlapWithCollector&>(c);

			const QueryParams params(cmd.filter);
			QueryGeometry builder;
			const physx::PxGeometry* geometry = (cmd.collector == nullptr || params.Empty()) ? nullptr : builder.Build(cmd.geometry);
			if (geometry == nullptr)
			{
				return;
			}

			std::vector<void*> userDatas;
			{
				Lock lock(Target{ Kind::World, sdkObject, ctx.world });
				physx::PxScene* scene = lock ? SceneOf(lock) : nullptr;
				if (scene == nullptr)
				{
					return;
				}
				std::vector<const physx::PxShape*> shapes;
				CastOverlap(*scene, *geometry, ToPx(cmd.pose), params, params.earlyOut ? Collect::Any : Collect::All, shapes);
				for (const physx::PxShape* px : shapes)
				{
					const ShapeObj* shape = ShapeObjOf(px);
					userDatas.push_back(shape != nullptr ? shape->userData : nullptr);
				}
			}
			if (!userDatas.empty())
			{
				cmd.collector(cmd.dataContext, userDatas.data(), userDatas.size());
			}
		}

		// Approximated with the AABB of the query shape: every shape touching that box is reported one by one.
		void OverlapWithBroadphaseCollector(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::OverlapWithBroadphaseCollector&>(c);

			const QueryParams params(cmd.filter);
			QueryGeometry builder;
			const physx::PxGeometry* geometry = (cmd.collector == nullptr || params.Empty()) ? nullptr : builder.Build(cmd.geometry);
			if (geometry == nullptr)
			{
				return;
			}

			struct Entry
			{
				Handle shape;
				Handle body;
				void* userData;
				Physics::GeometryType type;
			};
			std::vector<Entry> entries;
			{
				Lock lock(Target{ Kind::World, sdkObject, ctx.world });
				physx::PxScene* scene = lock ? SceneOf(lock) : nullptr;
				if (scene == nullptr)
				{
					return;
				}
				const physx::PxBounds3 bounds = compat::ComputeGeometryBounds(*geometry, ToPx(cmd.pose));
				if (bounds.isEmpty())
				{
					return;
				}
				const physx::PxVec3 extents = bounds.getExtents();
				const physx::PxBoxGeometry box(std::max(extents.x, kMinExtent), std::max(extents.y, kMinExtent), std::max(extents.z, kMinExtent));
				std::vector<const physx::PxShape*> shapes;
				CastOverlap(*scene, box, physx::PxTransform(bounds.getCenter()), params, Collect::All, shapes);
				for (const physx::PxShape* px : shapes)
				{
					const ShapeObj* shape = ShapeObjOf(px);
					if (shape != nullptr)
					{
						entries.push_back({ const_cast<ShapeObj*>(shape), ShapeOwnerHandle(*shape), shape->userData, shape->geometry.type });
					}
				}
			}
			for (const Entry& e : entries)
			{
				if (!cmd.collector(cmd.dataContext, e.shape, e.body, e.userData, e.type))
				{
					break;
				}
			}
		}

		void ShapeCast(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::ShapeCast&>(c);
			ResetHit(cmd.hit, cmd.hitCount);

			const QueryParams params(cmd.filter);
			QueryGeometry builder;
			const physx::PxGeometry* geometry = params.Empty() ? nullptr : builder.Build(cmd.geometry);
			physx::PxVec3 dir;
			float distance = 0.0f;
			if (geometry == nullptr || !PrepareCast(cmd.direction, cmd.maxDistance, false, dir, distance))
			{
				return;
			}

			std::vector<HitRec> hits;
			{
				Lock lock(Target{ Kind::World, sdkObject, ctx.world });
				physx::PxScene* scene = lock ? SceneOf(lock) : nullptr;
				if (scene == nullptr)
				{
					return;
				}
				CastSweep(*scene, *geometry, ToPx(cmd.pose), dir, distance, params,
					params.earlyOut ? Collect::Any : Collect::Closest, false, nullptr, hits);
			}
			SetClosest(hits, cmd.hit, cmd.hitCount);
		}

		void ShapeCastWithCollector(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::ShapeCastWithCollector&>(c);

			const QueryParams params(cmd.filter);
			QueryGeometry builder;
			const physx::PxGeometry* geometry = (cmd.collector == nullptr || params.Empty()) ? nullptr : builder.Build(cmd.geometry);
			physx::PxVec3 dir;
			float distance = 0.0f;
			if (geometry == nullptr || !PrepareCast(cmd.direction, cmd.maxDistance, cmd.keepInitialOverlaps, dir, distance))
			{
				return;
			}

			std::vector<HitRec> hits;
			{
				Lock lock(Target{ Kind::World, sdkObject, ctx.world });
				physx::PxScene* scene = lock ? SceneOf(lock) : nullptr;
				if (scene == nullptr)
				{
					return;
				}
				CastSweep(*scene, *geometry, ToPx(cmd.pose), dir, distance, params, params.earlyOut ? Collect::Any : Collect::All,
					cmd.keepInitialOverlaps, nullptr, hits);
			}
			SortByDistance(hits);
			Deliver(cmd.collector, cmd.dataContext, hits);
		}

		// ---- body cast ----

		// Sweeps every non-trigger shape of the body; the body itself is never hit.
		void BodyCastImpl(Handle sdkObject, const Context& ctx, Handle bodyHandle, const Physics::Vec3& direction, float maxDistance,
			const Physics::QueryFilterData& filter, bool all, std::vector<HitRec>& hits)
		{
			const QueryParams params(filter);
			physx::PxVec3 dir;
			float distance = 0.0f;
			if (params.Empty() || !PrepareCast(direction, maxDistance, false, dir, distance))
			{
				return;
			}

			const Handle body = Pick(Kind::Body, bodyHandle, sdkObject, ctx.body);
			Lock lock(Target{ Kind::Body, body });
			if (!lock)
			{
				return;
			}
			const BodyObj* bodyObj = lock.get<BodyObj>();
			if (bodyObj->world == nullptr || bodyObj->world->scene == nullptr || bodyObj->actor == nullptr)
			{
				return;
			}

			for (const ShapeObj* shape : bodyObj->shapes)
			{
				if (shape->shape == nullptr || (shape->shape->getFlags() & physx::PxShapeFlag::eTRIGGER_SHAPE))
				{
					continue;
				}
				const physx::PxGeometryHolder holder = compat::CopyShapeGeometry(*shape->shape);
				if (!SweepSupported(holder.getType()))
				{
					WarnUnsupportedGeometry(static_cast<int>(holder.getType()));
					continue;
				}
				CastSweep(*bodyObj->world->scene, holder.any(), ShapeWorldPose(*shape->shape), dir, distance, params,
					all ? Collect::All : Collect::Closest, false, bodyObj->actor, hits);
			}
			KeepClosestPerShape(hits);
		}

		void BodyCast(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::BodyCast&>(c);
			ResetHit(cmd.hit, cmd.hitCount);

			std::vector<HitRec> hits;
			BodyCastImpl(sdkObject, ctx, cmd.body, cmd.direction, cmd.maxDistance, cmd.filter, false, hits);
			SetClosest(hits, cmd.hit, cmd.hitCount);
		}

		void BodyCastWithCollector(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::BodyCastWithCollector&>(c);
			if (cmd.collector == nullptr)
			{
				return;
			}

			std::vector<HitRec> hits;
			BodyCastImpl(sdkObject, ctx, cmd.body, cmd.direction, cmd.maxDistance, cmd.filter, true, hits);
			Deliver(cmd.collector, cmd.dataContext, hits);
		}

		// ---- single shape commands ----

		// Resolves the target of an "AgainstShape" command: the shape handle itself, or all shapes of a body handle.
		struct TargetShapes
		{
			static bool IsShapeHandle(Handle sdkObject)
			{
				return IsKind(sdkObject, Kind::Shape);
			}

			static Target MakeTarget(Handle sdkObject, const Context& ctx)
			{
				return IsShapeHandle(sdkObject) ? Target{ Kind::Shape, sdkObject } : Target{ Kind::Body, sdkObject, ctx.body };
			}

			static std::vector<const ShapeObj*> Resolve(const Lock& lock)
			{
				std::vector<const ShapeObj*> shapes;
				if (IsKind(lock.handle(), Kind::Shape))
				{
					shapes.push_back(lock.get<ShapeObj>());
				}
				else
				{
					for (const ShapeObj* shape : lock.get<BodyObj>()->shapes)
					{
						shapes.push_back(shape);
					}
				}
				return shapes;
			}
		};

		bool BodyAccepted(const QueryParams& p, const ShapeObj& shape)
		{
			return p.AcceptsBody(shape.ownerLink == nullptr && (shape.owner == nullptr || shape.owner->isStatic));
		}

		void RayCastAgainstShape(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::RayCastAgainstShape&>(c);
			ResetHit(cmd.hit, cmd.hitCount);

			const QueryParams params(cmd.filterOptions, 0xFFFFFFFFu);
			physx::PxVec3 dir;
			float distance = 0.0f;
			if (params.Empty() || !PrepareCast(cmd.direction, cmd.maxDistance, false, dir, distance))
			{
				return;
			}

			const physx::PxVec3 origin = ToPx(cmd.origin);
			const physx::PxHitFlags hitFlags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL
				| physx::PxHitFlag::eFACE_INDEX | physx::PxHitFlag::eUV | params.MeshHitFlags();

			Lock lock(TargetShapes::MakeTarget(sdkObject, ctx));
			if (!lock)
			{
				return;
			}
			for (const ShapeObj* shape : TargetShapes::Resolve(lock))
			{
				if (shape->shape == nullptr || !BodyAccepted(params, *shape) || !params.AcceptsShape(*shape->shape, false))
				{
					continue;
				}
				const physx::PxGeometryHolder holder = compat::CopyShapeGeometry(*shape->shape);
				compat::GeomRaycastHit hit;
				if (physx::PxGeometryQuery::raycast(origin, dir, holder.any(), ShapeWorldPose(*shape->shape), distance, hitFlags, 1, &hit) == 0)
				{
					continue;
				}
				const bool meshBackface = params.backfaces && holder.getType() == physx::PxGeometryType::eTRIANGLEMESH;
				if (hit.hadInitialOverlap() && !meshBackface)
				{
					continue;
				}
				if (cmd.hitCount == 0 || hit.distance < cmd.hit.distance)
				{
					cmd.hit = MakeHit(hit, shape->shape, hit.u, hit.v).hit;
					cmd.hitCount = 1;
				}
			}
		}

		// The shape given by the handle stays where it is; the geometry in the command (at targetPose) is swept against it.
		void ShapeCastAgainstShape(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<qd::ShapeCastAgainstShape&>(c);
			ResetHit(cmd.hit, cmd.hitCount);

			const QueryParams params(cmd.filter);
			QueryGeometry builder;
			const physx::PxGeometry* swept = params.Empty() ? nullptr : builder.Build(cmd.targetGeometry);
			physx::PxVec3 dir;
			float distance = 0.0f;
			if (swept == nullptr || !PrepareCast(cmd.direction, cmd.maxDistance, cmd.distanceAsDepth, dir, distance))
			{
				return;
			}

			physx::PxHitFlags hitFlags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL | physx::PxHitFlag::eFACE_INDEX;
			if (cmd.distanceAsDepth)
			{
				hitFlags |= physx::PxHitFlag::eMTD;
			}
			const physx::PxTransform sweptPose = ToPx(cmd.targetPose);

			Lock lock(TargetShapes::MakeTarget(sdkObject, ctx));
			if (!lock)
			{
				return;
			}
			for (const ShapeObj* shape : TargetShapes::Resolve(lock))
			{
				if (shape->shape == nullptr || !BodyAccepted(params, *shape) || !params.AcceptsShape(*shape->shape, true))
				{
					continue;
				}
				const physx::PxGeometryHolder holder = compat::CopyShapeGeometry(*shape->shape);
				compat::GeomSweepHit hit;
				if (!physx::PxGeometryQuery::sweep(dir, distance, *swept, sweptPose, holder.any(), ShapeWorldPose(*shape->shape), hit, hitFlags))
				{
					continue;
				}

				HitRec rec = MakeHit(hit, shape->shape, 0.0f, 0.0f);
				if (hit.hadInitialOverlap())
				{
					if (!cmd.distanceAsDepth)
					{
						continue;
					}
					rec.hit.distance = std::max(-hit.distance, 0.0f); // eMTD: negative distance = penetration depth
				}
				if (cmd.hitCount == 0 || rec.hit.distance < cmd.hit.distance)
				{
					cmd.hit = rec.hit;
					cmd.hitCount = 1;
				}
			}
		}

		// ---- shape pair queries ----

		void ComputeShapePenetration(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<qd::ComputeShapePenetration&>(c);
			cmd.outDirection = Physics::Vec3(0.0f, 0.0f, 0.0f);
			cmd.outDistance = 0.0f;
			cmd.areOverlapping = false;

			if (cmd.shapeA == cmd.shapeB)
			{
				return;
			}
			Lock lock(Target{ Kind::Shape, cmd.shapeA }, Target{ Kind::Shape, cmd.shapeB });
			if (!lock)
			{
				return;
			}
			const ShapeObj* a = lock.get<ShapeObj>(0);
			const ShapeObj* b = lock.get<ShapeObj>(1);
			if (a->shape == nullptr || b->shape == nullptr)
			{
				return;
			}

			const physx::PxGeometryHolder geomA = compat::CopyShapeGeometry(*a->shape);
			const physx::PxGeometryHolder geomB = compat::CopyShapeGeometry(*b->shape);
			if (IsComplexGeometry(geomA.getType()) && IsComplexGeometry(geomB.getType()))
			{
				WarnUnsupportedGeometry(static_cast<int>(geomA.getType()));
				return;
			}

			physx::PxVec3 direction(0.0f);
			physx::PxF32 depth = 0.0f;
			if (physx::PxGeometryQuery::computePenetration(direction, depth, geomA.any(), ToPx(cmd.poseA), geomB.any(), ToPx(cmd.poseB)))
			{
				cmd.outDirection = FromPx(direction);
				cmd.outDistance = depth;
				cmd.areOverlapping = true;
			}
		}

		// A point inside the shape yields the point itself with distance 0 (PhysX has no signed distance).
		void ClosestPointOnShape(Handle sdkObject, const Context&, Command& c)
		{
			auto& cmd = static_cast<qd::ClosestPointOnShape&>(c);
			cmd.outPoint = cmd.point;
			cmd.outDistance = 0.0f;

			Lock lock(Target{ Kind::Shape, sdkObject });
			if (!lock || lock.get<ShapeObj>()->shape == nullptr)
			{
				return;
			}

			const physx::PxGeometryHolder holder = compat::CopyShapeGeometry(*lock.get<ShapeObj>()->shape);
			physx::PxVec3 closest(0.0f);
			const float squared = physx::PxGeometryQuery::pointDistance(ToPx(cmd.point), holder.any(), ToPx(cmd.pose), &closest);
			if (squared < 0.0f)
			{
				WarnUnsupportedGeometry(static_cast<int>(holder.getType()));
				return;
			}
			if (squared > 0.0f)
			{
				cmd.outPoint = FromPx(closest);
				cmd.outDistance = std::sqrt(squared);
			}
		}
	}

	void RegisterQueryHandlers()
	{
		using Q = pc::Query;
		Bind(Q::ComputeShapePenetration, &ComputeShapePenetration);
		Bind(Q::ClosestPointOnShape, &ClosestPointOnShape);
		Bind(Q::RayCast, &RayCast);
		Bind(Q::RayCastAgainstShape, &RayCastAgainstShape);
		Bind(Q::RayCastWithCollector, &RayCastWithCollector);
		Bind(Q::Overlap, &Overlap);
		Bind(Q::OverlapWithCollector, &OverlapWithCollector);
		Bind(Q::OverlapWithBroadphaseCollector, &OverlapWithBroadphaseCollector);
		Bind(Q::ShapeCast, &ShapeCast);
		Bind(Q::ShapeCastAgainstShape, &ShapeCastAgainstShape);
		Bind(Q::ShapeCastWithCollector, &ShapeCastWithCollector);
		Bind(Q::BodyCast, &BodyCast);
		Bind(Q::BodyCastWithCollector, &BodyCastWithCollector);
	}
}
