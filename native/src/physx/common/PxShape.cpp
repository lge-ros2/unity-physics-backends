#include "core/Dispatch.h"
#include "physx/common/PhysXSdk.h"
#include "physx/common/PxFilter.h"
#include "physx/common/PxMesh.h"
#include "physx/common/PxObjects.h"

#include "PhysicsShapeCommandData.h"

#include <algorithm>
#include <atomic>
#include <cmath>

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace sd = PhysicsCommands::ShapeData;

	namespace
	{
		constexpr float kMinExtent = 1e-4f;

		float Clamp(float v)
		{
			return std::isfinite(v) ? std::max(v, kMinExtent) : kMinExtent;
		}

		// Mesh scales keep their sign (mirroring) but must not collapse to zero.
		float ClampScale(float v)
		{
			if (!std::isfinite(v))
			{
				return 1.0f;
			}
			return std::fabs(v) < kMinExtent ? std::copysign(kMinExtent, v) : v;
		}

		physx::PxMeshScale ToMeshScale(const Physics::Vec3& s)
		{
			return physx::PxMeshScale(physx::PxVec3(ClampScale(s.x), ClampScale(s.y), ClampScale(s.z)));
		}

		// Converts a Physics::ShapeGeometry into a PhysX geometry (box, sphere, capsule, convex / triangle mesh, heightfield).
		struct GeometryBuilder
		{
			physx::PxBoxGeometry box;
			physx::PxSphereGeometry sphere;
			physx::PxCapsuleGeometry capsule;
			physx::PxConvexMeshGeometry convexMesh;
			physx::PxTriangleMeshGeometry triangleMesh;
			physx::PxHeightFieldGeometry heightField;

			// Returns null for unsupported geometry types and for mesh handles that are not known.
			const physx::PxGeometry* Build(const Physics::ShapeGeometry& g)
			{
				switch (g.type)
				{
				case Physics::GeometryType::ConvexMesh:
				{
					const MeshObj* mesh = FindMesh(g.meshGeometry.mesh);
					if (mesh == nullptr || mesh->convex == nullptr)
					{
						return nullptr;
					}
					convexMesh = physx::PxConvexMeshGeometry(mesh->convex, ToMeshScale(g.meshGeometry.scale));
					return &convexMesh;
				}
				case Physics::GeometryType::TriangleMesh:
				{
					const MeshObj* mesh = FindMesh(g.meshGeometry.mesh);
					if (mesh == nullptr || mesh->triangle == nullptr)
					{
						return nullptr;
					}
					triangleMesh = physx::PxTriangleMeshGeometry(mesh->triangle, ToMeshScale(g.meshGeometry.scale));
					return &triangleMesh;
				}
				case Physics::GeometryType::Terrain:
				{
					const HeightFieldObj* field = FindHeightField(g.heightFieldGeometry.heightField);
					if (field == nullptr || field->field == nullptr)
					{
						return nullptr;
					}
					// Samples are laid out as rows along x and columns along z; scale = (x spacing, height scale, z spacing).
					const Physics::Vec3& s = g.heightFieldGeometry.scale;
					heightField = physx::PxHeightFieldGeometry(field->field, physx::PxMeshGeometryFlags(), Clamp(s.y), Clamp(s.x), Clamp(s.z));
					return &heightField;
				}
				case Physics::GeometryType::Box:
					box = physx::PxBoxGeometry(Clamp(g.boxGeometry.halfExtents.x), Clamp(g.boxGeometry.halfExtents.y),
						Clamp(g.boxGeometry.halfExtents.z));
					return &box;
				case Physics::GeometryType::Sphere:
					sphere = physx::PxSphereGeometry(Clamp(g.sphereGeometry.radius));
					return &sphere;
				case Physics::GeometryType::Capsule:
				{
					// Unity passes the total height including both hemispheres; PhysX wants the cylinder half height.
					const float radius = Clamp(g.capsuleGeometry.radius);
					const float halfHeight = Clamp((g.capsuleGeometry.height - 2.0f * radius) * 0.5f);
					capsule = physx::PxCapsuleGeometry(radius, halfHeight);
					return &capsule;
				}
				default:
					return nullptr;
				}
			}
		};

		void WarnUnsupportedGeometry(Physics::GeometryType type)
		{
			static std::atomic<uint32_t> s_warned{ 0 };
			const uint32_t bit = 1u << (static_cast<uint32_t>(type) & 31u);
			if ((s_warned.fetch_or(bit) & bit) == 0)
			{
				BACKEND_WARNF("Shape geometry type %d is not supported by the PhysX backend (or its mesh handle is invalid); shape rejected",
					static_cast<int>(type));
			}
		}

		bool IsMeshRestricted(Physics::GeometryType type)
		{
			return type == Physics::GeometryType::TriangleMesh || type == Physics::GeometryType::Terrain;
		}

		// Triangle meshes and heightfields can only simulate on static or kinematic bodies.
		bool MeshSimulationAllowed(const BodyObj* owner)
		{
			if (owner == nullptr || owner->dynamic == nullptr)
			{
				return true;
			}
			return (owner->dynamic->getRigidBodyFlags() & physx::PxRigidBodyFlag::eKINEMATIC) ? true : false;
		}

		// Pins the mesh / heightfield a geometry refers to until it is handed to a shape.
		class GeometryPin
		{
		public:
			GeometryPin() = default;
			GeometryPin(const GeometryPin&) = delete;
			GeometryPin& operator=(const GeometryPin&) = delete;

			~GeometryPin()
			{
				ReleaseMesh(m_mesh);
				ReleaseHeightField(m_field, nullptr);
			}

			// False when the geometry needs a mesh / heightfield handle that is not alive.
			bool Acquire(const Physics::ShapeGeometry& g)
			{
				switch (g.type)
				{
				case Physics::GeometryType::ConvexMesh:
				case Physics::GeometryType::TriangleMesh:
					m_mesh = AcquireMesh(g.meshGeometry.mesh, g.type);
					return m_mesh != nullptr;
				case Physics::GeometryType::Terrain:
					m_field = AcquireHeightField(g.heightFieldGeometry.heightField);
					return m_field != nullptr;
				default:
					return true;
				}
			}

			// Transfers the references to the shape (which must not hold any).
			void CommitTo(ShapeObj& shape)
			{
				shape.mesh = m_mesh;
				shape.heightField = m_field;
				if (m_field != nullptr)
				{
					LinkHeightFieldShape(m_field, &shape);
				}
				m_mesh = nullptr;
				m_field = nullptr;
			}

		private:
			MeshObj* m_mesh = nullptr;
			HeightFieldObj* m_field = nullptr;
		};

		// meshSimulationAllowed: whether the owning body lets triangle mesh / heightfield shapes simulate.
		void ApplyShapeFlags(ShapeObj& s, Physics::ShapeFlags flags, bool meshSimulationAllowed)
		{
			s.flags = flags;
			const bool restricted = IsMeshRestricted(s.geometry.type);
			bool trigger = (flags & Physics::ShapeFlags::Trigger) != Physics::ShapeFlags::None;
			const bool query = (flags & Physics::ShapeFlags::SceneQuery) != Physics::ShapeFlags::None;
			if (trigger && restricted)
			{
				BACKEND_WARNF("Triangle mesh / heightfield shapes cannot be triggers; trigger flag ignored");
				trigger = false;
			}
			const bool wantSimulation = !trigger && (!restricted || meshSimulationAllowed);
			if (restricted && !trigger && !meshSimulationAllowed)
			{
				static std::atomic<bool> s_warned{ false };
				if (!s_warned.exchange(true))
				{
					BACKEND_WARNF("Triangle mesh / heightfield shapes cannot simulate on a non-kinematic dynamic body; they stay scene-query only");
				}
			}

			const physx::PxShapeFlags cur = s.shape->getFlags();
			const bool isTrigger = cur & physx::PxShapeFlag::eTRIGGER_SHAPE;
			const bool isSim = cur & physx::PxShapeFlag::eSIMULATION_SHAPE;
			const bool isQuery = cur & physx::PxShapeFlag::eSCENE_QUERY_SHAPE;

			// A shape is either a simulation shape or a trigger shape; clear one before raising the other.
			if (trigger)
			{
				if (isSim)
				{
					s.shape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, false);
				}
				if (!isTrigger)
				{
					s.shape->setFlag(physx::PxShapeFlag::eTRIGGER_SHAPE, true);
				}
			}
			else
			{
				if (isTrigger)
				{
					s.shape->setFlag(physx::PxShapeFlag::eTRIGGER_SHAPE, false);
				}
				if (isSim != wantSimulation)
				{
					s.shape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, wantSimulation);
				}
			}
			if (query != isQuery)
			{
				s.shape->setFlag(physx::PxShapeFlag::eSCENE_QUERY_SHAPE, query);
			}
		}

		physx::PxTransform ShapeWorldPose(const ShapeObj& s)
		{
			const physx::PxTransform local = s.shape->getLocalPose();
			const physx::PxRigidActor* actor = s.shape->getActor(); // body or articulation link
			return actor != nullptr ? actor->getGlobalPose() * local : local;
		}

		void FillBounds(const physx::PxBounds3& bounds, Physics::Vec3& extents, Physics::Vec3& center)
		{
			if (bounds.isEmpty())
			{
				extents = Physics::Vec3(0.0f, 0.0f, 0.0f);
				return;
			}
			extents = FromPx(bounds.getExtents());
			center = FromPx(bounds.getCenter());
		}

		// Shapes that carry one of the ignore ids need their cached pair decisions refreshed.
		void RefilterIgnoreIds(uint16_t id0, uint16_t id1)
		{
			for (Handle h : AllHandles(Kind::World))
			{
				Lock lock(Target{ Kind::World, h, nullptr });
				if (!lock)
				{
					continue;
				}
				WorldObj* world = lock.get<WorldObj>();
				for (BodyObj* body : world->bodies)
				{
					if (!body->inScene)
					{
						continue;
					}
					for (const ShapeObj* shape : body->shapes)
					{
						const uint16_t id = shape->filter.ignorePairID;
						if (id != Physics::FilterData::invalidIgnorePairID && (id == id0 || id == id1))
						{
							world->scene->resetFiltering(*body->actor);
							break;
						}
					}
				}
				RefilterLinkShapes(*world, id0, id1);
			}
		}

		// ---- handlers ----

		void CreateShape(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CreateShape&>(c);
			cmd.value = nullptr;

			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (physics == nullptr)
			{
				return;
			}

			GeometryPin pin;
			GeometryBuilder builder;
			const physx::PxGeometry* geometry = pin.Acquire(cmd.geom) ? builder.Build(cmd.geom) : nullptr;
			if (geometry == nullptr)
			{
				WarnUnsupportedGeometry(cmd.geom.type);
				return;
			}

			MaterialObj* material = static_cast<MaterialObj*>(Lookup(Pick(Kind::Material, cmd.materialPtr), Kind::Material));
			if (material == nullptr)
			{
				material = DefaultMaterial();
			}
			if (material == nullptr)
			{
				return;
			}

			auto* shape = new ShapeObj();
			shape->geometry = cmd.geom;
			shape->userData = cmd.userData;
			shape->shape = physics->createShape(*geometry, *material->material, true,
				physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSIMULATION_SHAPE);
			if (shape->shape == nullptr)
			{
				BACKEND_ERRORF("PxPhysics::createShape failed");
				delete shape;
				return;
			}
			shape->shape->userData = shape;
			pin.CommitTo(*shape);
			ApplyShapeFlags(*shape, cmd.flags, true);
			shape->shape->setSimulationFilterData(EncodeFilter(shape->filter));
			shape->shape->setQueryFilterData(EncodeFilter(shape->filter));

			Register(shape, Kind::Shape, shape, LooseState());
			cmd.value = shape;
		}

		void DestroyShape(Handle sdkObject, const Context& ctx, Command&)
		{
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (lock)
			{
				DestroyShapeLocked(lock.handle(), lock.get<ShapeObj>());
			}
		}

		void GetActorDescriptor(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<sd::GetActorDescriptor&>(c);
			cmd.value = Physics::ActorDescriptor();
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (!lock)
			{
				return;
			}
			const BodyObj* owner = lock.get<ShapeObj>()->owner;
			if (owner != nullptr)
			{
				cmd.value.actor = const_cast<BodyObj*>(owner);
				cmd.value.userData = owner->userData;
				cmd.value.type = owner->isStatic ? Physics::ActorType::Static : Physics::ActorType::Dynamic;
			}
			else if (const Handle link = ShapeOwnerHandle(*lock.get<ShapeObj>()))
			{
				cmd.value.actor = link;
				cmd.value.userData = LinkUserData(link);
				cmd.value.type = Physics::ActorType::Link;
			}
		}

		void GetUserData(Handle sdkObject, const Context& ctx, Command& c)
		{
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			static_cast<sd::GetUserData&>(c).value = lock ? lock.get<ShapeObj>()->userData : nullptr;
		}

		void GetFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			static_cast<sd::GetFlags&>(c).flags = lock ? lock.get<ShapeObj>()->flags : Physics::ShapeFlags::None;
		}

		void SetFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Physics::ShapeFlags flags = static_cast<sd::SetFlags&>(c).flags;
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (lock)
			{
				ShapeObj* shape = lock.get<ShapeObj>();
				ApplyShapeFlags(*shape, flags, shape->ownerLink == nullptr && MeshSimulationAllowed(shape->owner));
			}
		}

		void GetFilterData(Handle sdkObject, const Context& ctx, Command& c)
		{
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			static_cast<sd::GetFilterData&>(c).data = lock ? lock.get<ShapeObj>()->filter : Physics::FilterData();
		}

		void SetFilterData(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Physics::FilterData& data = static_cast<sd::SetFilterData&>(c).data;
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (!lock)
			{
				return;
			}
			ShapeObj* shape = lock.get<ShapeObj>();
			if (shape->filter == data)
			{
				return;
			}
			shape->filter = data;
			shape->shape->setQueryFilterData(EncodeFilter(data));
			UpdateShapeSimulationFilter(*shape);
		}

		void SetMaterial(Handle sdkObject, const Context& ctx, Command& c)
		{
			Handle materialHandle = static_cast<sd::SetMaterial&>(c).value;
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (!lock)
			{
				return;
			}
			auto* material = static_cast<MaterialObj*>(Lookup(Pick(Kind::Material, materialHandle), Kind::Material));
			if (material == nullptr)
			{
				material = DefaultMaterial();
			}
			if (material != nullptr)
			{
				physx::PxMaterial* px = material->material;
				lock.get<ShapeObj>()->shape->setMaterials(&px, 1);
			}
		}

		void GetLocalPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			static_cast<sd::GetLocalPose&>(c).value = lock ? FromPx(lock.get<ShapeObj>()->shape->getLocalPose()) : Physics::Pose();
		}

		void GetPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			static_cast<sd::GetPose&>(c).value = lock ? FromPx(ShapeWorldPose(*lock.get<ShapeObj>())) : Physics::Pose();
		}

		void SetPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Physics::Pose& pose = static_cast<sd::SetPose&>(c).value;
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (lock)
			{
				lock.get<ShapeObj>()->shape->setLocalPose(ToPx(pose));
			}
		}

		void SetContactOffset(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = static_cast<sd::SetContactOffset&>(c).value;
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (lock && std::isfinite(value))
			{
				// PhysX requires contactOffset > restOffset (0 by default).
				lock.get<ShapeObj>()->shape->setContactOffset(std::max(value, 1e-5f));
			}
		}

		void GetGeometryType(Handle sdkObject, const Context& ctx, Command& c)
		{
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			static_cast<sd::GetGeometryType&>(c).value = lock ? lock.get<ShapeObj>()->geometry.type : Physics::GeometryType::Invalid;
		}

		void GetGeometry(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<sd::GetGeometry&>(c);
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (lock)
			{
				cmd.value = lock.get<ShapeObj>()->geometry;
			}
			else
			{
				cmd.value = Physics::ShapeGeometry();
				cmd.value.type = Physics::GeometryType::Invalid;
			}
		}

		void SetGeometry(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Physics::ShapeGeometry& geom = static_cast<sd::SetGeometry&>(c).value;
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (!lock)
			{
				return;
			}
			ShapeObj* shape = lock.get<ShapeObj>();
			GeometryPin pin;
			GeometryBuilder builder;
			const physx::PxGeometry* px = pin.Acquire(geom) ? builder.Build(geom) : nullptr;
			if (px == nullptr || geom.type != shape->geometry.type)
			{
				// PhysX cannot swap the geometry type of an existing shape; Unity recreates the shape instead.
				WarnUnsupportedGeometry(geom.type);
				return;
			}
			shape->shape->setGeometry(*px);
			shape->geometry = geom;
			// The new references are pinned first, so a mesh shared by old and new geometry never drops to zero.
			ReleaseShapeGeometryRefs(*shape);
			pin.CommitTo(*shape);
		}

		void GetWorldBounds(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<sd::GetWorldBounds&>(c);
			cmd.extents = Physics::Vec3(0.0f, 0.0f, 0.0f);
			cmd.center = Physics::Vec3(0.0f, 0.0f, 0.0f);
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (!lock)
			{
				return;
			}
			const ShapeObj* shape = lock.get<ShapeObj>();
			GeometryBuilder builder;
			if (const physx::PxGeometry* geometry = builder.Build(shape->geometry))
			{
				FillBounds(compat::ComputeGeometryBounds(*geometry, ShapeWorldPose(*shape)), cmd.extents, cmd.center);
			}
		}

		void GetBoundsAtPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<sd::GetBoundsAtPose&>(c);
			cmd.extents = Physics::Vec3(0.0f, 0.0f, 0.0f);
			cmd.center = cmd.pose.pos;
			Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
			if (!lock)
			{
				return;
			}
			GeometryBuilder builder;
			// The pose is taken as the shape's own world pose (not the body pose).
			if (const physx::PxGeometry* geometry = builder.Build(lock.get<ShapeObj>()->geometry))
			{
				FillBounds(compat::ComputeGeometryBounds(*geometry, ToPx(cmd.pose)), cmd.extents, cmd.center);
			}
		}

		void ShouldIgnoreCollision(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::ShouldIgnoreCollision&>(c);
			cmd.value = IsPairIgnored(cmd.ignoreId0, cmd.ignoreId1);
		}

		void IgnoreCollision(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::IgnoreCollision&>(c);
			SetPairIgnored(cmd.ignoreId0, cmd.ignoreId1, cmd.value);
			RefilterIgnoreIds(cmd.ignoreId0, cmd.ignoreId1);
		}

		void CleanupIgnoredColliders(Handle, const Context&, Command& c)
		{
			const uint16_t id = static_cast<sd::CleanupIgnoredColliders&>(c).ignoreID;
			if (CleanupIgnoredPairs(id))
			{
				RefilterIgnoreIds(id, id);
			}
		}
	}

	void ReapplyShapeFlags(ShapeObj& shape, const BodyObj* owner)
	{
		ApplyShapeFlags(shape, shape.flags, MeshSimulationAllowed(owner));
	}

	void ReapplyShapeFlagsForLink(ShapeObj& shape)
	{
		ApplyShapeFlags(shape, shape.flags, false);
	}

	void UpdateMeshShapeSimulation(BodyObj& body, bool kinematic)
	{
		for (ShapeObj* shape : body.shapes)
		{
			if (IsMeshRestricted(shape->geometry.type))
			{
				ApplyShapeFlags(*shape, shape->flags, kinematic);
			}
		}
	}

	void RefreshShapeGeometry(ShapeObj& shape)
	{
		GeometryBuilder builder;
		if (const physx::PxGeometry* geometry = builder.Build(shape.geometry))
		{
			shape.shape->setGeometry(*geometry);
		}
	}

	void RegisterShapeHandlers()
	{
		using S = pc::Shape;
		Bind(S::CreateShape, &CreateShape);
		Bind(S::DestroyShape, &DestroyShape);
		Bind(S::GetActorDescriptor, &GetActorDescriptor);
		Bind(S::GetUserData, &GetUserData);
		Bind(S::GetFlags, &GetFlags);
		Bind(S::SetFlags, &SetFlags);
		Bind(S::GetFilterData, &GetFilterData);
		Bind(S::SetFilterData, &SetFilterData);
		Bind(S::SetMaterial, &SetMaterial);
		Bind(S::GetLocalPose, &GetLocalPose);
		Bind(S::GetPose, &GetPose);
		Bind(S::SetPose, &SetPose);
		Bind(S::SetContactOffset, &SetContactOffset);
		Bind(S::GetWorldBounds, &GetWorldBounds);
		Bind(S::GetBoundsAtPose, &GetBoundsAtPose);
		Bind(S::ShouldIgnoreCollision, &ShouldIgnoreCollision);
		Bind(S::IgnoreCollision, &IgnoreCollision);
		Bind(S::CleanupIgnoredColliders, &CleanupIgnoredColliders);
		Bind(S::GetGeometryType, &GetGeometryType);
		Bind(S::SetGeometry, &SetGeometry);
		Bind(S::GetGeometry, &GetGeometry);
	}
}
