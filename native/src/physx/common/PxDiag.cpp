#include "physx/common/PxDiag.h"

#include "core/Dispatch.h"
#include "physx/common/PxArticulation.h"

#include <cmath>
#include <cstdlib>
#include <unordered_map>

#include <unistd.h>

// Opt-in diagnostics (env BACKEND_DIAG=1): dumps every articulation once its structure has settled, then reports
// links that move implausibly fast. Meant for chasing instability that only shows up in real scenes.

namespace backend::pxw
{
	namespace
	{
		struct ArtState
		{
			size_t linkCount = 0;
			int stableSteps = 0;
			size_t dumpedCount = 0;
		};

		// Env BACKEND_DIAG, or a file named BACKEND_DIAG in the working directory (the project folder when launched
		// from Unity Hub); the file is re-checked while running, so it can be created without restarting.
		bool Enabled()
		{
			static const bool env = std::getenv("BACKEND_DIAG") != nullptr;
			return env || access("BACKEND_DIAG", F_OK) == 0;
		}

		const char* MotionName(artic::Motion m)
		{
			switch (m)
			{
			case artic::Motion::Locked: return "Locked";
			case artic::Motion::Limited: return "Limited";
			case artic::Motion::Free: return "Free";
			}
			return "?";
		}

		const char* GeometryName(Physics::GeometryType type)
		{
			switch (type)
			{
			case Physics::GeometryType::Sphere: return "sphere";
			case Physics::GeometryType::Capsule: return "capsule";
			case Physics::GeometryType::Box: return "box";
			case Physics::GeometryType::ConvexMesh: return "convex";
			case Physics::GeometryType::TriangleMesh: return "trimesh";
			case Physics::GeometryType::Terrain: return "terrain";
			default: return "other";
			}
		}

		int IndexOf(const ArticulationObj& a, const LinkObj* l)
		{
			for (size_t i = 0; i < a.links.size(); ++i)
			{
				if (a.links[i] == l)
				{
					return static_cast<int>(i);
				}
			}
			return -1;
		}

		void Dump(const ArticulationObj& a, int artIndex)
		{
			BACKEND_LOGF("[diag] articulation #%d links=%zu iterations=%u/%u flags=%u", artIndex, a.links.size(),
				a.positionIterations, a.velocityIterations, static_cast<unsigned>(a.flags));
			for (size_t i = 0; i < a.links.size(); ++i)
			{
				const LinkObj& l = *a.links[i];
				if (l.link == nullptr)
				{
					continue;
				}
				const physx::PxTransform gp = l.link->getGlobalPose();
				const physx::PxVec3 inertia = l.link->getMassSpaceInertiaTensor();
				const physx::PxTransform com = l.link->getCMassLocalPose();
				BACKEND_LOGF("[diag]  link %zu parent=%d jointType=%d mass=%g inertia=(%g,%g,%g) com=(%g,%g,%g) "
					"damping=%g/%g maxJointVel=%g friction=%g pos=(%.4f,%.4f,%.4f) shapes=%zu",
					i, IndexOf(a, l.parent), static_cast<int>(l.jointType), l.link->getMass(), inertia.x, inertia.y,
					inertia.z, com.p.x, com.p.y, com.p.z, l.linearDamping, l.angularDamping, l.maxJointVelocity,
					l.friction, gp.p.x, gp.p.y, gp.p.z, l.shapes.size());
				if (l.parent != nullptr)
				{
					const physx::PxTransform pf = ToPx(l.parentFrame);
					const physx::PxTransform cf = ToPx(l.childFrame);
					BACKEND_LOGF("[diag]    frames parent=(%.4f,%.4f,%.4f | %.3f,%.3f,%.3f,%.3f) child=(%.4f,%.4f,%.4f | %.3f,%.3f,%.3f,%.3f)",
						pf.p.x, pf.p.y, pf.p.z, pf.q.x, pf.q.y, pf.q.z, pf.q.w, cf.p.x, cf.p.y, cf.p.z, cf.q.x, cf.q.y, cf.q.z,
						cf.q.w);
					for (size_t axis = 0; axis < l.axes.size(); ++axis)
					{
						const AxisConfig& cfg = l.axes[axis];
						if (cfg.motion == artic::Motion::Locked)
						{
							continue;
						}
						BACKEND_LOGF("[diag]    axis %zu %s limits=[%g,%g] drive stiffness=%g damping=%g forceLimit=%g target=%g "
							"targetVel=%g type=%d",
							axis, MotionName(cfg.motion), cfg.lowerLimit, cfg.upperLimit, cfg.motor.stiffness, cfg.motor.damping,
							cfg.motor.forceLimit, cfg.motor.target, cfg.motor.targetVelocity, static_cast<int>(cfg.motor.type));
					}
				}
				for (const ShapeObj* shape : l.shapes)
				{
					const physx::PxBounds3 b = physx::PxShapeExt::getWorldBounds(*shape->shape, *l.link);
					const physx::PxVec3 e = b.getExtents();
					BACKEND_LOGF("[diag]    shape %s extents=(%.4f,%.4f,%.4f) center=(%.4f,%.4f,%.4f)",
						GeometryName(shape->geometry.type), e.x, e.y, e.z, b.getCenter().x, b.getCenter().y,
						b.getCenter().z);
				}
			}
		}
	}

	void DiagStep(WorldObj& world)
	{
		static unsigned long long step = 0;
		static bool enabled = false;
		if (step % 100 == 0)
		{
			enabled = Enabled();
		}
		++step;
		if (!enabled)
		{
			return;
		}

		static std::unordered_map<const ArticulationObj*, ArtState> states;
		static int anomalyLines = 0;

		int artIndex = 0;
		for (const ArticulationObj* a : world.articulations)
		{
			ArtState& s = states[a];
			if (s.linkCount != a->links.size())
			{
				s.linkCount = a->links.size();
				s.stableSteps = 0;
			}
			else if (s.stableSteps < 1000000)
			{
				++s.stableSteps;
			}
			if (s.stableSteps == 30 && s.dumpedCount != s.linkCount)
			{
				s.dumpedCount = s.linkCount;
				Dump(*a, artIndex);
			}

			float worstSpeed = 0.0f;
			size_t worstLink = 0;
			for (size_t i = 0; i < a->links.size(); ++i)
			{
				const LinkObj& l = *a->links[i];
				if (l.link == nullptr || !a->inPxScene)
				{
					continue;
				}
				const physx::PxVec3 v = l.link->getLinearVelocity();
				const physx::PxVec3 w = l.link->getAngularVelocity();
				const float speed = v.magnitude();
				const float spin = w.magnitude();
				const bool bad = !std::isfinite(speed) || !std::isfinite(spin);
				if (bad || speed > worstSpeed)
				{
					worstSpeed = bad ? INFINITY : speed;
					worstLink = i;
				}
				if ((bad || speed > 3.0f || spin > 30.0f) && anomalyLines < 200)
				{
					++anomalyLines;
					const physx::PxTransform gp = l.link->getGlobalPose();
					BACKEND_WARNF("[diag] step %llu art #%d link %zu speed=%g spin=%g pos=(%.3f,%.3f,%.3f)", step, artIndex, i,
						speed, spin, gp.p.x, gp.p.y, gp.p.z);
				}
			}
			if (step % 200 == 0 && !a->links.empty())
			{
				const physx::PxTransform root = a->links[0]->link != nullptr ? a->links[0]->link->getGlobalPose() : physx::PxTransform(physx::PxIdentity);
				BACKEND_LOGF("[diag] step %llu art #%d inScene=%d links=%zu root=(%.3f,%.3f,%.3f) worstSpeed=%g (link %zu)", step,
					artIndex, a->inPxScene, a->links.size(), root.p.x, root.p.y, root.p.z, worstSpeed, worstLink);
			}
			++artIndex;
		}
	}
}
