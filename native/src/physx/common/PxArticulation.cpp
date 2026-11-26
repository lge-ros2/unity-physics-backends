#include "physx/common/PxArticulation.h"
#include "physx/common/PxEvents.h"

#include "core/Dispatch.h"
#include "physx/common/PhysXSdk.h"
#include "physx/common/PxFilter.h"

#include "PhysicsWorldCommandData.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <unordered_set>

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace wd = PhysicsCommands::WorldData;
	namespace ald = PhysicsCommands::ArticulationLinkData;
	namespace ajd = PhysicsCommands::ArticulationJointData;
	namespace ad = PhysicsCommands::ArticulationData;

	using artic::JointData;
	using artic::Motion;
	using Physics::BodyFlags;
	using Physics::JointType;

	namespace
	{
		constexpr float kUnlimited = 1e20f;
		constexpr float kPi = 3.14159265358979f;

		void WarnOnce(std::atomic<bool>& flag, const char* what)
		{
			if (!flag.exchange(true))
			{
				BACKEND_WARNF("%s", what);
			}
		}

		// ---- float buffers handed to Unity (sdkOwnedMemory) ----

		std::mutex g_bufferMutex;
		std::unordered_set<const float*> g_buffers;

		Physics::FloatBuffer MakeBuffer(const std::vector<float>& values)
		{
			Physics::FloatBuffer buffer;
			buffer.data = nullptr;
			buffer.size = 0;
			buffer.sdkOwnedMemory = false;
			if (values.empty())
			{
				return buffer;
			}
			float* data = new float[values.size()];
			std::memcpy(data, values.data(), values.size() * sizeof(float));
			{
				std::lock_guard<std::mutex> g(g_bufferMutex);
				g_buffers.insert(data);
			}
			buffer.data = data;
			buffer.size = values.size();
			buffer.sdkOwnedMemory = true;
			return buffer;
		}

		void ReleaseBuffer(const float* data)
		{
			if (data == nullptr)
			{
				return;
			}
			{
				std::lock_guard<std::mutex> g(g_bufferMutex);
				if (g_buffers.erase(data) == 0)
				{
					return; // not ours
				}
			}
			delete[] data;
		}

		physx::PxForceMode::Enum ToPxForceMode(Physics::ForceMode mode)
		{
			switch (mode)
			{
			case Physics::ForceMode::Impulse: return physx::PxForceMode::eIMPULSE;
			case Physics::ForceMode::VelocityChange: return physx::PxForceMode::eVELOCITY_CHANGE;
			case Physics::ForceMode::Acceleration: return physx::PxForceMode::eACCELERATION;
			default: return physx::PxForceMode::eFORCE;
			}
		}

		// ---- joint configuration helpers ----

		int AxisIndex(Physics::ConstraintAxis axis)
		{
			switch (axis)
			{
			case Physics::ConstraintAxis::AngularX: return 0;
			case Physics::ConstraintAxis::AngularY: return 1;
			case Physics::ConstraintAxis::AngularZ: return 2;
			case Physics::ConstraintAxis::LinearX: return 3;
			case Physics::ConstraintAxis::LinearY: return 4;
			case Physics::ConstraintAxis::LinearZ: return 5;
			default: return -1;
			}
		}

		// Which axes a joint type can use: revolute = twist, spherical = twist + two swings, prismatic = x / y / z.
		bool AxisValid(JointType type, int axis)
		{
			switch (type)
			{
			case JointType::Hinge: return axis == 0;
			case JointType::BallAndSocket: return axis >= 0 && axis <= 2;
			case JointType::Prismatic: return axis >= 3 && axis <= 5;
			default: return false;
			}
		}

		Motion EffectiveMotionOf(JointType type, int axis, const AxisConfig& cfg)
		{
			if (!AxisValid(type, axis))
			{
				return Motion::Locked;
			}
			if (cfg.motion == Motion::Limited && !(cfg.lowerLimit < cfg.upperLimit))
			{
				return Motion::Locked;
			}
			return cfg.motion;
		}

		Motion EffectiveMotion(const LinkObj& l, int axis)
		{
			return EffectiveMotionOf(l.jointType, axis, l.axes[axis]);
		}

		uint32_t DofCount(const LinkObj& l)
		{
			uint32_t count = 0;
			for (int axis = 0; axis < artic::kAxisCount; ++axis)
			{
				if (EffectiveMotion(l, axis) != Motion::Locked)
				{
					++count;
				}
			}
			return count;
		}

		float LimitBound(JointType type)
		{
			if (type == JointType::BallAndSocket)
			{
				return kPi;
			}
			if (type == JointType::Hinge && compat::kPhysXMajor < 5)
			{
				return 2.0f * kPi; // wrapped revolute joints of PhysX 4.1
			}
			return kUnlimited;
		}

		float SafeForceLimit(float value)
		{
			if (!(value > 0.0f))
			{
				return 0.0f;
			}
			return std::min(value, kUnlimited);
		}

		bool Alive(const LinkObj* l)
		{
			return l != nullptr && !l->dead && l->link != nullptr;
		}

		// ---- articulation state (callers hold the world domain) ----

		void InvalidateCache(ArticulationObj& a)
		{
			if (a.cache != nullptr)
			{
				artic::ReleaseCache(*a.art, a.cache);
				a.cache = nullptr;
			}
		}

		artic::Cache* EnsureCache(ArticulationObj& a)
		{
			if (!a.inPxScene)
			{
				return nullptr;
			}
			if (a.cache == nullptr)
			{
				a.cache = artic::CreateCache(*a.art);
			}
			return a.cache;
		}

		// Link DOF layout: (link, first flat DOF index) in creation order, plus the total DOF count.
		std::vector<std::pair<LinkObj*, uint32_t>> DofLayout(const ArticulationObj& a, uint32_t& total)
		{
			std::vector<std::pair<LinkObj*, uint32_t>> layout;
			total = 0;
			// PhysX lays the cache out in its own link index order (children grouped per level), which differs from the
			// creation order when a link has siblings with children of their own (suspension arm + wheel on each side).
			std::vector<LinkObj*> ordered(a.links.begin(), a.links.end());
			if (a.inPxScene)
			{
				std::stable_sort(ordered.begin(), ordered.end(), [](const LinkObj* x, const LinkObj* y) {
					const uint32_t ix = x->link != nullptr ? x->link->getLinkIndex() : ~0u;
					const uint32_t iy = y->link != nullptr ? y->link->getLinkIndex() : ~0u;
					return ix < iy;
				});
			}
			for (LinkObj* l : ordered)
			{
				layout.emplace_back(l, total);
				total += DofCount(*l);
			}
			return layout;
		}

		PendingJointState& Pending(LinkObj& l, JointData what)
		{
			switch (what)
			{
			case JointData::Position: return l.pendingPosition;
			case JointData::Velocity: return l.pendingVelocity;
			default: return l.pendingForce;
			}
		}

		// Period of the position wrap of a joint, 0 when PhysX does not wrap it.
		float UnwrapPeriod(const LinkObj& l)
		{
			if (l.jointType == JointType::BallAndSocket)
			{
				return 2.0f * kPi;
			}
			if (l.jointType == JointType::Hinge && compat::kPhysXMajor < 5)
			{
				return 4.0f * kPi;
			}
			return 0.0f;
		}

		bool ReadFlatRaw(ArticulationObj& a, JointData what, std::vector<float>& out);

		// Flat per-DOF values of the whole articulation (PhysX cache DOF order). False if unavailable.
		// Positions are continuous: the wrap offsets are added (see UpdateJointUnwrapOf).
		bool ReadFlat(ArticulationObj& a, JointData what, std::vector<float>& out)
		{
			if (!ReadFlatRaw(a, what, out))
			{
				return false;
			}
			if (what == JointData::Position && a.inPxScene)
			{
				uint32_t total = 0;
				for (const auto& entry : DofLayout(a, total))
				{
					if (UnwrapPeriod(*entry.first) <= 0.0f)
					{
						continue;
					}
					const uint32_t n = std::min(DofCount(*entry.first), 3u);
					for (uint32_t i = 0; i < n && entry.second + i < out.size(); ++i)
					{
						out[entry.second + i] += entry.first->unwrapOffset[i];
					}
				}
			}
			return true;
		}

		// Advances the wrap offsets from the raw positions; call after every step.
		void UpdateJointUnwrapOf(ArticulationObj& a)
		{
			if (!a.inPxScene)
			{
				return;
			}
			bool any = false;
			for (const LinkObj* l : a.links)
			{
				any = any || (UnwrapPeriod(*l) > 0.0f && DofCount(*l) > 0);
			}
			if (!any)
			{
				return;
			}
			std::vector<float> raw;
			if (!ReadFlatRaw(a, JointData::Position, raw))
			{
				return;
			}
			uint32_t total = 0;
			for (const auto& entry : DofLayout(a, total))
			{
				LinkObj& l = *entry.first;
				const float period = UnwrapPeriod(l);
				if (period <= 0.0f)
				{
					continue;
				}
				const uint32_t n = std::min(DofCount(l), 3u);
				for (uint32_t i = 0; i < n && entry.second + i < raw.size(); ++i)
				{
					const float value = raw[entry.second + i];
					if (l.unwrapInit)
					{
						const float jumps = std::round((value - l.unwrapLast[i]) / period);
						l.unwrapOffset[i] -= jumps * period;
					}
					l.unwrapLast[i] = value;
				}
				l.unwrapInit = true;
			}
		}

		// Positions written by the caller are continuous values: wraps them into what PhysX stores and restarts the
		// offsets so that reading them back returns the written value.
		void AdjustWrittenPositions(ArticulationObj& a, size_t start, std::vector<float>& values)
		{
			uint32_t total = 0;
			for (const auto& entry : DofLayout(a, total))
			{
				LinkObj& l = *entry.first;
				const float period = UnwrapPeriod(l);
				const uint32_t n = std::min(DofCount(l), 3u);
				if (period <= 0.0f)
				{
					continue;
				}
				for (uint32_t i = 0; i < n; ++i)
				{
					const size_t global = entry.second + i;
					if (global < start || global - start >= values.size())
					{
						continue;
					}
					float& v = values[global - start];
					const float raw = v - period * std::round(v / period);
					l.unwrapOffset[i] = v - raw;
					l.unwrapLast[i] = raw;
					v = raw;
				}
				l.unwrapInit = true;
			}
		}

		bool ReadFlatRaw(ArticulationObj& a, JointData what, std::vector<float>& out)
		{
			uint32_t total = 0;
			const auto layout = DofLayout(a, total);
			if (a.inPxScene)
			{
				artic::Cache* cache = EnsureCache(a);
				if (cache == nullptr)
				{
					return false;
				}
				const uint32_t dofs = a.art->getDofs();
				artic::CopyState(*a.art, *cache, what);
				const float* data = artic::Data(*cache, what);
				out.assign(data, data + dofs);
				return true;
			}
			out.assign(total, 0.0f);
			if (what == JointData::Acceleration)
			{
				return true;
			}
			for (const auto& entry : layout)
			{
				const PendingJointState& p = Pending(*entry.first, what);
				const uint32_t n = DofCount(*entry.first);
				for (uint32_t i = 0; i < n && i < 3; ++i)
				{
					out[entry.second + i] = p.values[i];
				}
			}
			return true;
		}

		// Writes n values at flat DOF offset start.
		void WriteFlat(ArticulationObj& a, JointData what, size_t start, const float* values, size_t n)
		{
			if (a.inPxScene)
			{
				artic::Cache* cache = EnsureCache(a);
				if (cache == nullptr)
				{
					return;
				}
				const size_t dofs = a.art->getDofs();
				artic::CopyState(*a.art, *cache, what);
				float* data = artic::Data(*cache, what);
				std::vector<float> adjusted(values, values + n);
				if (what == JointData::Position)
				{
					AdjustWrittenPositions(a, start, adjusted);
				}
				for (size_t i = 0; i < n && start + i < dofs; ++i)
				{
					data[start + i] = adjusted[i];
				}
				artic::ApplyState(*a.art, *cache, what);
				return;
			}
			if (what == JointData::Acceleration)
			{
				return;
			}
			uint32_t total = 0;
			for (const auto& entry : DofLayout(a, total))
			{
				PendingJointState& p = Pending(*entry.first, what);
				const uint32_t count = DofCount(*entry.first);
				for (uint32_t i = 0; i < count && i < 3; ++i)
				{
					const size_t idx = entry.second + i;
					if (idx >= start && idx < start + n)
					{
						p.values[i] = values[idx - start];
						p.set = true;
					}
				}
			}
		}

		// Applies state that was stored while the articulation had no scene.
		void ApplyPending(ArticulationObj& a)
		{
			artic::Cache* cache = EnsureCache(a);
			if (cache == nullptr)
			{
				return;
			}
			uint32_t total = 0;
			const auto layout = DofLayout(a, total);
			const uint32_t dofs = a.art->getDofs();
			for (JointData what : { JointData::Position, JointData::Velocity, JointData::Force })
			{
				bool any = false;
				for (const auto& entry : layout)
				{
					any = any || Pending(*entry.first, what).set;
				}
				if (!any)
				{
					continue;
				}
				artic::CopyState(*a.art, *cache, what);
				float* data = artic::Data(*cache, what);
				for (const auto& entry : layout)
				{
					PendingJointState& p = Pending(*entry.first, what);
					if (!p.set)
					{
						continue;
					}
					const uint32_t n = DofCount(*entry.first);
					if (n == 0)
					{
						continue; // a locked joint keeps its stored state until it gets DOFs again
					}
					for (uint32_t i = 0; i < n && i < 3 && entry.second + i < dofs; ++i)
					{
						data[entry.second + i] = p.values[i];
					}
					p.set = false;
				}
				artic::ApplyState(*a.art, *cache, what);
			}
		}

		// Moves the joint state of every link into its pending state, so it survives a trip outside the scene
		// even when the DOF layout changes meanwhile (PhysX recomputes link poses from stored joint positions on add).
		void SaveState(ArticulationObj& a)
		{
			if (!a.inPxScene)
			{
				return;
			}
			uint32_t total = 0;
			const auto layout = DofLayout(a, total);
			for (JointData what : { JointData::Position, JointData::Velocity })
			{
				std::vector<float> flat;
				if (!ReadFlat(a, what, flat))
				{
					continue;
				}
				for (const auto& entry : layout)
				{
					const uint32_t n = std::min(DofCount(*entry.first), 3u);
					PendingJointState& p = Pending(*entry.first, what);
					for (uint32_t i = 0; i < n && entry.second + i < flat.size(); ++i)
					{
						p.values[i] = flat[entry.second + i];
						p.set = true;
					}
				}
			}
		}

		void AddToPxScene(ArticulationObj& a)
		{
			if (a.inPxScene || a.art == nullptr || a.world == nullptr || a.world->scene == nullptr || a.links.empty()
				|| a.art->getNbLinks() == 0)
			{
				return;
			}
			artic::AddToScene(*a.world->scene, *a.art);
			a.inPxScene = a.art->getScene() != nullptr;
			InvalidateCache(a);
			if (!a.inPxScene)
			{
				BACKEND_ERRORF("Articulation could not be added to the scene");
				return;
			}
			ApplyPending(a);
			// An articulation re-entering the scene (structure change, simulated toggle) must not start asleep.
			a.art->wakeUp();
		}

		void RemoveFromPxScene(ArticulationObj& a, bool keepState)
		{
			if (!a.inPxScene)
			{
				return;
			}
			if (keepState)
			{
				SaveState(a);
			}
			InvalidateCache(a);
			artic::RemoveFromScene(*a.world->scene, *a.art);
			a.inPxScene = false;
		}

		void SyncScene(ArticulationObj& a)
		{
			if (a.added && a.simulated && !a.links.empty())
			{
				AddToPxScene(a);
			}
			else
			{
				RemoveFromPxScene(a, true);
			}
		}

		void ApplyArticulationFlags(ArticulationObj& a)
		{
			artic::SetBaseFlags(*a.art,
				(a.flags & Physics::ArticulationFlags::ImmovableRoot) != Physics::ArticulationFlags::None,
				(a.flags & Physics::ArticulationFlags::DriveLimitsAsForces) != Physics::ArticulationFlags::None);
		}

		void WakeIfInScene(ArticulationObj& a)
		{
			if (a.inPxScene)
			{
				a.art->wakeUp();
			}
		}

		void ApplyAxis(LinkObj& l, int axis)
		{
			if (l.joint == nullptr)
			{
				return;
			}
			const AxisConfig& cfg = l.axes[axis];
			const Motion motion = EffectiveMotion(l, axis);
			if (!l.art->inPxScene)
			{
				// PhysX only accepts motion changes outside a scene (callers go through ApplyChange).
				artic::SetMotion(*l.joint, axis, motion);
			}
			if (motion == Motion::Locked)
			{
				return;
			}
			if (motion == Motion::Limited)
			{
				const float bound = LimitBound(l.jointType);
				artic::SetLimit(*l.joint, axis, std::max(cfg.lowerLimit, -bound), std::min(cfg.upperLimit, bound));
			}
			artic::DriveParams drive;
			drive.stiffness = std::max(cfg.motor.stiffness, 0.0f);
			drive.damping = std::max(cfg.motor.damping, 0.0f);
			drive.forceLimit = SafeForceLimit(cfg.motor.forceLimit);
			drive.type = cfg.motor.type;
			artic::SetDrive(*l.joint, axis, drive);
			artic::SetDriveTarget(*l.joint, axis, cfg.motor.target, cfg.motor.targetVelocity);
			WakeIfInScene(*l.art); // a sleeping articulation would ignore the new drive / limit (PhysX 4.1 does not wake on its own)
		}

		void ApplyJoint(LinkObj& l)
		{
			if (l.joint == nullptr)
			{
				return;
			}
			static std::atomic<bool> s_warned{ false };
			if (!artic::SetJointType(*l.joint, l.jointType))
			{
				WarnOnce(s_warned, "Articulation links only support Fixed, Prismatic, Hinge and BallAndSocket joints; other types are made fixed");
			}
			l.joint->setParentPose(ToPx(l.parentFrame));
			l.joint->setChildPose(ToPx(l.childFrame));
			for (int axis = 0; axis < artic::kAxisCount; ++axis)
			{
				ApplyAxis(l, axis);
			}
			artic::SetFriction(*l.joint, std::max(l.friction, 0.0f));
			artic::SetMaxVelocity(*l.joint, std::max(l.maxJointVelocity, 0.0f));
		}

		void ApplyLinkGravity(LinkObj& l)
		{
			l.link->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, (l.flags & BodyFlags::Gravity) == BodyFlags::None);
		}

		// Creates the PhysX link of a wrapper (parent first) and applies everything the wrapper remembers.
		bool CreatePxLink(ArticulationObj& a, LinkObj& l)
		{
			physx::PxArticulationLink* px = a.art->createLink(l.parent != nullptr ? l.parent->link : nullptr, l.pose);
			if (px == nullptr)
			{
				return false;
			}
			l.link = px;
			px->userData = &l;
			l.joint = l.parent != nullptr ? static_cast<artic::Joint*>(px->getInboundJoint()) : nullptr;

			if (!l.hasProps)
			{
				l.mass = px->getMass();
				l.inertia = px->getMassSpaceInertiaTensor();
				l.centerOfMass = px->getCMassLocalPose();
				l.linearDamping = px->getLinearDamping();
				l.angularDamping = px->getAngularDamping();
				l.maxLinearVelocity = px->getMaxLinearVelocity();
				l.maxAngularVelocity = px->getMaxAngularVelocity();
				l.maxDepenetrationVelocity = px->getMaxDepenetrationVelocity();
				l.hasProps = true;
			}
			else
			{
				if (l.mass > 0.0f)
				{
					px->setMass(l.mass);
				}
				px->setMassSpaceInertiaTensor(l.inertia);
				px->setCMassLocalPose(l.centerOfMass);
				px->setLinearDamping(std::max(l.linearDamping, 0.0f));
				px->setAngularDamping(std::max(l.angularDamping, 0.0f));
				px->setMaxLinearVelocity(l.maxLinearVelocity);
				px->setMaxAngularVelocity(l.maxAngularVelocity);
				px->setMaxDepenetrationVelocity(l.maxDepenetrationVelocity);
			}
			ApplyLinkGravity(l);
			ApplyJoint(l);
			for (ShapeObj* shape : l.shapes)
			{
				if (!px->attachShape(*shape->shape))
				{
					BACKEND_ERRORF("PxRigidActor::attachShape failed while rebuilding an articulation link");
				}
			}
			return true;
		}

		// ---- rebuilding the PhysX articulation ----
		//
		// PhysX only accepts structural changes (links, joint types, axis motion, joint frames) outside a scene,
		// and a articulation that was changed after having been in a scene does not simulate reliably. So any such
		// change on an articulation that is in the scene releases the PhysX objects and builds them again from the
		// wrappers, which hold the full configuration. Handles stay valid; joint state is carried over.

		void SnapshotLink(LinkObj& l)
		{
			if (!Alive(&l))
			{
				return;
			}
			l.pose = l.link->getGlobalPose();
			l.mass = l.link->getMass();
			l.inertia = l.link->getMassSpaceInertiaTensor();
			l.centerOfMass = l.link->getCMassLocalPose();
			l.linearDamping = l.link->getLinearDamping();
			l.angularDamping = l.link->getAngularDamping();
			l.maxLinearVelocity = l.link->getMaxLinearVelocity();
			l.maxAngularVelocity = l.link->getMaxAngularVelocity();
			l.maxDepenetrationVelocity = l.link->getMaxDepenetrationVelocity();
			l.hasProps = true;
		}

		// Releases the PhysX articulation (shapes stay owned by their wrappers).
		void TeardownPx(ArticulationObj& a)
		{
			InvalidateCache(a);
			for (LinkObj* l : a.links)
			{
				if (Alive(l))
				{
					for (ShapeObj* shape : l->shapes)
					{
						l->link->detachShape(*shape->shape);
					}
				}
			}
			a.art->release();
			a.art = nullptr;
			for (LinkObj* l : a.links)
			{
				l->link = nullptr;
				l->joint = nullptr;
			}
		}

		bool BuildPx(ArticulationObj& a)
		{
			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (physics == nullptr)
			{
				return false;
			}
			a.art = artic::CreateArticulation(*physics);
			if (a.art == nullptr)
			{
				BACKEND_ERRORF("PxPhysics::createArticulationReducedCoordinate failed");
				return false;
			}
			a.art->userData = &a;
			ApplyArticulationFlags(a);
			a.art->setSleepThreshold(a.sleepThreshold);
			a.art->setSolverIterationCounts(a.positionIterations, a.velocityIterations);
			bool ok = true;
			for (LinkObj* l : a.links)
			{
				ok = CreatePxLink(a, *l) && ok;
			}
			return ok;
		}

		// config() edits the wrappers (links, joint settings); the PhysX side is rebuilt around it.
		template <typename Config>
		void RebuildScoped(ArticulationObj& a, Config&& config)
		{
			SaveState(a); // needs the DOF layout PhysX currently has, i.e. before config()
			for (LinkObj* l : a.links)
			{
				SnapshotLink(*l);
			}
			RemoveFromPxScene(a, false);
			TeardownPx(a);
			config();
			if (!BuildPx(a))
			{
				BACKEND_ERRORF("Rebuilding an articulation failed");
			}
			SyncScene(a);
		}

		// Same for an articulation outside the scene (the root pose can only be chosen when the root link is created).
		template <typename Config>
		void RebuildUnscened(ArticulationObj& a, Config&& config)
		{
			for (LinkObj* l : a.links)
			{
				SnapshotLink(*l);
			}
			TeardownPx(a);
			config();
			if (!BuildPx(a))
			{
				BACKEND_ERRORF("Rebuilding an articulation failed");
			}
		}

		// A change outside the scene is applied to the live PhysX objects, inside the scene it rebuilds them.
		template <typename Config, typename Incremental>
		void ApplyChange(ArticulationObj& a, Config&& config, Incremental&& incremental)
		{
			if (a.inPxScene)
			{
				RebuildScoped(a, config);
				return;
			}
			config();
			incremental();
			InvalidateCache(a);
		}

		// Changes the config of one axis; a change of its effective motion is structural, the rest is live.
		template <typename F>
		void ChangeAxis(LinkObj& l, int axis, F&& change)
		{
			AxisConfig next = l.axes[axis];
			change(next);
			if (l.joint == nullptr)
			{
				l.axes[axis] = next;
				return;
			}
			if (EffectiveMotion(l, axis) != EffectiveMotionOf(l.jointType, axis, next))
			{
				ApplyChange(*l.art, [&] { l.axes[axis] = next; }, [&] { ApplyAxis(l, axis); });
			}
			else
			{
				l.axes[axis] = next;
				ApplyAxis(l, axis);
			}
		}

		void DetachLinkShapes(LinkObj& l)
		{
			for (ShapeObj* shape : l.shapes)
			{
				NotifyShapeRemoved(*shape);
				if (Alive(&l))
				{
					l.link->detachShape(*shape->shape);
				}
				shape->ownerLink = nullptr;
				shape->shape->setSimulationFilterData(EncodeShapeFilter(*shape));
				Rebind(shape, LooseState());
			}
			l.shapes.clear();
		}

		void FreeLinkWrapper(LinkObj* l)
		{
			if (l->jointHandle != nullptr)
			{
				Unregister(l->jointHandle);
				delete l->jointHandle;
			}
			Unregister(l);
			delete l;
		}

		void CollectSubtree(LinkObj* l, std::vector<LinkObj*>& postOrder)
		{
			for (LinkObj* child : l->children)
			{
				CollectSubtree(child, postOrder);
			}
			postOrder.push_back(l);
		}

		void DestroyLinkLocked(LinkObj* l)
		{
			ArticulationObj* a = l->art;
			if (l->dead)
			{
				// Already released together with an ancestor; only the wrapper is left.
				a->graveyard.erase(std::remove(a->graveyard.begin(), a->graveyard.end(), l), a->graveyard.end());
				FreeLinkWrapper(l);
				return;
			}

			std::vector<LinkObj*> subtree;
			CollectSubtree(l, subtree);

			if (l->parent != nullptr)
			{
				auto& siblings = l->parent->children;
				siblings.erase(std::remove(siblings.begin(), siblings.end(), l), siblings.end());
			}
			// The links of the subtree leave the articulation; their handles stay until DestroyLink is called on them.
			auto detach = [&] {
				for (LinkObj* s : subtree)
				{
					DetachLinkShapes(*s);
					a->links.erase(std::remove(a->links.begin(), a->links.end(), s), a->links.end());
					s->link = nullptr;
					s->joint = nullptr;
					s->parent = nullptr;
					s->children.clear();
					s->dead = true;
					if (s != l)
					{
						a->graveyard.push_back(s);
					}
				}
			};
			if (a->inPxScene)
			{
				RebuildScoped(*a, detach);
			}
			else
			{
				for (LinkObj* s : subtree) // leaves first
				{
					DetachLinkShapes(*s);
					s->link->release();
					s->link = nullptr;
				}
				detach();
				InvalidateCache(*a);
			}
			FreeLinkWrapper(l);
		}

		// ---- handle resolution ----

		Handle PickLink(Handle sdkObject, const Context& ctx, Handle extra = nullptr)
		{
			Handle h = Pick(Kind::Link, ctx.body, sdkObject, extra);
			if (h != nullptr)
			{
				return h;
			}
			// The joint handle of a link stands for the link itself.
			const Handle joint = Pick(Kind::LinkJoint, sdkObject, ctx.body);
			if (joint != nullptr)
			{
				const auto* obj = static_cast<const LinkJointObj*>(Lookup(joint, Kind::LinkJoint));
				return obj != nullptr ? obj->link : nullptr;
			}
			return nullptr;
		}

		// Locks the world domain of the link addressed by an Articulation link / joint command.
		class LinkScope
		{
		public:
			LinkScope(Handle sdkObject, const Context& ctx, Handle extra = nullptr)
				: m_lock(Target{ Kind::Link, PickLink(sdkObject, ctx, extra) })
			{
			}

			// Found, whether or not its PhysX counterpart still exists.
			bool found() const { return static_cast<bool>(m_lock); }
			explicit operator bool() const { return found() && Alive(get()); }
			LinkObj* get() const { return m_lock ? m_lock.get<LinkObj>() : nullptr; }
			LinkObj* operator->() const { return get(); }
			LinkObj& operator*() const { return *get(); }

		private:
			Lock m_lock;
		};

		// Locks the world domain of the articulation addressed by an Articulation command.
		class ArtScope
		{
		public:
			ArtScope(Handle sdkObject, const Context& ctx, Handle extraLink = nullptr)
				: m_viaLink(false)
				, m_lock(Choose(sdkObject, ctx, extraLink, m_viaLink))
			{
			}

			explicit operator bool() const { return static_cast<bool>(m_lock); }
			ArticulationObj* get() const
			{
				if (!m_lock)
				{
					return nullptr;
				}
				return m_viaLink ? m_lock.get<LinkObj>()->art : m_lock.get<ArticulationObj>();
			}
			ArticulationObj* operator->() const { return get(); }
			ArticulationObj& operator*() const { return *get(); }

		private:
			static Target Choose(Handle sdkObject, const Context& ctx, Handle extraLink, bool& viaLink)
			{
				const Handle art = Pick(Kind::Articulation, sdkObject, ctx.body, ctx.world);
				if (art != nullptr)
				{
					return Target{ Kind::Articulation, art };
				}
				viaLink = true;
				return Target{ Kind::Link, PickLink(sdkObject, ctx, extraLink) };
			}

			bool m_viaLink;
			Lock m_lock;
		};

		// ---- world commands ----

		void CreateArticulation(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::CreateArticulation&>(c);
			cmd.outArticulation = nullptr;

			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (!lock || physics == nullptr)
			{
				return;
			}
			WorldObj* world = lock.get<WorldObj>();

			artic::Articulation* px = artic::CreateArticulation(*physics);
			if (px == nullptr)
			{
				BACKEND_ERRORF("PxPhysics::createArticulationReducedCoordinate failed");
				return;
			}
			auto* a = new ArticulationObj();
			a->art = px;
			a->world = world;
			a->userData = cmd.userData;
			a->flags = cmd.flags;
			a->collisionGroup = NextArticulationCollisionGroup();
			px->userData = a;
			ApplyArticulationFlags(*a);
			a->sleepThreshold = px->getSleepThreshold();
			physx::PxU32 pos = 0;
			physx::PxU32 vel = 0;
			px->getSolverIterationCounts(pos, vel);
			a->positionIterations = pos;
			a->velocityIterations = vel;

			world->articulations.push_back(a);
			Register(a, Kind::Articulation, a, world->state);
			// Unity 6000.6.7f1 never sends AddActorsToScene for articulations either: they join the scene on creation.
			a->added = true;
			cmd.outArticulation = a;
		}

		void DestroyArticulation(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Handle target = static_cast<wd::DestoryArticulation&>(c).value;
			Lock lock(Target{ Kind::Articulation, target });
			if (lock)
			{
				DestroyArticulationLocked(lock.handle(), lock.get<ArticulationObj>());
			}
			(void)sdkObject;
			(void)ctx;
		}

		void GetArticulationCount(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetArticulationCount&>(c);
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			cmd.value = lock ? static_cast<uint32_t>(lock.get<WorldObj>()->articulations.size()) : 0;
		}

		void GetArticulations(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<wd::GetArticulations&>(c);
			cmd.written = 0;
			Lock lock(Target{ Kind::World, sdkObject, ctx.world });
			if (!lock || cmd.buffer == nullptr)
			{
				return;
			}
			for (ArticulationObj* a : lock.get<WorldObj>()->articulations)
			{
				if (cmd.written >= cmd.bufferSize)
				{
					break;
				}
				cmd.buffer[cmd.written++] = a;
			}
		}

		// ---- articulation commands ----

		void CreateLink(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::CreateLink&>(c);
			cmd.outNewLink = nullptr;

			ArtScope a(sdkObject, ctx, cmd.parentLink);
			if (!a)
			{
				return;
			}
			ArticulationObj* art = a.get();

			LinkObj* parent = nullptr;
			if (cmd.parentLink != nullptr)
			{
				parent = static_cast<LinkObj*>(Lookup(cmd.parentLink, Kind::Link));
				if (!Alive(parent) || parent->art != art)
				{
					BACKEND_ERRORF("Articulation::CreateLink: parent link is not a link of this articulation");
					return;
				}
			}
			else if (!art->links.empty())
			{
				BACKEND_ERRORF("Articulation::CreateLink: the articulation already has a root link");
				return;
			}

			auto* l = new LinkObj();
			l->art = art;
			l->parent = parent;
			l->userData = cmd.userData;
			l->pose = ToPx(cmd.pose);

			auto attach = [&] {
				art->links.push_back(l);
				if (parent != nullptr)
				{
					parent->children.push_back(l);
				}
			};
			if (art->inPxScene)
			{
				RebuildScoped(*art, attach);
			}
			else if (CreatePxLink(*art, *l))
			{
				attach();
				InvalidateCache(*art);
			}
			if (l->link == nullptr)
			{
				BACKEND_ERRORF("PxArticulationReducedCoordinate::createLink failed (more than 64 links?)");
				auto& links = art->links;
				links.erase(std::remove(links.begin(), links.end(), l), links.end());
				if (parent != nullptr)
				{
					parent->children.erase(std::remove(parent->children.begin(), parent->children.end(), l), parent->children.end());
				}
				delete l;
				return;
			}

			l->jointHandle = new LinkJointObj{ l };
			Register(l, Kind::Link, l, art->world->state);
			Register(l->jointHandle, Kind::LinkJoint, l->jointHandle, art->world->state);
			cmd.outNewLink = l;

			if (art->added)
			{
				SyncScene(*art);
			}
		}

		void DestroyLink(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Handle target = static_cast<ad::DestroyLink&>(c).value;
			Lock lock(Target{ Kind::Link, target });
			if (lock)
			{
				DestroyLinkLocked(lock.get<LinkObj>());
			}
			(void)sdkObject;
			(void)ctx;
		}

		void GetLinkCount(Handle sdkObject, const Context& ctx, Command& c)
		{
			ArtScope a(sdkObject, ctx);
			static_cast<ad::GetLinkCount&>(c).value = a ? a->links.size() : 0;
		}

		void GetLinksUserData(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinksUserData&>(c);
			cmd.written = 0;
			ArtScope a(sdkObject, ctx);
			if (!a || cmd.buffer == nullptr)
			{
				return;
			}
			// Creation order already is the link index order.
			for (const LinkObj* l : a->links)
			{
				if (cmd.written >= cmd.bufferSize)
				{
					break;
				}
				cmd.buffer[cmd.written++] = l->userData;
			}
		}

		void GetLinkParent(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkParent&>(c);
			Lock lock(Target{ Kind::Link, cmd.link });
			cmd.value = lock ? lock.get<LinkObj>()->parent : nullptr;
		}

		void GetLinkParentUserData(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkParentUserData&>(c);
			Lock lock(Target{ Kind::Link, cmd.link });
			const LinkObj* parent = lock ? lock.get<LinkObj>()->parent : nullptr;
			cmd.value = parent != nullptr ? parent->userData : nullptr;
		}

		void GetLinkChildCount(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkChildCount&>(c);
			Lock lock(Target{ Kind::Link, cmd.link });
			cmd.value = lock ? lock.get<LinkObj>()->children.size() : 0;
		}

		void GetLinkChildrenUserData(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<ad::GetLinkChildrenUserData&>(c);
			cmd.written = 0;
			Lock lock(Target{ Kind::Link, cmd.link });
			if (!lock || cmd.buffer == nullptr)
			{
				return;
			}
			for (const LinkObj* child : lock.get<LinkObj>()->children)
			{
				if (cmd.written >= cmd.bufferSize)
				{
					break;
				}
				cmd.buffer[cmd.written++] = child->userData;
			}
		}

		void SetArticulationFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::SetFlags&>(c);
			ArtScope a(sdkObject, ctx);
			if (!a)
			{
				return;
			}
			using Raw = std::underlying_type_t<Physics::ArticulationFlags>;
			const Raw bits = static_cast<Raw>(cmd.value);
			const Raw cur = static_cast<Raw>(a->flags);
			const Raw next = cmd.enabled ? (cur | bits) : (cur & ~bits);
			const bool baseChanged = ((next ^ cur) & (static_cast<Raw>(Physics::ArticulationFlags::ImmovableRoot)
				| static_cast<Raw>(Physics::ArticulationFlags::DriveLimitsAsForces))) != 0;
			if (baseChanged)
			{
				ApplyChange(*a, [&] { a->flags = static_cast<Physics::ArticulationFlags>(next); }, [&] { ApplyArticulationFlags(*a); });
			}
			else
			{
				a->flags = static_cast<Physics::ArticulationFlags>(next);
			}
			if ((bits & static_cast<Raw>(Physics::ArticulationFlags::Simulated)) != 0)
			{
				a->simulated = cmd.enabled;
				SyncScene(*a);
			}
		}

		void GetArticulationFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::GetFlags&>(c);
			ArtScope a(sdkObject, ctx);
			if (!a)
			{
				cmd.value = Physics::ArticulationFlags::None;
				return;
			}
			using Raw = std::underlying_type_t<Physics::ArticulationFlags>;
			const Raw simulated = static_cast<Raw>(Physics::ArticulationFlags::Simulated);
			const Raw raw = static_cast<Raw>(a->flags);
			cmd.value = static_cast<Physics::ArticulationFlags>(a->simulated ? (raw | simulated) : (raw & ~simulated));
		}

		void GetArticulationUserData(Handle sdkObject, const Context& ctx, Command& c)
		{
			ArtScope a(sdkObject, ctx);
			static_cast<ad::GetUserData&>(c).value = a ? a->userData : nullptr;
		}

		void GetArticulationIsSleeping(Handle sdkObject, const Context& ctx, Command& c)
		{
			ArtScope a(sdkObject, ctx);
			static_cast<ad::GetIsSleeping&>(c).value = a && a->inPxScene && a->art->isSleeping();
		}

		void SleepArticulation(Handle sdkObject, const Context& ctx, Command&)
		{
			ArtScope a(sdkObject, ctx);
			if (a && a->inPxScene)
			{
				a->art->putToSleep();
			}
		}

		void WakeUpArticulation(Handle sdkObject, const Context& ctx, Command&)
		{
			ArtScope a(sdkObject, ctx);
			if (a && a->inPxScene)
			{
				a->art->wakeUp();
			}
		}

		void SetSleepThresholdOverride(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = std::max(static_cast<ad::SetSleepThresholdOverride&>(c).value, 0.0f);
			ArtScope a(sdkObject, ctx);
			if (a)
			{
				a->sleepThreshold = value;
				a->art->setSleepThreshold(value);
			}
		}

		void GetSleepThresholdOverride(Handle sdkObject, const Context& ctx, Command& c)
		{
			ArtScope a(sdkObject, ctx);
			static_cast<ad::GetSleepThresholdOverride&>(c).value = a ? a->art->getSleepThreshold() : 0.0f;
		}

		void SetSolverIterationsOverride(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::SetSolverIterationsOverride&>(c);
			ArtScope a(sdkObject, ctx);
			if (a)
			{
				// PhysX rejects 0 iterations; the built-in backend then keeps the previous count, so a 0 is ignored.
				if (cmd.positionIterations > 0)
				{
					a->positionIterations = std::min(cmd.positionIterations, 255u);
				}
				if (cmd.velocityIterations > 0)
				{
					a->velocityIterations = std::min(cmd.velocityIterations, 255u);
				}
				a->art->setSolverIterationCounts(a->positionIterations, a->velocityIterations);
			}
		}

		void GetSolverIterationsOverride(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::GetSolverIterationsOverride&>(c);
			cmd.positionIterations = 0;
			cmd.velocityIterations = 0;
			ArtScope a(sdkObject, ctx);
			if (a)
			{
				physx::PxU32 pos = 0;
				physx::PxU32 vel = 0;
				a->art->getSolverIterationCounts(pos, vel);
				cmd.positionIterations = pos;
				cmd.velocityIterations = vel;
			}
		}

		// ---- bulk joint data ----

		void GetJointsData(Handle sdkObject, const Context& ctx, Physics::FloatBuffer& out, JointData what)
		{
			out = MakeBuffer({});
			ArtScope a(sdkObject, ctx);
			std::vector<float> values;
			if (a && ReadFlat(*a, what, values))
			{
				out = MakeBuffer(values);
			}
		}

		void SetJointsData(Handle sdkObject, const Context& ctx, const Physics::FloatBuffer& in, JointData what)
		{
			ArtScope a(sdkObject, ctx);
			if (!a || in.data == nullptr || in.size == 0)
			{
				return;
			}
			uint32_t total = 0;
			DofLayout(*a, total);
			const size_t dofs = a->inPxScene ? a->art->getDofs() : total;
			if (in.size != dofs)
			{
				static std::atomic<bool> s_warned{ false };
				WarnOnce(s_warned, "Articulation joint buffer size differs from the DOF count; the overlapping part is applied");
			}
			WriteFlat(*a, what, 0, in.data, std::min<size_t>(in.size, dofs));
		}

		void SetJointsAccelerationsBulk(ArticulationObj& a, const float* values, size_t n)
		{
			// Joint accelerations are realized as the joint forces that produce them (inverse dynamics).
			artic::Cache* cache = EnsureCache(a);
			if (cache == nullptr)
			{
				return;
			}
			const size_t dofs = a.art->getDofs();
			artic::ZeroCache(*a.art, *cache);
			float* acc = artic::Data(*cache, JointData::Acceleration);
			for (size_t i = 0; i < n && i < dofs; ++i)
			{
				acc[i] = values[i];
			}
			artic::ComputeForcesForAcceleration(*a.art, *cache);
			artic::ApplyState(*a.art, *cache, JointData::Force);
		}

		void GetJointsPositions(Handle s, const Context& ctx, Command& c)
		{
			GetJointsData(s, ctx, static_cast<ad::GetJointsPositions&>(c).value, JointData::Position);
		}

		void SetJointsPositions(Handle s, const Context& ctx, Command& c)
		{
			SetJointsData(s, ctx, static_cast<ad::SetJointsPositions&>(c).value, JointData::Position);
		}

		void GetJointsVelocities(Handle s, const Context& ctx, Command& c)
		{
			GetJointsData(s, ctx, static_cast<ad::GetJointsVelocities&>(c).value, JointData::Velocity);
		}

		void SetJointsVelocities(Handle s, const Context& ctx, Command& c)
		{
			SetJointsData(s, ctx, static_cast<ad::SetJointsVelocities&>(c).value, JointData::Velocity);
		}

		void GetJointsAccelerations(Handle s, const Context& ctx, Command& c)
		{
			GetJointsData(s, ctx, static_cast<ad::GetJointsAccelerations&>(c).value, JointData::Acceleration);
		}

		void SetJointsAccelerations(Handle sdkObject, const Context& ctx, Command& c)
		{
			const auto& in = static_cast<ad::SetJointsAccelerations&>(c).value;
			ArtScope a(sdkObject, ctx);
			if (a && a->inPxScene && in.data != nullptr)
			{
				SetJointsAccelerationsBulk(*a, in.data, in.size);
			}
		}

		void GetJointsForces(Handle s, const Context& ctx, Command& c)
		{
			GetJointsData(s, ctx, static_cast<ad::GetJointsForces&>(c).value, JointData::Force);
		}

		void SetJointsForces(Handle s, const Context& ctx, Command& c)
		{
			SetJointsData(s, ctx, static_cast<ad::SetJointsForces&>(c).value, JointData::Force);
		}

		// Inverse dynamics outputs; zeros when the articulation is not in a scene.
		enum class Compute
		{
			Gravity,
			Coriolis,
			ForceForAcceleration
		};

		bool ComputeFlat(ArticulationObj& a, Compute what, const std::vector<float>* acceleration, std::vector<float>& out)
		{
			artic::Cache* cache = EnsureCache(a);
			if (cache == nullptr)
			{
				return false;
			}
			const size_t dofs = a.art->getDofs();
			artic::ZeroCache(*a.art, *cache);
			switch (what)
			{
			case Compute::Gravity:
				artic::ComputeGravityForces(*a.art, *cache);
				break;
			case Compute::Coriolis:
				artic::CopyState(*a.art, *cache, JointData::Velocity);
				artic::ComputeCoriolisForces(*a.art, *cache);
				break;
			case Compute::ForceForAcceleration:
			{
				float* acc = artic::Data(*cache, JointData::Acceleration);
				if (acceleration != nullptr)
				{
					for (size_t i = 0; i < acceleration->size() && i < dofs; ++i)
					{
						acc[i] = (*acceleration)[i];
					}
				}
				artic::ComputeForcesForAcceleration(*a.art, *cache);
				break;
			}
			}
			const float* force = artic::Data(*cache, JointData::Force);
			out.assign(force, force + dofs);
			return true;
		}

		void GetJointsInverseDynamics(Handle sdkObject, const Context& ctx, Physics::FloatBuffer& out, Compute what)
		{
			out = MakeBuffer({});
			ArtScope a(sdkObject, ctx);
			std::vector<float> values;
			if (a && a->inPxScene && ComputeFlat(*a, what, nullptr, values))
			{
				out = MakeBuffer(values);
			}
		}

		void GetJointsGravityForces(Handle s, const Context& ctx, Command& c)
		{
			GetJointsInverseDynamics(s, ctx, static_cast<ad::GetJointsGravityForces&>(c).value, Compute::Gravity);
		}

		void GetJointsCoriolisCentrifugalForces(Handle s, const Context& ctx, Command& c)
		{
			GetJointsInverseDynamics(s, ctx, static_cast<ad::GetJointsCoriolisCentrifugalForces&>(c).value, Compute::Coriolis);
		}

		void GetJointsExternalForces(Handle sdkObject, const Context& ctx, Command& c)
		{
			// Forces added with AddForceToLink are applied immediately and are not tracked, so nothing is pending.
			auto& cmd = static_cast<ad::GetJointsExternalForces&>(c);
			cmd.value = MakeBuffer({});
			ArtScope a(sdkObject, ctx);
			if (a && a->inPxScene)
			{
				cmd.value = MakeBuffer(std::vector<float>(a->art->getDofs(), 0.0f));
			}
		}

		// Estimate of the drive effort per DOF: stiffness * position error + damping * velocity error,
		// clamped to the force limit. Acceleration drives report an acceleration.
		bool ComputeDriveForces(ArticulationObj& a, std::vector<float>& out)
		{
			std::vector<float> pos;
			std::vector<float> vel;
			if (!ReadFlat(a, JointData::Position, pos) || !ReadFlat(a, JointData::Velocity, vel))
			{
				return false;
			}
			out.assign(pos.size(), 0.0f);
			uint32_t total = 0;
			for (const auto& entry : DofLayout(a, total))
			{
				const LinkObj& l = *entry.first;
				uint32_t i = 0;
				for (int axis = 0; axis < artic::kAxisCount; ++axis)
				{
					if (EffectiveMotion(l, axis) == Motion::Locked)
					{
						continue;
					}
					const size_t idx = entry.second + i++;
					if (idx >= out.size())
					{
						break;
					}
					const auto& m = l.axes[axis].motor;
					float f = m.stiffness * (m.target - pos[idx]) + m.damping * (m.targetVelocity - vel[idx]);
					if (m.type != Physics::ArticulationMotorType::Acceleration)
					{
						const float limit = SafeForceLimit(m.forceLimit);
						f = std::min(std::max(f, -limit), limit);
					}
					out[idx] = f;
				}
			}
			return true;
		}

		void GetJointsDriveForces(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::GetJointsDriveForces&>(c);
			cmd.value = MakeBuffer({});
			ArtScope a(sdkObject, ctx);
			std::vector<float> values;
			if (a && ComputeDriveForces(*a, values))
			{
				cmd.value = MakeBuffer(values);
			}
		}

		void GetDenseJacobian(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ad::GetDenseJacobianAsBuffer&>(c);
			cmd.rowCount = 0;
			cmd.colCount = 0;
			cmd.value = MakeBuffer({});
			ArtScope a(sdkObject, ctx);
			artic::Cache* cache = a ? EnsureCache(*a) : nullptr;
			if (cache == nullptr)
			{
				return;
			}
			uint32_t rows = 0;
			uint32_t cols = 0;
			const float* jacobian = artic::ComputeDenseJacobian(*a->art, *cache, rows, cols);
			if (jacobian == nullptr || rows == 0 || cols == 0)
			{
				return;
			}
			cmd.rowCount = rows;
			cmd.colCount = cols;
			cmd.value = MakeBuffer(std::vector<float>(jacobian, jacobian + static_cast<size_t>(rows) * cols));
		}

		void ReleaseDataBuffer(Handle, const Context&, Command& c)
		{
			ReleaseBuffer(static_cast<ad::ReleaseDataBuffer&>(c).value.data);
		}

		// ---- link body properties ----

		void GetLinkIndex(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetLinkIndex&>(c);
			cmd.value = -1;
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				const auto& links = l->art->links;
				const auto it = std::find(links.begin(), links.end(), l.get());
				if (it != links.end())
				{
					cmd.value = static_cast<int>(it - links.begin());
				}
			}
		}

		void GetLinkJoint(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			// The root link has no joint; handing Unity one makes it write joint anchors against a missing parent.
			static_cast<ald::GetLinkJoint&>(c).value = (l && l->parent != nullptr) ? l->jointHandle : nullptr;
		}

		void SetLinkFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::SetFlags&>(c);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->flags = cmd.enabled ? (l->flags | cmd.value) : (l->flags & ~cmd.value);
				ApplyLinkGravity(*l);
			}
		}

		void GetLinkFlags(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetFlags&>(c).value = l ? l->flags : BodyFlags::None;
		}

		void SetLinkMass(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = static_cast<ald::SetMass&>(c).value;
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->mass = value;
				if (value > 0.0f)
				{
					l->link->setMass(value);
				}
			}
		}

		void GetLinkMass(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetMass&>(c).value = l ? l->link->getMass() : 0.0f;
		}

		void SetLinkDamping(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::SetDamping&>(c);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->linearDamping = cmd.linear;
				l->angularDamping = cmd.angular;
				l->link->setLinearDamping(std::max(cmd.linear, 0.0f));
				l->link->setAngularDamping(std::max(cmd.angular, 0.0f));
			}
		}

		void GetLinkDamping(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetDamping&>(c);
			LinkScope l(sdkObject, ctx);
			cmd.linear = l ? l->link->getLinearDamping() : 0.0f;
			cmd.angular = l ? l->link->getAngularDamping() : 0.0f;
		}

		void SetLinkPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::SetPose&>(c);
			LinkScope l(sdkObject, ctx);
			if (!l)
			{
				return;
			}
			ArticulationObj& a = *l->art;
			if (!a.inPxScene && l->parent == nullptr)
			{
				// Only the root pose is meaningful (PhysX derives every other link from the joints when it is added),
				// and PhysX takes it from createLink, so the PhysX articulation is built again with the new pose.
				const physx::PxTransform pose = ToPx(cmd.value);
				RebuildUnscened(a, [&] { l->pose = pose; });
			}
			else if (a.inPxScene && l->parent == nullptr)
			{
				artic::SetRootPose(*a.art, ToPx(cmd.value));
			}
			else
			{
				static std::atomic<bool> s_warned{ false };
				WarnOnce(s_warned, "Articulation links below the root cannot be moved directly; set joint positions instead");
			}
		}

		void GetLinkPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetPose&>(c).value = l ? FromPx(l->link->getGlobalPose()) : Physics::Pose();
		}

		// ---- link shapes ----

		void AttachShapeToLink(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::AttachShape&>(c);
			Lock lock(Target{ Kind::Link, PickLink(sdkObject, ctx) }, Target{ Kind::Shape, cmd.shape });
			if (!lock)
			{
				return;
			}
			auto* l = lock.get<LinkObj>(0);
			auto* shape = lock.get<ShapeObj>(1);
			if (!Alive(l))
			{
				return;
			}

			shape->shape->setLocalPose(ToPx(cmd.pose));
			if (shape->ownerLink == l)
			{
				return;
			}
			if (shape->ownerLink != nullptr)
			{
				DetachShapeFromLinkLocked(shape);
			}
			else if (shape->owner != nullptr)
			{
				BodyObj* old = shape->owner;
				old->actor->detachShape(*shape->shape);
				old->shapes.erase(std::remove(old->shapes.begin(), old->shapes.end(), shape), old->shapes.end());
				shape->owner = nullptr;
			}

			ReapplyShapeFlagsForLink(*shape);
			shape->ownerLink = l;
			shape->shape->setSimulationFilterData(EncodeShapeFilter(*shape)); // before attaching, so new pairs see the group
			if (!l->link->attachShape(*shape->shape))
			{
				BACKEND_ERRORF("PxRigidActor::attachShape failed for an articulation link");
				shape->ownerLink = nullptr;
				shape->shape->setSimulationFilterData(EncodeShapeFilter(*shape));
				Rebind(shape, LooseState());
				return;
			}
			l->shapes.push_back(shape);
			Rebind(shape, l->art->world->state);
		}

		void DetachShapeFromLink(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::DetachShape&>(c);
			Lock lock(Target{ Kind::Link, PickLink(sdkObject, ctx) }, Target{ Kind::Shape, cmd.shape });
			if (!lock)
			{
				return;
			}
			auto* shape = lock.get<ShapeObj>(1);
			if (shape->ownerLink == lock.get<LinkObj>(0))
			{
				DetachShapeFromLinkLocked(shape);
			}
		}

		void GetLinkShapeCount(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetShapeCount&>(c).value = l.found() ? l->shapes.size() : 0;
		}

		void GetLinkShapesUserData(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetShapesUserData&>(c);
			cmd.written = 0;
			LinkScope l(sdkObject, ctx);
			if (!l.found() || cmd.buffer == nullptr)
			{
				return;
			}
			for (const ShapeObj* shape : l->shapes)
			{
				if (cmd.written >= cmd.bufferSize)
				{
					break;
				}
				cmd.buffer[cmd.written++] = shape->userData;
			}
		}

		void GetLinkUserData(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetUserData&>(c).value = l.found() ? l->userData : nullptr;
		}

		void SetLinkCollisionDetectionMode(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->ccdMode = static_cast<ald::SetCollisionDetectionMode&>(c).value;
				if (l->ccdMode != Physics::CollisionDetectionMode::Discrete)
				{
					static std::atomic<bool> s_warned{ false };
					WarnOnce(s_warned, "Continuous collision detection is not supported for articulation links; staying discrete");
				}
			}
		}

		// ---- link forces / velocities ----

		void AddForceToLink(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::AddForce&>(c);
			LinkScope l(sdkObject, ctx);
			if (l && l->art->inPxScene)
			{
				l->link->addForce(ToPx(cmd.value), ToPxForceMode(cmd.mode), cmd.awake);
			}
		}

		void AddTorqueToLink(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::AddTorque&>(c);
			LinkScope l(sdkObject, ctx);
			if (l && l->art->inPxScene)
			{
				l->link->addTorque(ToPx(cmd.value), ToPxForceMode(cmd.mode), cmd.awake);
			}
		}

		// Only the root link velocity can be set (the others follow from the joint velocities).
		void SetLinkVelocityImpl(LinkScope& l, const physx::PxVec3* linear, const physx::PxVec3* angular)
		{
			ArticulationObj& a = *l->art;
			if (!a.inPxScene || l->parent != nullptr)
			{
				if (a.inPxScene)
				{
					static std::atomic<bool> s_warned{ false };
					WarnOnce(s_warned, "Articulation link velocities below the root cannot be set; set joint velocities instead");
				}
				return;
			}
			if (artic::Cache* cache = EnsureCache(a))
			{
				artic::SetRootVelocity(*a.art, *cache, linear, angular);
			}
		}

		void SetLinkLinearVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const physx::PxVec3 v = ToPx(static_cast<ald::SetLinearVelocity&>(c).value);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				SetLinkVelocityImpl(l, &v, nullptr);
			}
		}

		void SetLinkAngularVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const physx::PxVec3 v = ToPx(static_cast<ald::SetAngularVelocity&>(c).value);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				SetLinkVelocityImpl(l, nullptr, &v);
			}
		}

		void GetLinkLinearVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetLinearVelocity&>(c).value = l ? FromPx(l->link->getLinearVelocity()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void GetLinkAngularVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetAngularVelocity&>(c).value = l ? FromPx(l->link->getAngularVelocity()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void SetLinkMaxLinearVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = std::max(static_cast<ald::SetMaxLinearVelocity&>(c).value, 0.0f);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->maxLinearVelocity = value;
				l->link->setMaxLinearVelocity(value);
			}
		}

		void GetLinkMaxLinearVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetMaxLinearVelocity&>(c).value = l ? l->link->getMaxLinearVelocity() : 0.0f;
		}

		void SetLinkMaxAngularVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = std::max(static_cast<ald::SetMaxAngularVelocity&>(c).value, 0.0f);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->maxAngularVelocity = value;
				l->link->setMaxAngularVelocity(value);
			}
		}

		void GetLinkMaxAngularVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetMaxAngularVelocity&>(c).value = l ? l->link->getMaxAngularVelocity() : 0.0f;
		}

		void SetLinkMaxDepenetrationVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = std::max(static_cast<ald::SetMaxDepenetrationVelocity&>(c).value, 0.0f);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->maxDepenetrationVelocity = value;
				l->link->setMaxDepenetrationVelocity(value);
			}
		}

		void GetLinkMaxDepenetrationVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetMaxDepenetrationVelocity&>(c).value = l ? l->link->getMaxDepenetrationVelocity() : 0.0f;
		}

		// ---- link mass properties ----

		void SetLinkInertiaTensor(Handle sdkObject, const Context& ctx, Command& c)
		{
			const physx::PxVec3 v = ToPx(static_cast<ald::SetInertiaTensor&>(c).value);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->link->setMassSpaceInertiaTensor(physx::PxVec3(std::max(v.x, 0.0f), std::max(v.y, 0.0f), std::max(v.z, 0.0f)));
			}
		}

		void GetLinkInertiaTensor(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetInertiaTensor&>(c).value = l ? FromPx(l->link->getMassSpaceInertiaTensor()) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void SetLinkInertiaTensorRotation(Handle sdkObject, const Context& ctx, Command& c)
		{
			const Physics::Vec4 q = static_cast<ald::SetInertiaTensorRotation&>(c).value;
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				physx::PxTransform t = l->link->getCMassLocalPose();
				t.q = ToPx(Physics::Pose(0, 0, 0, q.x, q.y, q.z, q.w)).q;
				l->link->setCMassLocalPose(t);
			}
		}

		void GetLinkInertiaTensorRotation(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			auto& out = static_cast<ald::GetInertiaTensorRotation&>(c).value;
			if (l)
			{
				const physx::PxQuat q = l->link->getCMassLocalPose().q;
				out = Physics::Vec4(q.x, q.y, q.z, q.w);
			}
			else
			{
				out = Physics::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
			}
		}

		void SetLinkLocalCenterOfMass(Handle sdkObject, const Context& ctx, Command& c)
		{
			const physx::PxVec3 v = ToPx(static_cast<ald::SetLocalCenterOfMass&>(c).value);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				physx::PxTransform t = l->link->getCMassLocalPose();
				t.p = v;
				l->link->setCMassLocalPose(t);
			}
		}

		void GetLinkLocalCenterOfMass(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ald::GetLocalCenterOfMass&>(c).value = l ? FromPx(l->link->getCMassLocalPose().p) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void GetLinkWorldInertiaTensorMatrix(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetWorldInertiaTensorMatrix&>(c);
			std::fill(std::begin(cmd.value), std::end(cmd.value), 0.0f);
			LinkScope l(sdkObject, ctx);
			if (!l)
			{
				return;
			}
			const physx::PxTransform cm = l->link->getGlobalPose() * l->link->getCMassLocalPose();
			const physx::PxMat33 r(cm.q);
			const physx::PxMat33 w = r * physx::PxMat33::createDiagonal(l->link->getMassSpaceInertiaTensor()) * r.getTranspose();
			for (int row = 0; row < 3; ++row)
			{
				for (int col = 0; col < 3; ++col)
				{
					cmd.value[row * 3 + col] = w[col][row];
				}
			}
		}

		void RecomputeLinkMassProperties(Handle sdkObject, const Context& ctx, Command& c)
		{
			const auto& overrides = static_cast<ald::RecomputeMassProperties&>(c).overrides;
			LinkScope l(sdkObject, ctx);
			if (!l)
			{
				return;
			}
			size_t simShapes = 0;
			for (const ShapeObj* shape : l->shapes)
			{
				if (shape->shape->getFlags() & physx::PxShapeFlag::eSIMULATION_SHAPE)
				{
					++simShapes;
				}
			}
			if (simShapes > 0 && l->mass > 0.0f)
			{
				physx::PxRigidBodyExt::setMassAndUpdateInertia(*l->link, l->mass);
			}

			physx::PxTransform cm = l->link->getCMassLocalPose();
			bool changed = false;
			if (overrides.overrideCenterOfMass)
			{
				cm.p = ToPx(overrides.centerOfMass);
				changed = true;
			}
			if (overrides.overrideInertiaTensor)
			{
				cm.q = ToPx(Physics::Pose(0, 0, 0, overrides.inertiaTensorRotation.x, overrides.inertiaTensorRotation.y,
					overrides.inertiaTensorRotation.z, overrides.inertiaTensorRotation.w)).q;
				changed = true;
			}
			if (changed)
			{
				l->link->setCMassLocalPose(cm);
			}
			if (overrides.overrideInertiaTensor)
			{
				const physx::PxVec3 v = ToPx(overrides.inertiaTensor);
				l->link->setMassSpaceInertiaTensor(physx::PxVec3(std::max(v.x, 0.0f), std::max(v.y, 0.0f), std::max(v.z, 0.0f)));
			}
		}

		void GetLinkLocalPointVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetLocalPointVelocity&>(c);
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				const physx::PxVec3 world = l->link->getGlobalPose().transform(ToPx(cmd.localPoint));
				cmd.value = FromPx(physx::PxRigidBodyExt::getVelocityAtPos(*l->link, world));
			}
			else
			{
				cmd.value = Physics::Vec3(0.0f, 0.0f, 0.0f);
			}
		}

		void GetLinkWorldPointVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetWorldPointVelocity&>(c);
			LinkScope l(sdkObject, ctx);
			cmd.value = l ? FromPx(physx::PxRigidBodyExt::getVelocityAtPos(*l->link, ToPx(cmd.worldPoint))) : Physics::Vec3(0.0f, 0.0f, 0.0f);
		}

		void GetLinkClosestWorldPoint(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ald::GetClosestWorldPoint&>(c);
			cmd.outClosestWorldPosition = cmd.worldPosition;
			cmd.outSqrDistance = 0.0f;
			LinkScope l(sdkObject, ctx);
			if (!l || l->shapes.empty())
			{
				return;
			}
			const physx::PxBounds3 bounds = l->link->getWorldBounds(1.0f);
			if (bounds.isEmpty())
			{
				return;
			}
			const physx::PxVec3 p = ToPx(cmd.worldPosition);
			const physx::PxVec3 closest = bounds.minimum.maximum(p.minimum(bounds.maximum));
			cmd.outClosestWorldPosition = FromPx(closest);
			cmd.outSqrDistance = (closest - p).magnitudeSquared();
		}

		// ---- link joint ----

		void SetJointType(Handle sdkObject, const Context& ctx, Command& c)
		{
			const JointType type = static_cast<ajd::SetType&>(c).value;
			LinkScope l(sdkObject, ctx);
			if (!l || l->jointType == type)
			{
				return;
			}
			if (l->joint != nullptr)
			{
				ApplyChange(*l->art, [&] { l->jointType = type; }, [&] { ApplyJoint(*l); });
			}
			else
			{
				l->jointType = type;
			}
		}

		void GetJointType(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ajd::GetType&>(c).value = l.found() ? l->jointType : JointType::Invalid;
		}

		void SetJointAxisLock(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::SetAxisLock&>(c);
			const int axis = AxisIndex(cmd.axis);
			LinkScope l(sdkObject, ctx);
			if (!l || axis < 0)
			{
				return;
			}
			Motion motion = Motion::Locked;
			switch (cmd.value)
			{
			case Physics::JointDofLock::Limited: motion = Motion::Limited; break;
			case Physics::JointDofLock::Free: motion = Motion::Free; break;
			default: break;
			}
			ChangeAxis(*l, axis, [&](AxisConfig& cfg) { cfg.motion = motion; });
		}

		void SetJointLimit(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::SetLimit&>(c);
			const int axis = AxisIndex(cmd.axis);
			LinkScope l(sdkObject, ctx);
			if (!l || axis < 0)
			{
				return;
			}
			ChangeAxis(*l, axis, [&](AxisConfig& cfg) {
				cfg.lowerLimit = std::min(cmd.min, cmd.max);
				cfg.upperLimit = std::max(cmd.min, cmd.max);
			});
		}

		void SetJointActorLocalPose(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::SetActorLocalPose&>(c);
			LinkScope l(sdkObject, ctx);
			if (!l)
			{
				return;
			}
			auto store = [&] { (cmd.target == Physics::PoseTarget::Parent ? l->parentFrame : l->childFrame) = cmd.value; };
			auto apply = [&] {
				if (l->joint == nullptr)
				{
					return;
				}
				if (cmd.target == Physics::PoseTarget::Parent)
				{
					l->joint->setParentPose(ToPx(cmd.value));
				}
				else
				{
					l->joint->setChildPose(ToPx(cmd.value));
				}
			};
			// Link poses are derived from the joint frames when the articulation enters a scene, so a frame
			// change inside a scene rebuilds it to keep the links consistent.
			if (l->joint != nullptr)
			{
				ApplyChange(*l->art, store, apply);
			}
			else
			{
				store();
			}
		}

		void SetJointAxisMotorConfiguration(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::SetAxisMotorConfiguration&>(c);
			const int axis = AxisIndex(cmd.axis);
			LinkScope l(sdkObject, ctx);
			if (!l || axis < 0)
			{
				return;
			}
			ChangeAxis(*l, axis, [&](AxisConfig& cfg) {
				cfg.motor = cmd.value;
				// The limits that travel with the drive only count when they describe a real range.
				if (cmd.value.lowerLimit < cmd.value.upperLimit)
				{
					cfg.lowerLimit = cmd.value.lowerLimit;
					cfg.upperLimit = cmd.value.upperLimit;
				}
			});
		}

		void SetJointFriction(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = static_cast<ajd::SetFriction&>(c).value;
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->friction = value;
				if (l->joint != nullptr)
				{
					artic::SetFriction(*l->joint, std::max(value, 0.0f));
					WakeIfInScene(*l->art);
				}
			}
		}

		void SetJointMaxVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			const float value = static_cast<ajd::SetMaxVelocity&>(c).value;
			LinkScope l(sdkObject, ctx);
			if (l)
			{
				l->maxJointVelocity = value;
				if (l->joint != nullptr)
				{
					artic::SetMaxVelocity(*l->joint, std::max(value, 0.0f));
					WakeIfInScene(*l->art);
				}
			}
		}

		void GetJointMaxVelocity(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ajd::GetMaxVelocity&>(c).value = l.found() ? l->maxJointVelocity : 0.0f;
		}

		void GetJointDofCount(Handle sdkObject, const Context& ctx, Command& c)
		{
			LinkScope l(sdkObject, ctx);
			static_cast<ajd::GetDofCount&>(c).value = l.found() && l->joint != nullptr ? DofCount(*l) : 0;
		}

		// Reads the DOF values of one link as up to three coordinates.
		void ReadLinkJoint(LinkObj& l, JointData what, Physics::ReducedSpaceCoordinateStorage& out)
		{
			out = Physics::ReducedSpaceCoordinateStorage();
			if (l.joint == nullptr)
			{
				return;
			}
			const uint32_t n = std::min(DofCount(l), 3u);
			out.dofCount = static_cast<int>(n);
			if (n == 0)
			{
				return;
			}
			uint32_t total = 0;
			uint32_t offset = 0;
			for (const auto& entry : DofLayout(*l.art, total))
			{
				if (entry.first == &l)
				{
					offset = entry.second;
				}
			}
			std::vector<float> flat;
			if (!ReadFlat(*l.art, what, flat))
			{
				return;
			}
			float* dst = out.coordinate.Data();
			for (uint32_t i = 0; i < n && offset + i < flat.size(); ++i)
			{
				dst[i] = flat[offset + i];
			}
		}

		void WriteLinkJoint(LinkObj& l, JointData what, const Physics::ReducedSpaceCoordinateStorage& in)
		{
			if (l.joint == nullptr)
			{
				return;
			}
			const uint32_t n = std::min(DofCount(l), 3u);
			if (n == 0)
			{
				return;
			}
			uint32_t total = 0;
			uint32_t offset = 0;
			for (const auto& entry : DofLayout(*l.art, total))
			{
				if (entry.first == &l)
				{
					offset = entry.second;
				}
			}
			const float* src = in.coordinate.Data();
			WriteFlat(*l.art, what, offset, src, n);
		}

		void GetJointPosition(Handle s, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::GetPosition&>(c);
			LinkScope l(s, ctx);
			if (l) ReadLinkJoint(*l, JointData::Position, cmd.value); else cmd.value = Physics::ReducedSpaceCoordinateStorage();
		}

		void SetJointPosition(Handle s, const Context& ctx, Command& c)
		{
			LinkScope l(s, ctx);
			if (l) WriteLinkJoint(*l, JointData::Position, static_cast<ajd::SetPosition&>(c).value);
		}

		void GetJointVelocity(Handle s, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::GetVelocity&>(c);
			LinkScope l(s, ctx);
			if (l) ReadLinkJoint(*l, JointData::Velocity, cmd.value); else cmd.value = Physics::ReducedSpaceCoordinateStorage();
		}

		void SetJointVelocity(Handle s, const Context& ctx, Command& c)
		{
			LinkScope l(s, ctx);
			if (l) WriteLinkJoint(*l, JointData::Velocity, static_cast<ajd::SetVelocity&>(c).value);
		}

		void GetJointAcceleration(Handle s, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::GetAcceleration&>(c);
			LinkScope l(s, ctx);
			if (l) ReadLinkJoint(*l, JointData::Acceleration, cmd.value); else cmd.value = Physics::ReducedSpaceCoordinateStorage();
		}

		// Forces of one link that produce the given accelerations on that link's DOFs (all other DOFs at rest).
		bool ForcesForLinkAcceleration(LinkObj& l, const Physics::ReducedSpaceCoordinateStorage& acceleration, Physics::ReducedSpaceCoordinateStorage& forces,
			size_t& offset)
		{
			forces = Physics::ReducedSpaceCoordinateStorage();
			ArticulationObj& a = *l.art;
			if (!a.inPxScene || l.joint == nullptr)
			{
				return false;
			}
			const uint32_t n = std::min(DofCount(l), 3u);
			forces.dofCount = static_cast<int>(n);
			uint32_t total = 0;
			offset = 0;
			for (const auto& entry : DofLayout(a, total))
			{
				if (entry.first == &l)
				{
					offset = entry.second;
				}
			}
			std::vector<float> acc(a.art->getDofs(), 0.0f);
			const float* src = acceleration.coordinate.Data();
			for (uint32_t i = 0; i < n && offset + i < acc.size(); ++i)
			{
				acc[offset + i] = src[i];
			}
			std::vector<float> out;
			if (!ComputeFlat(a, Compute::ForceForAcceleration, &acc, out))
			{
				return false;
			}
			float* dst = forces.coordinate.Data();
			for (uint32_t i = 0; i < n && offset + i < out.size(); ++i)
			{
				dst[i] = out[offset + i];
			}
			return true;
		}

		void SetJointAcceleration(Handle s, const Context& ctx, Command& c)
		{
			// Realized as the force that produces the acceleration; forces of the other joints are kept.
			const auto& in = static_cast<ajd::SetAcceleration&>(c).value;
			LinkScope l(s, ctx);
			if (!l || !l->art->inPxScene || l->joint == nullptr)
			{
				return;
			}
			std::vector<float> before;
			ReadFlat(*l->art, JointData::Force, before);
			Physics::ReducedSpaceCoordinateStorage forces;
			size_t offset = 0;
			if (ForcesForLinkAcceleration(*l, in, forces, offset))
			{
				std::vector<float> next = before;
				for (int i = 0; i < forces.dofCount && offset + i < next.size(); ++i)
				{
					next[offset + i] = forces.coordinate.Data()[i];
				}
				WriteFlat(*l->art, JointData::Force, 0, next.data(), next.size());
			}
		}

		void GetJointForce(Handle s, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::GetForce&>(c);
			LinkScope l(s, ctx);
			if (l) ReadLinkJoint(*l, JointData::Force, cmd.value); else cmd.value = Physics::ReducedSpaceCoordinateStorage();
		}

		void SetJointForce(Handle s, const Context& ctx, Command& c)
		{
			LinkScope l(s, ctx);
			if (l) WriteLinkJoint(*l, JointData::Force, static_cast<ajd::SetForce&>(c).value);
		}

		void GetJointMotorForce(Handle s, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::GetMotorForce&>(c);
			cmd.value = Physics::ReducedSpaceCoordinateStorage();
			LinkScope l(s, ctx);
			if (!l || l->joint == nullptr)
			{
				return;
			}
			std::vector<float> drive;
			if (!ComputeDriveForces(*l->art, drive))
			{
				return;
			}
			uint32_t total = 0;
			for (const auto& entry : DofLayout(*l->art, total))
			{
				if (entry.first != l.get())
				{
					continue;
				}
				const uint32_t n = std::min(DofCount(*l), 3u);
				cmd.value.dofCount = static_cast<int>(n);
				for (uint32_t i = 0; i < n && entry.second + i < drive.size(); ++i)
				{
					cmd.value.coordinate.Data()[i] = drive[entry.second + i];
				}
			}
		}

		void GetJointForcesForAcceleration(Handle s, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<ajd::GetForcesForAcceleration&>(c);
			cmd.value = Physics::ReducedSpaceCoordinateStorage();
			LinkScope l(s, ctx);
			if (l)
			{
				size_t offset = 0;
				ForcesForLinkAcceleration(*l, cmd.acceleration, cmd.value, offset);
			}
		}
	}

	// ---- shared entry points (declared in PxObjects.h) ----

	void DestroyArticulationLocked(Handle h, ArticulationObj* a)
	{
		if (a->added && a->world != nullptr)
		{
			for (LinkObj* l : a->links)
			{
				if (l->link != nullptr)
				{
					NotifyPxActorRemoved(*a->world, l->link, l, l->userData);
				}
			}
		}
		RemoveFromPxScene(*a, false);
		InvalidateCache(*a);
		for (LinkObj* l : a->links)
		{
			DetachLinkShapes(*l);
		}
		a->art->release();
		a->art = nullptr;

		if (a->world != nullptr)
		{
			auto& list = a->world->articulations;
			list.erase(std::remove(list.begin(), list.end(), a), list.end());
		}
		for (LinkObj* l : a->links)
		{
			FreeLinkWrapper(l);
		}
		for (LinkObj* l : a->graveyard)
		{
			FreeLinkWrapper(l);
		}
		Unregister(h);
		delete a;
	}

	void DetachShapeFromLinkLocked(ShapeObj* shape)
	{
		LinkObj* l = shape->ownerLink;
		if (l == nullptr)
		{
			return;
		}
		NotifyShapeRemoved(*shape);
		if (l->link != nullptr)
		{
			l->link->detachShape(*shape->shape);
		}
		l->shapes.erase(std::remove(l->shapes.begin(), l->shapes.end(), shape), l->shapes.end());
		shape->ownerLink = nullptr;
		shape->shape->setSimulationFilterData(EncodeShapeFilter(*shape));
		Rebind(shape, LooseState());
	}

	bool AddArticulationToSceneLocked(WorldObj* world, Handle h)
	{
		auto* a = static_cast<ArticulationObj*>(Lookup(h, Kind::Articulation));
		if (a == nullptr || a->world != world)
		{
			return false;
		}
		a->added = true;
		SyncScene(*a);
		return true;
	}

	void CollectLinkShapesUserData(const WorldObj& world, uint32_t layerMask, std::vector<void*>& out)
	{
		for (const ArticulationObj* a : world.articulations)
		{
			for (const LinkObj* l : a->links)
			{
				for (const ShapeObj* shape : l->shapes)
				{
					if ((shape->filter.belongsTo & layerMask) != 0)
					{
						out.push_back(shape->userData);
					}
				}
			}
		}
	}

	void UpdateJointUnwrap(WorldObj& world)
	{
		for (ArticulationObj* a : world.articulations)
		{
			UpdateJointUnwrapOf(*a);
		}
	}

	void RefilterLinkShapes(WorldObj& world, uint16_t id0, uint16_t id1)
	{
		for (ArticulationObj* a : world.articulations)
		{
			if (!a->inPxScene)
			{
				continue;
			}
			for (LinkObj* l : a->links)
			{
				for (const ShapeObj* shape : l->shapes)
				{
					const uint16_t id = shape->filter.ignorePairID;
					if (id != Physics::FilterData::invalidIgnorePairID && (id == id0 || id == id1))
					{
						world.scene->resetFiltering(*l->link);
						break;
					}
				}
			}
		}
	}

	Handle ShapeOwnerHandle(const ShapeObj& shape)
	{
		if (shape.owner != nullptr)
		{
			return shape.owner;
		}
		return shape.ownerLink;
	}

	void* LinkUserData(Handle link)
	{
		const auto* l = static_cast<const LinkObj*>(Lookup(link, Kind::Link));
		return l != nullptr ? l->userData : nullptr;
	}

	void RegisterArticulationHandlers()
	{
		using W = pc::World;
		using A = pc::Articulation;

		Bind(W::CreateArticulation, &CreateArticulation);
		Bind(W::DestroyArticulation, &DestroyArticulation);
		Bind(W::GetArticulationCount, &GetArticulationCount);
		Bind(W::GetArticulations, &GetArticulations);

		// articulation
		Bind(A::CreateLink, &CreateLink);
		Bind(A::DestroyLink, &DestroyLink);
		Bind(A::GetLinkCount, &GetLinkCount);
		Bind(A::GetLinksUserData, &GetLinksUserData);
		Bind(A::GetLinkParent, &GetLinkParent);
		Bind(A::GetLinkParentUserData, &GetLinkParentUserData);
		Bind(A::GetLinkChildCount, &GetLinkChildCount);
		Bind(A::GetLinkChildrenUserData, &GetLinkChildrenUserData);
		Bind(A::SetFlags, &SetArticulationFlags);
		Bind(A::GetFlags, &GetArticulationFlags);
		Bind(A::GetUserData, &GetArticulationUserData);
		Bind(A::GetIsSleeping, &GetArticulationIsSleeping);
		Bind(A::Sleep, &SleepArticulation);
		Bind(A::WakeUp, &WakeUpArticulation);
		Bind(A::SetSleepThresholdOverride, &SetSleepThresholdOverride);
		Bind(A::GetSleepThresholdOverride, &GetSleepThresholdOverride);
		Bind(A::SetSolverIterationsOverride, &SetSolverIterationsOverride);
		Bind(A::GetSolverIterationsOverride, &GetSolverIterationsOverride);
		Bind(A::GetDenseJacobianAsFloatBuffer, &GetDenseJacobian);
		Bind(A::SetJointsPositions, &SetJointsPositions);
		Bind(A::GetJointsPositions, &GetJointsPositions);
		Bind(A::SetJointsVelocities, &SetJointsVelocities);
		Bind(A::GetJointsVelocities, &GetJointsVelocities);
		Bind(A::SetJointsAccelerations, &SetJointsAccelerations);
		Bind(A::GetJointsAccelerations, &GetJointsAccelerations);
		Bind(A::SetJointsForces, &SetJointsForces);
		Bind(A::GetJointsForces, &GetJointsForces);
		Bind(A::GetJointsDriveForces, &GetJointsDriveForces);
		Bind(A::GetJointsGravityForces, &GetJointsGravityForces);
		Bind(A::GetJointsCoriolisCentrifugalForces, &GetJointsCoriolisCentrifugalForces);
		Bind(A::GetJointsExternalForces, &GetJointsExternalForces);
		Bind(A::ReleaseDataBuffer, &ReleaseDataBuffer);

		// link body
		Bind(A::GetLinkIndex, &GetLinkIndex);
		Bind(A::GetLinkJoint, &GetLinkJoint);
		Bind(A::SetLinkFlags, &SetLinkFlags);
		Bind(A::GetLinkFlags, &GetLinkFlags);
		Bind(A::SetLinkMass, &SetLinkMass);
		Bind(A::GetLinkMass, &GetLinkMass);
		Bind(A::SetLinkDamping, &SetLinkDamping);
		Bind(A::GetLinkDamping, &GetLinkDamping);
		Bind(A::SetLinkPose, &SetLinkPose);
		Bind(A::GetLinkPose, &GetLinkPose);
		Bind(A::AttachShapeToLink, &AttachShapeToLink);
		Bind(A::DetachShapeFromLink, &DetachShapeFromLink);
		Bind(A::GetLinkShapeCount, &GetLinkShapeCount);
		Bind(A::GetLinkShapesUserData, &GetLinkShapesUserData);
		Bind(A::GetLinkUserData, &GetLinkUserData);
		Bind(A::SetLinkCollisionDetectionMode, &SetLinkCollisionDetectionMode);
		Bind(A::AddForceToLink, &AddForceToLink);
		Bind(A::AddTorqueToLink, &AddTorqueToLink);
		Bind(A::SetLinkLinearVelocity, &SetLinkLinearVelocity);
		Bind(A::GetLinkLinearVelocity, &GetLinkLinearVelocity);
		Bind(A::SetLinkAngularVelocity, &SetLinkAngularVelocity);
		Bind(A::GetLinkAngularVelocity, &GetLinkAngularVelocity);
		Bind(A::SetLinkMaxLinearVelocity, &SetLinkMaxLinearVelocity);
		Bind(A::GetLinkMaxLinearVelocity, &GetLinkMaxLinearVelocity);
		Bind(A::SetLinkMaxAngularVelocity, &SetLinkMaxAngularVelocity);
		Bind(A::GetLinkMaxAngularVelocity, &GetLinkMaxAngularVelocity);
		Bind(A::SetLinkMaxDepenetrationVelocity, &SetLinkMaxDepenetrationVelocity);
		Bind(A::GetLinkMaxDepenetrationVelocity, &GetLinkMaxDepenetrationVelocity);
		Bind(A::SetLinkInertiaTensor, &SetLinkInertiaTensor);
		Bind(A::GetLinkInertiaTensor, &GetLinkInertiaTensor);
		Bind(A::SetLinkInertiaTensorRotation, &SetLinkInertiaTensorRotation);
		Bind(A::GetLinkInertiaTensorRotation, &GetLinkInertiaTensorRotation);
		Bind(A::SetLinkLocalCenterOfMass, &SetLinkLocalCenterOfMass);
		Bind(A::GetLinkLocalCenterOfMass, &GetLinkLocalCenterOfMass);
		Bind(A::GetLinkWorldInertiaTensorMatrix, &GetLinkWorldInertiaTensorMatrix);
		Bind(A::RecomputeLinkMassProperties, &RecomputeLinkMassProperties);
		Bind(A::GetLinkLocalPointVelocity, &GetLinkLocalPointVelocity);
		Bind(A::GetLinkWorldPointVelocity, &GetLinkWorldPointVelocity);
		Bind(A::GetLinkClosestWorldPoint, &GetLinkClosestWorldPoint);

		// link joint
		Bind(A::SetLinkJointType, &SetJointType);
		Bind(A::GetLinkJointType, &GetJointType);
		Bind(A::SetLinkJointAxisLock, &SetJointAxisLock);
		Bind(A::SetLinkJointLimit, &SetJointLimit);
		Bind(A::SetLinkJointActorLocalPose, &SetJointActorLocalPose);
		Bind(A::SetLinkJointAxisMotorConfiguration, &SetJointAxisMotorConfiguration);
		Bind(A::SetLinkJointFriction, &SetJointFriction);
		Bind(A::SetLinkJointPosition, &SetJointPosition);
		Bind(A::GetLinkJointPosition, &GetJointPosition);
		Bind(A::SetLinkJointVelocity, &SetJointVelocity);
		Bind(A::GetLinkJointVelocity, &GetJointVelocity);
		Bind(A::SetLinkJointAcceleration, &SetJointAcceleration);
		Bind(A::GetLinkJointAcceleration, &GetJointAcceleration);
		Bind(A::SetLinkJointForce, &SetJointForce);
		Bind(A::GetLinkJointForce, &GetJointForce);
		Bind(A::GetLinkJointMotorForce, &GetJointMotorForce);
		Bind(A::GetLinkJointForcesForAcceleration, &GetJointForcesForAcceleration);
		Bind(A::SetLinkJointMaxVelocity, &SetJointMaxVelocity);
		Bind(A::GetLinkJointMaxVelocity, &GetJointMaxVelocity);
		Bind(A::GetLinkJointDofCount, &GetJointDofCount);
	}
}
