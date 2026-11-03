#include "CommandTrace.h"
#include "BackendIds.h"

#include "PhysicsGeneralCommandData.h"
#include "PhysicsMaterialCommandData.h"
#include "PhysicsWorldCommandData.h"
#include "PhysicsBodyCommandData.h"
#include "PhysicsShapeCommandData.h"
#include "PhysicsArticulationCommandData.h"

#include <cstdio>

namespace backend
{
	using namespace PhysicsCommands;
	using PhysXExt::PhysXExtension;

	namespace
	{
#include "CommandNames.inc"

		template <size_t N>
		const char* NameOf(const char* const (&names)[N], size_t id)
		{
			return id < N ? names[id] : "?";
		}

		const char* CommandName(uint32_t commandType, size_t id)
		{
			switch (commandType)
			{
			case kSDKFunctionMask: return NameOf(kSDKNames, id);
			case kWorldFunctionMask: return NameOf(kWorldNames, id);
			case kBodyFunctionMask: return NameOf(kBodyNames, id);
			case kShapeFunctionMask: return NameOf(kShapeNames, id);
			case kQueryFunctionMask: return NameOf(kQueryNames, id);
			case kJointFunctionMask: return NameOf(kJointNames, id);
			case kArticulationFunctionMask: return NameOf(kArticulationNames, id);
			case kPhysXExtensionMask: return NameOf(kPhysXExtensionNames, id);
			default: return "?";
			}
		}

		const char* CategoryLabel(uint32_t commandType)
		{
			switch (commandType)
			{
			case kSDKFunctionMask: return "SDK";
			case kWorldFunctionMask: return "World";
			case kBodyFunctionMask: return "Body";
			case kShapeFunctionMask: return "Shape";
			case kQueryFunctionMask: return "Query";
			case kJointFunctionMask: return "Joint";
			case kArticulationFunctionMask: return "Articulation";
			case kPhysXExtensionMask: return "PhysXExtension";
			default: return "?";
			}
		}

		const char* CombineModeName(Physics::MaterialCombineMode mode)
		{
			switch (mode)
			{
			case Physics::MaterialCombineMode::Average: return "Average";
			case Physics::MaterialCombineMode::Multiply: return "Multiply";
			case Physics::MaterialCombineMode::Minimum: return "Minimum";
			case Physics::MaterialCombineMode::Maximum: return "Maximum";
			default: return "?";
			}
		}

		const char* VerbosityName(Physics::ErrorVerbosityLevel level)
		{
			switch (level)
			{
			case Physics::ErrorVerbosityLevel::Disabled: return "Disabled";
			case Physics::ErrorVerbosityLevel::Minimal: return "Minimal";
			case Physics::ErrorVerbosityLevel::Default: return "Default";
			case Physics::ErrorVerbosityLevel::Verbose: return "Verbose";
			case Physics::ErrorVerbosityLevel::Debug: return "Debug";
			default: return "?";
			}
		}

		// Only inputs are printed: Get* commands carry uninitialized output fields at call time.
		void FormatArgs(uint32_t commandType, size_t id, Command& c, char* out, size_t size)
		{
			out[0] = '\0';

			if (commandType == kSDKFunctionMask)
			{
				switch (static_cast<SDK>(id))
				{
				case SDK::SetErrorVerbosityLevel:
					std::snprintf(out, size, "level=%s", VerbosityName(static_cast<SdkData::SetErrorVerbosityLevel&>(c).value));
					break;
				case SDK::SetImprovedPatchFrictionEnabledGlobal:
					std::snprintf(out, size, "enabled=%d", static_cast<MaterialData::SetImprovedPatchFrictionEnabledGlobal&>(c).value);
					break;
				case SDK::SetMaterialConfiguration:
				{
					auto& v = static_cast<MaterialData::SetMaterialConfiguration&>(c);
					std::snprintf(out, size, "dynamicFriction=%g staticFriction=%g bounciness=%g",
						v.dynamicFriction, v.staticFriction, v.bounciness);
					break;
				}
				case SDK::SetMaterialCombineModes:
				{
					auto& v = static_cast<MaterialData::SetMaterialCombineModes&>(c);
					std::snprintf(out, size, "friction=%s bounciness=%s", CombineModeName(v.friction), CombineModeName(v.bounciness));
					break;
				}
				default: break;
				}
			}
			else if (commandType == kWorldFunctionMask)
			{
				switch (static_cast<World>(id))
				{
				case World::CreateWorld:
				{
					auto& v = static_cast<WorldData::CreateWorld&>(c);
					std::snprintf(out, size,
						"solver=%d broadphase=%d friction=%d contactPairs=%d gravity=(%g,%g,%g) bounceThreshold=%g "
						"fastMotionThreshold=%g jobWorkers=%u bounds=(%g..%g) subdivisions=%u incrementalStaticBP=%d "
						"adaptiveForce=%d enhancedDeterminism=%d",
						static_cast<int>(v.solverType), static_cast<int>(v.broadphaseType), static_cast<int>(v.frictionType),
						static_cast<int>(v.contactPairsMode), v.gravity.x, v.gravity.y, v.gravity.z, v.bounceThreshold,
						v.fastMotionThreshold, v.jobWorkerCount, v.worldMin.x, v.worldMax.x, v.worldSubdivisions,
						v.enableIncrementalStaticBroadphase, v.enableAdaptiveForce, v.enableEnhancedDeterminism);
					break;
				}
				case World::SetGravity:
				{
					auto& v = static_cast<WorldData::SetGravity&>(c).value;
					std::snprintf(out, size, "value=(%g,%g,%g)", v.x, v.y, v.z);
					break;
				}
				case World::SetBounceThreshold:
					std::snprintf(out, size, "value=%g", static_cast<WorldData::SetBounceThreshold&>(c).value);
					break;
				case World::SetScratchBufferChunkCount:
					std::snprintf(out, size, "value=%u", static_cast<WorldData::SetScratchBufferChunkCount&>(c).value);
					break;
				case World::Simulate:
					std::snprintf(out, size, "deltaTime=%g", static_cast<WorldData::Simulate&>(c).deltaTime);
					break;
				case World::CreateBody:
				{
					auto& v = static_cast<WorldData::CreateBody&>(c);
					std::snprintf(out, size, "mass=%g linearDamping=%g angularDamping=%g isStatic=%d pos=(%g,%g,%g)",
						v.mass, v.linearDamping, v.angularDamping, v.isStatic, v.pose.pos.x, v.pose.pos.y, v.pose.pos.z);
					break;
				}
				default: break;
				}
			}
			else if (commandType == kBodyFunctionMask)
			{
				switch (static_cast<Body>(id))
				{
				case Body::SetMass:
					std::snprintf(out, size, "value=%g", static_cast<BodyData::SetMass&>(c).value);
					break;
				case Body::SetPose:
				{
					auto& v = static_cast<BodyData::SetPose&>(c);
					std::snprintf(out, size, "pos=(%g,%g,%g) awake=%d", v.value.pos.x, v.value.pos.y, v.value.pos.z, v.awake);
					break;
				}
				case Body::AttachShape:
				{
					auto& v = static_cast<BodyData::AttachShape&>(c);
					std::snprintf(out, size, "pos=(%g,%g,%g) forceLocalPose=%d", v.pose.pos.x, v.pose.pos.y, v.pose.pos.z,
						v.forceLocalPose);
					break;
				}
				case Body::SetDamping:
				{
					auto& v = static_cast<BodyData::SetDamping&>(c);
					std::snprintf(out, size, "linear=%g angular=%g", v.linear, v.angular);
					break;
				}
				case Body::SetMaxLinearVelocity:
					std::snprintf(out, size, "value=%g", static_cast<BodyData::SetMaxLinearVelocity&>(c).value);
					break;
				case Body::SetMaxAngularVelocity:
					std::snprintf(out, size, "value=%g", static_cast<BodyData::SetMaxAngularVelocity&>(c).value);
					break;
				case Body::SetMaxDepenetrationVelocity:
					std::snprintf(out, size, "value=%g", static_cast<BodyData::SetMaxDepenetrationVelocity&>(c).value);
					break;
				case Body::SetSleepThresholdOverride:
					std::snprintf(out, size, "value=%g", static_cast<BodyData::SetSleepThresholdOverride&>(c).value);
					break;
				case Body::SetSolverIterationsOverride:
				{
					auto& v = static_cast<BodyData::SetSolverIterationsOverride&>(c);
					std::snprintf(out, size, "position=%u velocity=%u", v.positionIterations, v.velocityIterations);
					break;
				}
				default: break;
				}
			}
			else if (commandType == kArticulationFunctionMask)
			{
				if (static_cast<Articulation>(id) == Articulation::SetSolverIterationsOverride)
				{
					auto& v = static_cast<ArticulationData::SetSolverIterationsOverride&>(c);
					std::snprintf(out, size, "position=%u velocity=%u", v.positionIterations, v.velocityIterations);
				}
			}
			else if (commandType == kShapeFunctionMask)
			{
				if (static_cast<Shape>(id) == Shape::SetContactOffset)
				{
					std::snprintf(out, size, "value=%g", static_cast<ShapeData::SetContactOffset&>(c).value);
				}
				else if (static_cast<Shape>(id) == Shape::SetPose)
				{
					auto& v = static_cast<ShapeData::SetPose&>(c).value;
					std::snprintf(out, size, "pos=(%g,%g,%g)", v.pos.x, v.pos.y, v.pos.z);
				}
			}
		}
	}

	void TraceCommand(Physics::SDKObjectHandle sdkObject, const Context& ctx,
		uint32_t commandType, size_t cmdId, Command& c)
	{
		char args[384];
		FormatArgs(commandType, cmdId, c, args, sizeof(args));

		LogFormatted(kUnityLogTypeLog, __FILE__, __LINE__,
			"%s%s::%s(%zu) sdk=%p world=%p body=%p %s",
			kLogPrefix, CategoryLabel(commandType), CommandName(commandType, cmdId), cmdId,
			static_cast<void*>(sdkObject), static_cast<void*>(ctx.world), static_cast<void*>(ctx.body), args);
	}
}
