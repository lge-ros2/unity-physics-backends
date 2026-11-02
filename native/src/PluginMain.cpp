#include "core/BackendIds.h"
#include "core/Dispatch.h"
#include "core/StubWorld.h"

#include "PhysicsGeneralCommandData.h"

#ifdef PHYSX5_LINK_PHYSX
#include "physx/common/PhysXSdk.h"
#endif

#include <atomic>
#include <cstdio>

namespace
{
	std::atomic<bool> g_hasRegistered{ false };

	void InitializeSdkHandler(Physics::SDKObjectHandle, const PhysicsCommands::Context&, PhysicsCommands::Command& c)
	{
#ifdef PHYSX5_LINK_PHYSX
		static_cast<PhysicsCommands::SdkData::InitializeSdk&>(c).result = backend::physx_sdk::Initialize();
#else
		static_cast<PhysicsCommands::SdkData::InitializeSdk&>(c).result = true;
#endif
	}

	void ShutdownSdkHandler(Physics::SDKObjectHandle, const PhysicsCommands::Context&, PhysicsCommands::Command&)
	{
#ifdef PHYSX5_LINK_PHYSX
		backend::physx_sdk::Shutdown();
#endif
	}

	void GetIntegrationInfoHandler(Physics::SDKObjectHandle, const PhysicsCommands::Context&, PhysicsCommands::Command& c)
	{
		auto& info = static_cast<PhysicsCommands::SdkData::GetSdkIntegrationInfo&>(c).value;

		info.id = backend::kIntegrationId;
		for (int i = 0; i < 3; ++i)
		{
			info.version[i] = backend::kIntegrationVersion[i];
			info.sdkVersion[i] = backend::kSdkVersion[i];
		}
		info.features = Physics::FeatureSupport::DynamicsSupport;
		std::snprintf(info.name, sizeof(info.name), "%s", backend::kIntegrationName);
		std::snprintf(info.desc, sizeof(info.desc), "%s", backend::kIntegrationDescription);
		// info.limit keeps its default-initialized joint limit ranges.
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces* unityInterfaces)
{
	if (g_hasRegistered.load())
	{
		return;
	}

	IUnityLog* log = unityInterfaces != nullptr ? unityInterfaces->Get<IUnityLog>() : nullptr;

	// Every slot gets a safe default before any handler is bound.
	backend::InitializeDispatch(log);

	if (unityInterfaces == nullptr)
	{
		return;
	}

	backend::RegisterStubWorldHandlers();
	backend::Bind(PhysicsCommands::SDK::InitializeSdk, &InitializeSdkHandler);
	backend::Bind(PhysicsCommands::SDK::ShutdownSdk, &ShutdownSdkHandler);
	backend::Bind(PhysicsCommands::SDK::GetIntegrationInfo, &GetIntegrationInfoHandler);

	auto* physicsBackend = unityInterfaces->Get<IUnityPhysicsBackendSystem>();
	if (physicsBackend == nullptr)
	{
		if (log != nullptr)
		{
			UNITY_LOG_ERROR(log, "[PhysX5Backend] IUnityPhysicsBackendSystem is not available in this Unity version.");
		}
		return;
	}

	const bool registered = physicsBackend->Register(reinterpret_cast<BackendEntryPoint>(&backend::ProcessCommand));
	g_hasRegistered.store(registered);

	if (log != nullptr)
	{
		if (registered)
		{
			UNITY_LOG(log, "[PhysX5Backend] Registered stub physics backend.");
		}
		else
		{
			UNITY_LOG_ERROR(log, "[PhysX5Backend] IUnityPhysicsBackendSystem::Register returned false.");
		}
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload()
{
	// Unity offers no way to unregister the entry point, so the registration flag stays set.
	backend::ResetStubWorld();
}
