#pragma once

#include <cstdint>
#include <cstddef>

#include "IUnityInterface.h"
#include "IUnityLog.h"
#include "IUnityPhysicsBackendSystem.h"
#include "PhysicsCommands.h"
#include "PhysXPhysicsExtensionCommands.h"

namespace backend
{
	using Handler = PhysicsCommands::FunctionBinding;

	// Logging through IUnityLog (silently ignored while no interface is set).
	void SetLogInterface(IUnityLog* log);
#if defined(__GNUC__)
	void LogFormatted(UnityLogType type, const char* file, int line, const char* fmt, ...)
		__attribute__((format(printf, 4, 5)));
#else
	void LogFormatted(UnityLogType type, const char* file, int line, const char* fmt, ...);
#endif

#define BACKEND_LOGF(...) ::backend::LogFormatted(kUnityLogTypeLog, __FILE__, __LINE__, __VA_ARGS__)
#define BACKEND_WARNF(...) ::backend::LogFormatted(kUnityLogTypeWarning, __FILE__, __LINE__, __VA_ARGS__)
#define BACKEND_ERRORF(...) ::backend::LogFormatted(kUnityLogTypeError, __FILE__, __LINE__, __VA_ARGS__)

	// Resets every binding table to the default handler and reads the trace env var once.
	void InitializeDispatch(IUnityLog* log);

	// Binds a handler to a command slot. Must be called before the entry point is registered.
	void Bind(PhysicsCommands::SDK id, Handler fn);
	void Bind(PhysicsCommands::World id, Handler fn);
	void Bind(PhysicsCommands::Body id, Handler fn);
	void Bind(PhysicsCommands::Shape id, Handler fn);
	void Bind(PhysicsCommands::Query id, Handler fn);
	void Bind(PhysicsCommands::Joint id, Handler fn);
	void Bind(PhysicsCommands::Articulation id, Handler fn);
	void Bind(PhysicsCommands::PhysXExt::PhysXExtension id, Handler fn);

	// Intentionally silent handler for commands that need no work in the stub.
	void NoopHandler(Physics::SDKObjectHandle sdkObject, const PhysicsCommands::Context& ctx, PhysicsCommands::Command& c);

	// Entry point registered with IUnityPhysicsBackendSystem::Register.
	void ProcessCommand(Physics::SDKObjectHandle sdkObject, const PhysicsCommands::Context& ctx, const uint32_t cmdFunc, PhysicsCommands::Command& c);
}
