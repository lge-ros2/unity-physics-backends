#pragma once

#include "Dispatch.h"

namespace backend
{
	// Logs one command with its name, target handles and (for known Set*/Create* commands) its input values.
	// Only call while tracing is enabled; formatting is not cheap.
	void TraceCommand(Physics::SDKObjectHandle sdkObject, const PhysicsCommands::Context& ctx,
		uint32_t commandType, size_t cmdId, PhysicsCommands::Command& c);
}
