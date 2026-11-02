#pragma once

#include <cstdint>

// Backend identity. Keep these as the only place where id/name are defined so that
// other backends can reuse the core code with different values.
namespace backend
{
	constexpr uint32_t kIntegrationId = 0x4C504835; // "LPH5"
	constexpr const char* kIntegrationName = "PhysX 5"; // must fit in char[16]
	constexpr const char* kIntegrationDescription = "Stub PhysX 5 physics backend (no simulation)";
	constexpr uint16_t kIntegrationVersion[3] = { 0, 1, 0 };
	constexpr uint16_t kSdkVersion[3] = { 0, 0, 0 };
	constexpr const char* kLogPrefix = "[PhysX5Backend] ";
	constexpr const char* kTraceEnvVar = "PHYSX5_BACKEND_TRACE";
}
