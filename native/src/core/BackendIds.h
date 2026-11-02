#pragma once

#include <cstdint>

// Backend identity. Keep these as the only place where id/name are defined so that
// other backends can reuse the core code with different values.
// BACKEND_PHYSX_SDK is set by CMake: 5 = PhysX 5.x backend, 4 = PhysX 4.1 reference backend (verification only).
#ifndef BACKEND_PHYSX_SDK
#define BACKEND_PHYSX_SDK 5
#endif

namespace backend
{
#if BACKEND_PHYSX_SDK == 4
	constexpr uint32_t kIntegrationId = 0x4C505834; // "LPX4"
	constexpr const char* kIntegrationName = "PhysX 4 Ref"; // must fit in char[16]
	constexpr const char* kIntegrationDescription = "PhysX 4.1 reference backend for verification";
	constexpr uint16_t kSdkVersion[3] = { 4, 1, 2 };
	constexpr const char* kLogPrefix = "[PhysX4RefBackend] ";
	constexpr const char* kTraceEnvVar = "PHYSX4REF_BACKEND_TRACE";
#else
	constexpr uint32_t kIntegrationId = 0x4C504835; // "LPH5"
	constexpr const char* kIntegrationName = "PhysX 5"; // must fit in char[16]
	constexpr const char* kIntegrationDescription = "PhysX 5 physics backend";
	constexpr uint16_t kSdkVersion[3] = { 5, 6, 1 };
	constexpr const char* kLogPrefix = "[PhysX5Backend] ";
	constexpr const char* kTraceEnvVar = "PHYSX5_BACKEND_TRACE";
#endif
	constexpr uint16_t kIntegrationVersion[3] = { 0, 1, 0 };
}
