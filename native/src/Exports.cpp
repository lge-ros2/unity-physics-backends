#include "core/BackendIds.h"

#include "IUnityInterface.h"

#include <cstdint>

extern "C" UNITY_INTERFACE_EXPORT uint32_t PhysicsBackend_GetIntegrationId()
{
	return backend::kIntegrationId;
}
