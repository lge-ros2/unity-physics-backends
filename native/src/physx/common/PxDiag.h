#pragma once

#include "physx/common/PxObjects.h"

namespace backend::pxw
{
	// Called after every simulation step; does nothing unless env BACKEND_DIAG is set.
	void DiagStep(WorldObj& world);
}
