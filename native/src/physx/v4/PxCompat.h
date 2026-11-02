#pragma once

// PhysX 4.1 specific adaptations. Shared glue in physx/common includes this header,
// so API differences between 4.1 and 5.x are confined to physx/v4 and physx/v5.
namespace backend::compat
{
	constexpr int kPhysXMajor = 4;
}
