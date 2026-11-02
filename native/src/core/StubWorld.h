#pragma once

#include <cstddef>

namespace backend
{
	// Binds the stub command handlers (handle bookkeeping, empty simulation results,
	// "no hit" queries) into the dispatch tables. Call after InitializeDispatch().
	void RegisterStubWorldHandlers();

	// Drops every handle issued so far.
	void ResetStubWorld();

	// Number of live handles (diagnostics).
	size_t GetLiveHandleCount();
}
