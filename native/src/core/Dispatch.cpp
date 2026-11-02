#include "Dispatch.h"
#include "BackendIds.h"
#include "CommandTrace.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace backend
{
	using namespace PhysicsCommands;
	using PhysXExt::PhysXExtension;

	namespace
	{
		std::atomic<IUnityLog*> g_log{ nullptr };
		std::atomic<bool> g_trace{ false };

		// One table per category: binding + "already warned" flag.
		template <typename Enum>
		struct Table
		{
			static constexpr size_t kCount = static_cast<size_t>(Enum::Count);
			Handler handlers[kCount];
			std::atomic<uint8_t> warned[kCount];
			const char* name;

			void Reset(const char* categoryName, Handler fn)
			{
				name = categoryName;
				for (size_t i = 0; i < kCount; ++i)
				{
					handlers[i] = fn;
					warned[i].store(0, std::memory_order_relaxed);
				}
			}

			void Set(Enum id, Handler fn)
			{
				const size_t index = static_cast<size_t>(id);
				if (index < kCount)
				{
					handlers[index] = fn;
				}
			}

			void Dispatch(size_t index, Physics::SDKObjectHandle sdkObject, const Context& ctx, Command& c)
			{
				if (index >= kCount)
				{
					return;
				}

				const Handler fn = handlers[index];
				if (fn == &DefaultHandler)
				{
					if (warned[index].exchange(1, std::memory_order_relaxed) == 0)
					{
						LogFormatted(kUnityLogTypeWarning, __FILE__, __LINE__,
							"%sunimplemented command (default handler): category=%s id=%zu", kLogPrefix, name, index);
					}
					return;
				}

				fn(sdkObject, ctx, c);
			}

			static void DefaultHandler(Physics::SDKObjectHandle, const Context&, Command&) {}
		};

		Table<SDK> g_sdk;
		Table<World> g_world;
		Table<Body> g_body;
		Table<Shape> g_shape;
		Table<Query> g_query;
		Table<Joint> g_joint;
		Table<Articulation> g_articulation;
		Table<PhysXExtension> g_physxExt;

		// All tables share the same default handler signature; the dispatcher compares
		// against it to emit a one-time warning that knows the (category, id).
		template <typename T>
		Handler DefaultFor()
		{
			return &T::DefaultHandler;
		}
	}

	void SetLogInterface(IUnityLog* log)
	{
		g_log.store(log, std::memory_order_release);
	}

	void LogFormatted(UnityLogType type, const char* file, int line, const char* fmt, ...)
	{
		IUnityLog* log = g_log.load(std::memory_order_acquire);
		if (log == nullptr)
		{
			return;
		}

		char buffer[512];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buffer, sizeof(buffer), fmt, args);
		va_end(args);

		log->Log(type, buffer, file, line);
	}

	void InitializeDispatch(IUnityLog* log)
	{
		SetLogInterface(log);

		g_sdk.Reset("SDK", DefaultFor<Table<SDK>>());
		g_world.Reset("World", DefaultFor<Table<World>>());
		g_body.Reset("Body", DefaultFor<Table<Body>>());
		g_shape.Reset("Shape", DefaultFor<Table<Shape>>());
		g_query.Reset("Query", DefaultFor<Table<Query>>());
		g_joint.Reset("Joint", DefaultFor<Table<Joint>>());
		g_articulation.Reset("Articulation", DefaultFor<Table<Articulation>>());
		g_physxExt.Reset("PhysXExtension", DefaultFor<Table<PhysXExtension>>());

		// Read once at load; never touched on the hot path.
		const char* trace = std::getenv(kTraceEnvVar);
		g_trace.store(trace != nullptr && std::strcmp(trace, "1") == 0, std::memory_order_relaxed);
	}

	void Bind(SDK id, Handler fn) { g_sdk.Set(id, fn); }
	void Bind(World id, Handler fn) { g_world.Set(id, fn); }
	void Bind(Body id, Handler fn) { g_body.Set(id, fn); }
	void Bind(Shape id, Handler fn) { g_shape.Set(id, fn); }
	void Bind(Query id, Handler fn) { g_query.Set(id, fn); }
	void Bind(Joint id, Handler fn) { g_joint.Set(id, fn); }
	void Bind(Articulation id, Handler fn) { g_articulation.Set(id, fn); }
	void Bind(PhysXExtension id, Handler fn) { g_physxExt.Set(id, fn); }

	void NoopHandler(Physics::SDKObjectHandle, const Context&, Command&) {}

	void ProcessCommand(Physics::SDKObjectHandle sdkObject, const Context& ctx, const uint32_t cmdFunc, Command& c)
	{
		const uint32_t commandType = cmdFunc & kFullMask;
		const size_t cmdId = cmdFunc & kReverseFullMask;

		if (g_trace.load(std::memory_order_relaxed))
		{
			TraceCommand(sdkObject, ctx, commandType, cmdId, c);
		}

		switch (commandType)
		{
		case kSDKFunctionMask: g_sdk.Dispatch(cmdId, sdkObject, ctx, c); break;
		case kWorldFunctionMask: g_world.Dispatch(cmdId, sdkObject, ctx, c); break;
		case kBodyFunctionMask: g_body.Dispatch(cmdId, sdkObject, ctx, c); break;
		case kShapeFunctionMask: g_shape.Dispatch(cmdId, sdkObject, ctx, c); break;
		case kQueryFunctionMask: g_query.Dispatch(cmdId, sdkObject, ctx, c); break;
		case kJointFunctionMask: g_joint.Dispatch(cmdId, sdkObject, ctx, c); break;
		case kArticulationFunctionMask: g_articulation.Dispatch(cmdId, sdkObject, ctx, c); break;
		case kPhysXExtensionMask: g_physxExt.Dispatch(cmdId, sdkObject, ctx, c); break;
		default: break;
		}
	}
}
