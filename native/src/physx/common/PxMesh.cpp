#include "core/Dispatch.h"
#include "physx/common/PhysXSdk.h"
#include "physx/common/PxCookingBackend.h"
#include "physx/common/PxMesh.h"
#include "physx/common/PxObjects.h"

#include "PhysicsShapeCommandData.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace sd = PhysicsCommands::ShapeData;

	namespace
	{
		// Guards every MeshObj / HeightFieldObj field. Never held while a world lock is taken.
		std::mutex g_meshMutex;

		// Unity heightfield sample grids are z-major (heights[z * resolution + x]); PhysX rows run along x.
		// Flip this if a Unity build turns out to deliver the x-major layout.
		constexpr bool kHeightsZMajor = true;

		constexpr size_t kDefaultVertexStride = sizeof(float) * 3;
		constexpr uint32_t kStreamMagic = 0x534D5043u; // "CPMS"
		constexpr uint16_t kStreamVersion = 1;
		constexpr uint32_t kKindConvex = 1;
		constexpr uint32_t kKindTriangle = 2;

		// Cooked mesh stream: this header followed by the PhysX binary payload of the linked SDK.
		struct StreamHeader
		{
			uint32_t magic;
			uint16_t version;
			uint16_t physxMajor;
			uint32_t kind;
			uint32_t payloadSize;
			uint32_t checksum;
			uint32_t reserved;
		};
		static_assert(sizeof(StreamHeader) == 24, "StreamHeader layout");

		uint32_t Checksum(const uint8_t* data, size_t size)
		{
			uint32_t hash = 2166136261u;
			for (size_t i = 0; i < size; ++i)
			{
				hash = (hash ^ data[i]) * 16777619u;
			}
			return hash;
		}

		class VectorOutputStream : public physx::PxOutputStream
		{
		public:
			physx::PxU32 write(const void* src, physx::PxU32 count) override
			{
				const auto* bytes = static_cast<const uint8_t*>(src);
				data.insert(data.end(), bytes, bytes + count);
				return count;
			}

			std::vector<uint8_t> data;
		};

		class MemoryInputStream : public physx::PxInputStream
		{
		public:
			MemoryInputStream(const uint8_t* data, size_t size)
				: m_data(data), m_left(size)
			{
			}

			physx::PxU32 read(void* dest, physx::PxU32 count) override
			{
				const size_t n = std::min<size_t>(count, m_left);
				std::memcpy(dest, m_data, n);
				m_data += n;
				m_left -= n;
				return static_cast<physx::PxU32>(n);
			}

		private:
			const uint8_t* m_data;
			size_t m_left;
		};

		// ---- object table (g_meshMutex held) ----

		void FreeMesh(MeshObj* mesh)
		{
			Unregister(mesh);
			if (mesh->convex != nullptr)
			{
				mesh->convex->release();
			}
			if (mesh->triangle != nullptr)
			{
				mesh->triangle->release();
			}
			delete mesh;
		}

		void FreeHeightField(HeightFieldObj* field)
		{
			Unregister(field);
			if (field->field != nullptr)
			{
				field->field->release();
			}
			delete field;
		}

		Handle NewMesh(physx::PxConvexMesh* convex, physx::PxTriangleMesh* triangle)
		{
			auto* mesh = new MeshObj();
			mesh->convex = convex;
			mesh->triangle = triangle;
			std::lock_guard<std::mutex> guard(g_meshMutex);
			Register(mesh, Kind::Mesh, mesh, LooseState());
			return mesh;
		}

		Handle NewHeightField(physx::PxHeightField* field, uint32_t resolution)
		{
			auto* obj = new HeightFieldObj();
			obj->field = field;
			obj->resolution = resolution;
			std::lock_guard<std::mutex> guard(g_meshMutex);
			Register(obj, Kind::HeightField, obj, LooseState());
			return obj;
		}

		// Pins the mesh behind a shape (or a mesh handle) for the duration of a callback.
		MeshObj* PinMeshOf(Handle sdkObject, const Context& ctx)
		{
			if (Pick(Kind::Shape, sdkObject, ctx.body) != nullptr)
			{
				Lock lock(Target{ Kind::Shape, sdkObject, ctx.body });
				if (!lock)
				{
					return nullptr;
				}
				MeshObj* mesh = lock.get<ShapeObj>()->mesh;
				if (mesh != nullptr)
				{
					std::lock_guard<std::mutex> guard(g_meshMutex);
					++mesh->refs;
				}
				return mesh;
			}
			return AcquireMesh(sdkObject, Physics::GeometryType::Invalid);
		}

		// ---- cooking ----

		struct CookOutcome
		{
			Physics::CookingError status = Physics::CookingError::Success;
			const char* message = nullptr;
			std::vector<uint8_t> payload;
		};

		const char* Validate(const Physics::CollisionMeshCookingDescriptor& d, size_t& vertexStride)
		{
			vertexStride = d.vertexElementSize != 0 ? d.vertexElementSize : kDefaultVertexStride;
			if (d.vertexStream == nullptr || d.vertexElementCount == 0)
			{
				return "mesh has no vertices";
			}
			if (vertexStride < kDefaultVertexStride)
			{
				return "vertex element size is smaller than a float3";
			}
			for (uint32_t i = 0; i < d.vertexElementCount; ++i)
			{
				float v[3];
				std::memcpy(v, static_cast<const uint8_t*>(d.vertexStream) + i * vertexStride, sizeof(v));
				if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
				{
					return "mesh contains non-finite vertices";
				}
			}
			if (d.convex)
			{
				return d.vertexElementCount >= 4 ? nullptr : "convex mesh needs at least 4 vertices";
			}
			if (d.indexStream == nullptr || d.indexElementCount < 3 || d.indexElementCount % 3 != 0)
			{
				return "triangle mesh needs a non-empty index stream with a multiple of 3 indices";
			}
			if (d.indexElementSize != 2 && d.indexElementSize != 4)
			{
				return "index element size must be 2 or 4";
			}
			for (uint32_t i = 0; i < d.indexElementCount; ++i)
			{
				uint32_t index = 0;
				if (d.indexElementSize == 2)
				{
					uint16_t v;
					std::memcpy(&v, static_cast<const uint8_t*>(d.indexStream) + i * 2, sizeof(v));
					index = v;
				}
				else
				{
					std::memcpy(&index, static_cast<const uint8_t*>(d.indexStream) + i * 4, sizeof(index));
				}
				if (index >= d.vertexElementCount)
				{
					return "triangle index is out of range";
				}
			}
			return nullptr;
		}

		// Cooks into out.payload (PhysX binary format). On failure out.status = Failed and out.message says why.
		bool Cook(const Physics::CollisionMeshCookingDescriptor& d, CookOutcome& out)
		{
			using Opt = Physics::CollisionMeshProcessingOptions;
			size_t vertexStride = 0;
			if (const char* error = Validate(d, vertexStride))
			{
				out.status = Physics::CookingError::Failed;
				out.message = error;
				return false;
			}

			cooking::Options options;
			options.optimizeForRuntime = (d.options & Opt::OptimizeForRuntimePerformance) != Opt::None;
			options.weld = (d.options & Opt::WeldVertices) != Opt::None;
			options.cleanup = (d.options & Opt::CleanupInputMesh) != Opt::None;

			VectorOutputStream stream;
			bool ok = false;
			if (d.convex)
			{
				physx::PxConvexMeshDesc desc;
				desc.points.count = d.vertexElementCount;
				desc.points.stride = static_cast<physx::PxU32>(vertexStride);
				desc.points.data = d.vertexStream;
				desc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;
				desc.vertexLimit = 255;
				bool limitReached = false;
				ok = cooking::CookConvexMesh(desc, options, stream, &limitReached);
				if (ok && limitReached)
				{
					out.status = Physics::CookingError::Warning;
					out.message = "convex hull was simplified to the polygon limit";
				}
			}
			else
			{
				physx::PxTriangleMeshDesc desc;
				desc.points.count = d.vertexElementCount;
				desc.points.stride = static_cast<physx::PxU32>(vertexStride);
				desc.points.data = d.vertexStream;
				desc.triangles.count = d.indexElementCount / 3;
				desc.triangles.stride = static_cast<physx::PxU32>(d.indexElementSize) * 3;
				desc.triangles.data = d.indexStream;
				if (d.indexElementSize == 2)
				{
					desc.flags |= physx::PxMeshFlag::e16_BIT_INDICES;
				}
				if (d.flippedNormals)
				{
					desc.flags |= physx::PxMeshFlag::eFLIPNORMALS;
				}
				ok = cooking::CookTriangleMesh(desc, options, stream);
			}

			if (!ok || stream.data.empty())
			{
				out.status = Physics::CookingError::Failed;
				out.message = "PhysX failed to cook the mesh";
				return false;
			}
			out.payload = std::move(stream.data);
			return true;
		}

		// Creates the mesh object from a PhysX payload. Null on failure.
		Handle CreateMeshFromPayload(bool convex, const uint8_t* payload, size_t size)
		{
			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (physics == nullptr || payload == nullptr || size == 0 || size > 0xFFFFFFFFu)
			{
				return nullptr;
			}
			MemoryInputStream input(payload, size);
			if (convex)
			{
				physx::PxConvexMesh* mesh = physics->createConvexMesh(input);
				return mesh != nullptr ? NewMesh(mesh, nullptr) : nullptr;
			}
			physx::PxTriangleMesh* mesh = physics->createTriangleMesh(input);
			if (mesh != nullptr && mesh->getNbTriangles() == 0)
			{
				mesh->release();
				return nullptr;
			}
			return mesh != nullptr ? NewMesh(nullptr, mesh) : nullptr;
		}

		// The contract allows an asynchronous completion; this backend cooks on the calling thread and
		// reports the result through the callback exactly once before returning.
		void CookCollisionMesh(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CookCollisionMesh&>(c);
			CookOutcome outcome;
			Handle mesh = nullptr;
			if (Cook(cmd.descriptor, outcome))
			{
				mesh = CreateMeshFromPayload(cmd.descriptor.convex, outcome.payload.data(), outcome.payload.size());
				if (mesh == nullptr)
				{
					outcome.status = Physics::CookingError::Failed;
					outcome.message = "PhysX failed to create the cooked mesh";
				}
			}
			if (outcome.status == Physics::CookingError::Failed)
			{
				BACKEND_WARNF("CookCollisionMesh failed: %s", outcome.message != nullptr ? outcome.message : "unknown");
			}
			if (cmd.onMeshReadyCallback != nullptr)
			{
				cmd.onMeshReadyCallback(outcome.status, outcome.message, cmd.dataContext, mesh);
			}
			else if (mesh != nullptr)
			{
				// Nobody can receive the handle; do not leak it.
				std::lock_guard<std::mutex> guard(g_meshMutex);
				FreeMesh(static_cast<MeshObj*>(mesh));
			}
		}

		void CookCollisionMeshStream(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CookCollisionMeshStream&>(c);
			CookOutcome outcome;
			std::vector<uint8_t> stream;
			if (Cook(cmd.descriptor, outcome))
			{
				StreamHeader header = {};
				header.magic = kStreamMagic;
				header.version = kStreamVersion;
				header.physxMajor = static_cast<uint16_t>(compat::kPhysXMajor);
				header.kind = cmd.descriptor.convex ? kKindConvex : kKindTriangle;
				header.payloadSize = static_cast<uint32_t>(outcome.payload.size());
				header.checksum = Checksum(outcome.payload.data(), outcome.payload.size());
				stream.resize(sizeof(header) + outcome.payload.size());
				std::memcpy(stream.data(), &header, sizeof(header));
				std::memcpy(stream.data() + sizeof(header), outcome.payload.data(), outcome.payload.size());
			}
			if (outcome.status == Physics::CookingError::Failed)
			{
				BACKEND_WARNF("CookCollisionMeshStream failed: %s", outcome.message != nullptr ? outcome.message : "unknown");
			}
			if (cmd.onStreamReadyCallback != nullptr)
			{
				// The buffer is only valid during the callback.
				cmd.onStreamReadyCallback(outcome.status, outcome.message, cmd.dataContext, stream.empty() ? nullptr : stream.data(), stream.size());
			}
		}

		void CreateCollisionMeshFromStream(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CreateCollisionMeshFromStream&>(c);
			cmd.outCollisionMesh = nullptr;

			if (cmd.stream == nullptr || cmd.streamSize < sizeof(StreamHeader))
			{
				BACKEND_WARNF("CreateCollisionMeshFromStream: stream is missing or too small; rejected");
				return;
			}
			StreamHeader header;
			std::memcpy(&header, cmd.stream, sizeof(header));
			if (header.magic != kStreamMagic)
			{
				BACKEND_WARNF("CreateCollisionMeshFromStream: not a PhysX backend mesh stream (bad magic); rejected");
				return;
			}
			if (header.version != kStreamVersion || header.physxMajor != compat::kPhysXMajor)
			{
				BACKEND_WARNF("CreateCollisionMeshFromStream: stream was cooked by an incompatible version (format %u, PhysX %u); rejected",
					static_cast<unsigned>(header.version), static_cast<unsigned>(header.physxMajor));
				return;
			}
			const uint32_t expectedKind = cmd.convex ? kKindConvex : kKindTriangle;
			if (header.kind != expectedKind)
			{
				BACKEND_WARNF("CreateCollisionMeshFromStream: stream mesh kind %u does not match the requested kind; rejected", static_cast<unsigned>(header.kind));
				return;
			}
			if (header.payloadSize != cmd.streamSize - sizeof(StreamHeader))
			{
				BACKEND_WARNF("CreateCollisionMeshFromStream: stream size mismatch; rejected");
				return;
			}
			const uint8_t* payload = cmd.stream + sizeof(StreamHeader);
			if (Checksum(payload, header.payloadSize) != header.checksum)
			{
				BACKEND_WARNF("CreateCollisionMeshFromStream: stream is corrupted (checksum mismatch); rejected");
				return;
			}
			cmd.outCollisionMesh = CreateMeshFromPayload(cmd.convex, payload, header.payloadSize);
			if (cmd.outCollisionMesh == nullptr)
			{
				BACKEND_WARNF("CreateCollisionMeshFromStream: PhysX rejected the mesh payload");
			}
		}

		void DestroyCollisionMesh(Handle, const Context&, Command& c)
		{
			const Handle h = static_cast<sd::DestroyCollisionMesh&>(c).collisionMesh;
			std::lock_guard<std::mutex> guard(g_meshMutex);
			auto* mesh = static_cast<MeshObj*>(Lookup(h, Kind::Mesh));
			if (mesh == nullptr || mesh->destroyed)
			{
				return;
			}
			mesh->destroyed = true;
			if (mesh->refs == 0)
			{
				FreeMesh(mesh);
			}
		}

		void ExtractCollisionMeshData(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<sd::ExtractCollisionMeshData&>(c);
			Physics::SDKCollisionMeshData data = {};
			MeshObj* mesh = PinMeshOf(sdkObject, ctx);

			std::vector<uint32_t> triangulated;
			if (mesh != nullptr && mesh->triangle != nullptr)
			{
				const physx::PxTriangleMesh* tri = mesh->triangle;
				data.vertices = reinterpret_cast<const float*>(tri->getVertices());
				data.vertexCount = tri->getNbVertices();
				data.vertexStride = sizeof(physx::PxVec3);
				data.indices = tri->getTriangles();
				data.indexCount = static_cast<size_t>(tri->getNbTriangles()) * 3;
				data.indexStride = (tri->getTriangleMeshFlags() & physx::PxTriangleMeshFlag::e16_BIT_INDICES) ? sizeof(uint16_t) : sizeof(uint32_t);
			}
			else if (mesh != nullptr && mesh->convex != nullptr)
			{
				// Convex meshes store polygons; hand out a triangle fan per polygon.
				const physx::PxConvexMesh* convex = mesh->convex;
				const physx::PxU8* indexBuffer = convex->getIndexBuffer();
				for (physx::PxU32 p = 0; p < convex->getNbPolygons(); ++p)
				{
					physx::PxHullPolygon polygon;
					if (!convex->getPolygonData(p, polygon))
					{
						continue;
					}
					for (uint32_t k = 1; k + 1 < polygon.mNbVerts; ++k)
					{
						triangulated.push_back(indexBuffer[polygon.mIndexBase]);
						triangulated.push_back(indexBuffer[polygon.mIndexBase + k]);
						triangulated.push_back(indexBuffer[polygon.mIndexBase + k + 1]);
					}
				}
				data.vertices = reinterpret_cast<const float*>(convex->getVertices());
				data.vertexCount = convex->getNbVertices();
				data.vertexStride = sizeof(physx::PxVec3);
				data.indices = triangulated.data();
				data.indexCount = triangulated.size();
				data.indexStride = sizeof(uint32_t);
			}

			if (cmd.callback != nullptr)
			{
				cmd.callback(cmd.ctx, data);
			}
			ReleaseMesh(mesh);
		}

		void RemapTriangleIndex(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<sd::RemapTriangleIndexToSourceIndex&>(c);
			cmd.outSourceMeshTriangleIndex = cmd.physicsMeshTriangleIndex;
			MeshObj* mesh = PinMeshOf(sdkObject, ctx);
			if (mesh != nullptr && mesh->triangle != nullptr)
			{
				const physx::PxU32* remap = mesh->triangle->getTrianglesRemap();
				if (remap != nullptr && cmd.physicsMeshTriangleIndex < mesh->triangle->getNbTriangles())
				{
					cmd.outSourceMeshTriangleIndex = remap[cmd.physicsMeshTriangleIndex];
				}
			}
			ReleaseMesh(mesh);
		}

		// ---- heightfields ----

		// Builds PhysX samples (row = x, column = z) for the cells [x0, x0 + w) x [z0, z0 + h) of a resolution^2 grid.
		// Heights come from the full grid (heightsCount == resolution^2) or from a w*h region buffer.
		bool BuildSamples(const Physics::HeightFieldDescriptor& d, int resolution, int x0, int z0, int w, int h,
			std::vector<physx::PxHeightFieldSample>& out, const char*& error)
		{
			const size_t full = static_cast<size_t>(resolution) * static_cast<size_t>(resolution);
			const size_t region = static_cast<size_t>(w) * static_cast<size_t>(h);
			if (d.heights == nullptr || d.heightsCount < 0)
			{
				error = "height field has no height samples";
				return false;
			}
			const size_t count = static_cast<size_t>(d.heightsCount);
			const bool fullGrid = count == full;
			if (!fullGrid && count != region)
			{
				error = "height sample count matches neither the full grid nor the updated region";
				return false;
			}

			// Hole mask: per cell ((resolution - 1)^2), per sample (resolution^2) or per updated sample (w*h); 0 marks a hole.
			const size_t cells = static_cast<size_t>(resolution - 1) * static_cast<size_t>(resolution - 1);
			int holeStride = 0;
			bool holeLocal = false;
			if (d.holes != nullptr && d.holesCount > 0)
			{
				const size_t holes = static_cast<size_t>(d.holesCount);
				if (holes == cells)
				{
					holeStride = resolution - 1;
				}
				else if (holes == full)
				{
					holeStride = resolution;
				}
				else if (holes == region)
				{
					holeStride = w;
					holeLocal = true;
				}
				else
				{
					BACKEND_WARNF("Height field hole mask size %d does not match the grid; holes ignored", d.holesCount);
				}
			}

			out.assign(region, physx::PxHeightFieldSample());
			for (int x = x0; x < x0 + w; ++x)
			{
				for (int z = z0; z < z0 + h; ++z)
				{
					const size_t src = fullGrid
						? (kHeightsZMajor ? static_cast<size_t>(z) * resolution + x : static_cast<size_t>(x) * resolution + z)
						: (kHeightsZMajor ? static_cast<size_t>(z - z0) * w + (x - x0) : static_cast<size_t>(x - x0) * h + (z - z0));
					physx::PxHeightFieldSample& sample = out[static_cast<size_t>(x - x0) * h + (z - z0)];
					sample.height = d.heights[src];

					if (holeStride > 0)
					{
						const int hx = holeLocal ? x - x0 : x;
						const int hz = holeLocal ? z - z0 : z;
						const int holeRows = holeLocal ? h : holeStride;
						if (hx < holeStride && hz < holeRows && d.holes[static_cast<size_t>(hz) * holeStride + hx] == 0)
						{
							sample.materialIndex0 = physx::PxBitAndByte(physx::PxHeightFieldMaterial::eHOLE);
							sample.materialIndex1 = physx::PxBitAndByte(physx::PxHeightFieldMaterial::eHOLE);
						}
					}
					// Built-in sets the tess flag on every sample (holes too: mat0 = 0xFF, mat1 = 0x7F).
					sample.setTessFlag();
				}
			}
			return true;
		}

		void CreateHeightField(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::CreateHeightField&>(c);
			cmd.heightField = nullptr;
			const Physics::HeightFieldDescriptor& d = cmd.descriptor;
			if (d.resolution < 2)
			{
				BACKEND_WARNF("CreateHeightField: resolution %d is too small; rejected", d.resolution);
				return;
			}

			std::vector<physx::PxHeightFieldSample> samples;
			const char* error = nullptr;
			if (!BuildSamples(d, d.resolution, 0, 0, d.resolution, d.resolution, samples, error))
			{
				BACKEND_WARNF("CreateHeightField: %s; rejected", error);
				return;
			}

			physx::PxHeightFieldDesc desc;
			desc.nbRows = d.resolution;
			desc.nbColumns = d.resolution;
			desc.format = physx::PxHeightFieldFormat::eS16_TM;
			desc.samples.data = samples.data();
			desc.samples.stride = sizeof(physx::PxHeightFieldSample);
			physx::PxHeightField* field = cooking::CreateHeightField(desc);
			if (field == nullptr)
			{
				BACKEND_WARNF("CreateHeightField: PhysX failed to create the height field");
				return;
			}
			cmd.heightField = NewHeightField(field, static_cast<uint32_t>(d.resolution));
		}

		void UpdateHeightFieldRegion(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<sd::UpdateHeightFieldRegion&>(c);
			HeightFieldObj* obj = AcquireHeightField(cmd.heightField);
			if (obj == nullptr)
			{
				return;
			}

			const int resolution = static_cast<int>(obj->resolution);
			const int x0 = cmd.xBase;
			const int z0 = cmd.yBase;
			const int w = cmd.width;
			const int h = cmd.height;
			const char* error = nullptr;
			std::vector<physx::PxHeightFieldSample> samples;
			bool ok = true;
			if (cmd.descriptor.resolution != resolution)
			{
				error = "descriptor resolution differs from the height field";
				ok = false;
			}
			else if (x0 < 0 || z0 < 0 || w < 1 || h < 1 || x0 + w > resolution || z0 + h > resolution)
			{
				error = "region is outside the height field";
				ok = false;
			}
			else
			{
				ok = BuildSamples(cmd.descriptor, resolution, x0, z0, w, h, samples, error);
			}

			std::vector<ShapeObj*> users;
			if (ok)
			{
				physx::PxHeightFieldDesc sub;
				sub.nbRows = w;
				sub.nbColumns = h;
				sub.format = physx::PxHeightFieldFormat::eS16_TM;
				sub.samples.data = samples.data();
				sub.samples.stride = sizeof(physx::PxHeightFieldSample);
				ok = obj->field->modifySamples(z0, x0, sub);
				if (!ok)
				{
					error = "PhysX rejected the new samples";
				}
				else
				{
					std::lock_guard<std::mutex> guard(g_meshMutex);
					users = obj->shapes;
				}
			}
			if (!ok)
			{
				BACKEND_WARNF("UpdateHeightFieldRegion: %s", error != nullptr ? error : "failed");
			}

			// Shapes must be handed the geometry again so the scene refreshes bounds and cached contacts.
			for (ShapeObj* shape : users)
			{
				Lock lock(Target{ Kind::Shape, shape, nullptr });
				if (lock && lock.get<ShapeObj>()->heightField == obj)
				{
					RefreshShapeGeometry(*lock.get<ShapeObj>());
				}
			}
			ReleaseHeightField(obj, nullptr);
		}

		void DestroyHeightField(Handle, const Context&, Command& c)
		{
			const Handle h = static_cast<sd::DestroyHeightField&>(c).heightField;
			std::lock_guard<std::mutex> guard(g_meshMutex);
			auto* obj = static_cast<HeightFieldObj*>(Lookup(h, Kind::HeightField));
			if (obj == nullptr || obj->destroyed)
			{
				return;
			}
			obj->destroyed = true;
			if (obj->refs == 0)
			{
				FreeHeightField(obj);
			}
		}
	}

	MeshObj* AcquireMesh(Handle h, Physics::GeometryType type)
	{
		std::lock_guard<std::mutex> guard(g_meshMutex);
		auto* mesh = static_cast<MeshObj*>(Lookup(h, Kind::Mesh));
		if (mesh == nullptr || mesh->destroyed)
		{
			return nullptr;
		}
		if (type != Physics::GeometryType::Invalid && (type == Physics::GeometryType::ConvexMesh) != (mesh->convex != nullptr))
		{
			return nullptr;
		}
		++mesh->refs;
		return mesh;
	}

	HeightFieldObj* AcquireHeightField(Handle h)
	{
		std::lock_guard<std::mutex> guard(g_meshMutex);
		auto* field = static_cast<HeightFieldObj*>(Lookup(h, Kind::HeightField));
		if (field == nullptr || field->destroyed)
		{
			return nullptr;
		}
		++field->refs;
		return field;
	}

	MeshObj* FindMesh(Handle h)
	{
		std::lock_guard<std::mutex> guard(g_meshMutex);
		return static_cast<MeshObj*>(Lookup(h, Kind::Mesh));
	}

	HeightFieldObj* FindHeightField(Handle h)
	{
		std::lock_guard<std::mutex> guard(g_meshMutex);
		return static_cast<HeightFieldObj*>(Lookup(h, Kind::HeightField));
	}

	void ReleaseMesh(MeshObj* mesh)
	{
		if (mesh == nullptr)
		{
			return;
		}
		std::lock_guard<std::mutex> guard(g_meshMutex);
		if (mesh->refs > 0)
		{
			--mesh->refs;
		}
		if (mesh->destroyed && mesh->refs == 0)
		{
			FreeMesh(mesh);
		}
	}

	void LinkHeightFieldShape(HeightFieldObj* field, ShapeObj* shape)
	{
		std::lock_guard<std::mutex> guard(g_meshMutex);
		field->shapes.push_back(shape);
	}

	void ReleaseHeightField(HeightFieldObj* field, ShapeObj* shape)
	{
		if (field == nullptr)
		{
			return;
		}
		std::lock_guard<std::mutex> guard(g_meshMutex);
		if (shape != nullptr)
		{
			field->shapes.erase(std::remove(field->shapes.begin(), field->shapes.end(), shape), field->shapes.end());
		}
		if (field->refs > 0)
		{
			--field->refs;
		}
		if (field->destroyed && field->refs == 0)
		{
			FreeHeightField(field);
		}
	}

	void ReleaseShapeGeometryRefs(ShapeObj& shape)
	{
		MeshObj* mesh = shape.mesh;
		HeightFieldObj* field = shape.heightField;
		shape.mesh = nullptr;
		shape.heightField = nullptr;
		ReleaseMesh(mesh);
		ReleaseHeightField(field, &shape);
	}

	void ReleaseAllMeshes()
	{
		std::lock_guard<std::mutex> guard(g_meshMutex);
		for (Handle h : AllHandles(Kind::Mesh))
		{
			if (auto* mesh = static_cast<MeshObj*>(Lookup(h, Kind::Mesh)))
			{
				FreeMesh(mesh);
			}
		}
		for (Handle h : AllHandles(Kind::HeightField))
		{
			if (auto* field = static_cast<HeightFieldObj*>(Lookup(h, Kind::HeightField)))
			{
				FreeHeightField(field);
			}
		}
	}

	void RegisterMeshHandlers()
	{
		using S = pc::Shape;
		Bind(S::CookCollisionMesh, &CookCollisionMesh);
		Bind(S::CookCollisionMeshStream, &CookCollisionMeshStream);
		Bind(S::CreateCollisionMeshFromStream, &CreateCollisionMeshFromStream);
		Bind(S::DestroyCollisionMesh, &DestroyCollisionMesh);
		Bind(S::ExtractCollisionMeshData, &ExtractCollisionMeshData);
		Bind(S::RemapTriangleIndexToSourceIndex, &RemapTriangleIndex);
		Bind(S::CreateHeightField, &CreateHeightField);
		Bind(S::UpdateHeightFieldRegion, &UpdateHeightFieldRegion);
		Bind(S::DestroyHeightField, &DestroyHeightField);
	}
}
