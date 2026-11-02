#include "core/Dispatch.h"
#include "physx/common/PhysXSdk.h"
#include "physx/common/PxObjects.h"

#include "PhysicsGeneralCommandData.h"
#include "PhysicsMaterialCommandData.h"

namespace backend::pxw
{
	namespace pc = PhysicsCommands;
	namespace sdkd = PhysicsCommands::SdkData;
	namespace matd = PhysicsCommands::MaterialData;

	namespace
	{
		physx::PxCombineMode::Enum ToPxCombine(Physics::MaterialCombineMode mode)
		{
			switch (mode)
			{
			case Physics::MaterialCombineMode::Multiply: return physx::PxCombineMode::eMULTIPLY;
			case Physics::MaterialCombineMode::Minimum: return physx::PxCombineMode::eMIN;
			case Physics::MaterialCombineMode::Maximum: return physx::PxCombineMode::eMAX;
			default: return physx::PxCombineMode::eAVERAGE;
			}
		}

		// The target may arrive as sdkObject or ctx.body; fall back to the default material.
		MaterialObj* ResolveMaterial(Handle sdkObject, const Context& ctx, Handle* outHandle = nullptr)
		{
			Handle h = Pick(Kind::Material, sdkObject, ctx.body);
			if (h == nullptr)
			{
				MaterialObj* def = DefaultMaterial();
				h = def;
			}
			if (outHandle != nullptr)
			{
				*outHandle = h;
			}
			return static_cast<MaterialObj*>(Lookup(h, Kind::Material));
		}

		void SetErrorVerbosityLevel(Handle, const Context&, Command& c)
		{
			physx_sdk::SetErrorVerbosity(static_cast<int>(static_cast<sdkd::SetErrorVerbosityLevel&>(c).value));
		}

		void CreateMaterial(Handle, const Context&, Command& c)
		{
			auto& cmd = static_cast<matd::CreateMaterial&>(c);
			cmd.outMaterial = nullptr;
			physx::PxPhysics* physics = physx_sdk::GetPhysics();
			if (physics == nullptr)
			{
				return;
			}
			auto* m = new MaterialObj();
			m->material = physics->createMaterial(0.6f, 0.6f, 0.0f);
			if (m->material == nullptr)
			{
				delete m;
				return;
			}
			m->userData = cmd.userData;
			compat::ApplyImprovedPatchFriction(*m->material, ImprovedPatchFrictionEnabled());
			Register(m, Kind::Material, m, LooseState());
			cmd.outMaterial = m;
		}

		void ReleaseMaterial(Handle sdkObject, const Context& ctx, Command&)
		{
			Handle h = Pick(Kind::Material, sdkObject, ctx.body);
			if (h == nullptr || h == DefaultMaterial())
			{
				return;
			}
			// Shapes hold their own reference on the PxMaterial, so releasing ours is always safe.
			auto* m = static_cast<MaterialObj*>(Lookup(h, Kind::Material));
			Unregister(h);
			if (m != nullptr)
			{
				m->material->release();
				delete m;
			}
		}

		void GetDefaultMaterial(Handle, const Context&, Command& c)
		{
			static_cast<matd::GetDefaultMaterial&>(c).value = DefaultMaterial();
		}

		void SetImprovedPatchFrictionEnabledGlobal(Handle, const Context&, Command& c)
		{
			const bool enabled = static_cast<matd::SetImprovedPatchFrictionEnabledGlobal&>(c).value;
			SetImprovedPatchFrictionEnabled(enabled);
			for (Handle h : AllHandles(Kind::Material))
			{
				if (auto* m = static_cast<MaterialObj*>(Lookup(h, Kind::Material)))
				{
					compat::ApplyImprovedPatchFriction(*m->material, enabled);
				}
			}
		}

		void SetMaterialConfiguration(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<matd::SetMaterialConfiguration&>(c);
			if (MaterialObj* m = ResolveMaterial(sdkObject, ctx))
			{
				m->material->setDynamicFriction(cmd.dynamicFriction < 0.0f ? 0.0f : cmd.dynamicFriction);
				m->material->setStaticFriction(cmd.staticFriction < 0.0f ? 0.0f : cmd.staticFriction);
				float bounce = cmd.bounciness;
				bounce = bounce < 0.0f ? 0.0f : (bounce > 1.0f ? 1.0f : bounce);
				m->material->setRestitution(bounce);
			}
		}

		void GetMaterialConfiguration(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<matd::GetMaterialConfiguration&>(c);
			if (MaterialObj* m = ResolveMaterial(sdkObject, ctx))
			{
				cmd.dynamicFriction = m->material->getDynamicFriction();
				cmd.staticFriction = m->material->getStaticFriction();
				cmd.bounciness = m->material->getRestitution();
			}
			else
			{
				cmd.dynamicFriction = 0.6f;
				cmd.staticFriction = 0.6f;
				cmd.bounciness = 0.0f;
			}
		}

		void SetMaterialCombineModes(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<matd::SetMaterialCombineModes&>(c);
			if (MaterialObj* m = ResolveMaterial(sdkObject, ctx))
			{
				m->frictionCombine = cmd.friction;
				m->bounceCombine = cmd.bounciness;
				m->material->setFrictionCombineMode(ToPxCombine(cmd.friction));
				m->material->setRestitutionCombineMode(ToPxCombine(cmd.bounciness));
			}
		}

		void GetMaterialCombineModes(Handle sdkObject, const Context& ctx, Command& c)
		{
			auto& cmd = static_cast<matd::GetMaterialCombineModes&>(c);
			MaterialObj* m = ResolveMaterial(sdkObject, ctx);
			cmd.friction = m != nullptr ? m->frictionCombine : Physics::MaterialCombineMode::Average;
			cmd.bounciness = m != nullptr ? m->bounceCombine : Physics::MaterialCombineMode::Average;
		}
	}

	void RegisterSdkHandlers()
	{
		Bind(pc::SDK::SetErrorVerbosityLevel, &SetErrorVerbosityLevel);
		Bind(pc::SDK::CreateMaterial, &CreateMaterial);
		Bind(pc::SDK::ReleaseMaterial, &ReleaseMaterial);
		Bind(pc::SDK::GetDefaultMaterial, &GetDefaultMaterial);
		Bind(pc::SDK::SetImprovedPatchFrictionEnabledGlobal, &SetImprovedPatchFrictionEnabledGlobal);
		Bind(pc::SDK::SetMaterialConfiguration, &SetMaterialConfiguration);
		Bind(pc::SDK::GetMaterialConfiguration, &GetMaterialConfiguration);
		Bind(pc::SDK::SetMaterialCombineModes, &SetMaterialCombineModes);
		Bind(pc::SDK::GetMaterialCombineModes, &GetMaterialCombineModes);
	}
}
