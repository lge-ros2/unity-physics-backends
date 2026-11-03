/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Lge.PhysicsBackend.PhysX5.Editor
{
	public static class PhysicsBackendMenu
	{
		private const string MenuRoot = "Tools/Physics Backend/";
		private const string MenuPhysX4 = MenuRoot + "PhysX 4 (Built-in)";
		private const string MenuPhysX5 = MenuRoot + "PhysX 5";
		private const string MenuPhysX4Ref = MenuRoot + "PhysX 4 Ref (dev)";
		// Development-only reference backend (PhysX 4.1 built from the same glue); not shipped in this package.
		private const uint PhysX4RefIntegrationId = 0x4C505834;
		private const string MenuLogInfo = MenuRoot + "Log Backend Info";
		private const string MenuReinstall = MenuRoot + "Reinstall Native Plugin";
		private const string MenuRestart = MenuRoot + "Restart Editor";

		[MenuItem(MenuPhysX4, false, 100)]
		private static void SelectPhysX4()
		{
			SelectBackend(PhysicsBackendSettings.GetBuiltInPhysX4Id(), "PhysX 4 (Built-in)");
		}

		[MenuItem(MenuPhysX4, true)]
		private static bool ValidateSelectPhysX4()
		{
			var isCurrent = PhysicsBackendSettings.TryGetCurrentBackendId(out var id) &&
				id == PhysicsBackendSettings.GetBuiltInPhysX4Id();
			Menu.SetChecked(MenuPhysX4, isCurrent);
			return true;
		}

		[MenuItem(MenuPhysX5, false, 101)]
		private static void SelectPhysX5()
		{
			SelectBackend(PhysicsBackendInfo.IntegrationId, PhysicsBackendInfo.BackendName);
		}

		[MenuItem(MenuPhysX5, true)]
		private static bool ValidateSelectPhysX5()
		{
			var isCurrent = PhysicsBackendSettings.TryGetCurrentBackendId(out var id) &&
				id == PhysicsBackendInfo.IntegrationId;
			Menu.SetChecked(MenuPhysX5, isCurrent);
			return PhysicsBackendInfo.IsPhysX5Registered;
		}

		[MenuItem(MenuPhysX4Ref, false, 102)]
		private static void SelectPhysX4Ref()
		{
			SelectBackend(PhysX4RefIntegrationId, "PhysX 4 Ref");
		}

		[MenuItem(MenuPhysX4Ref, true)]
		private static bool ValidateSelectPhysX4Ref()
		{
			var isCurrent = PhysicsBackendSettings.TryGetCurrentBackendId(out var id) &&
				id == PhysX4RefIntegrationId;
			Menu.SetChecked(MenuPhysX4Ref, isCurrent);
			return PhysicsBackendInfo.IsRegistered(PhysX4RefIntegrationId);
		}

		[MenuItem(MenuLogInfo, false, 200)]
		private static void LogBackendInfo()
		{
			PhysicsBackendInfo.LogSummary();
		}

		[MenuItem(MenuReinstall, false, 201)]
		private static void ReinstallNativePlugin()
		{
			NativePluginInstaller.Reinstall();
		}

		[MenuItem(MenuRestart, false, 202)]
		private static void RestartEditor()
		{
			NativePluginInstaller.RestartEditor();
		}

		private static void SelectBackend(uint id, string displayName)
		{
			if (PhysicsBackendSettings.TryGetCurrentBackendId(out var currentId) && currentId == id)
			{
				return;
			}

			if (!PhysicsBackendSettings.TrySetCurrentBackendId(id))
			{
				return;
			}

			Debug.Log("[PhysicsBackend] Selected " + displayName + " (id=" + id + "). Restart the editor to apply.");

			if (Application.isBatchMode)
			{
				return;
			}

			var restart = EditorUtility.DisplayDialog(
				"Physics Backend",
				displayName + " selected.\nThe editor must be restarted to apply the change.",
				"Restart Now",
				"Later");
			if (restart && EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
			{
				EditorApplication.OpenProject(Directory.GetCurrentDirectory());
			}
		}
	}
}
