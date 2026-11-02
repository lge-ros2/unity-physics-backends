/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace Lge.PhysicsBackend.PhysX5.Editor
{
	// Selects the physics backend for a build via "-physicsBackend physx4|physx5" and keeps the PhysX 5 native
	// library out of Standalone Linux builds that do not use it.
	public sealed class BuildBackendSelector : IPreprocessBuildWithReport, IPostprocessBuildWithReport
	{
		private const string CommandLineArg = "-physicsBackend";
		private const string LogPrefix = "[PhysicsBackend] ";

		// Persisted marker so the importer state is restored even if the build fails before the post-process step.
		private static readonly string MarkerPath = Path.Combine("Library", "Lge.PhysicsBackend.PhysX5.PluginDisabledForBuild");

		public int callbackOrder => -100;

		[InitializeOnLoadMethod]
		private static void RestoreAfterFailedBuild()
		{
			if (!File.Exists(MarkerPath))
			{
				return;
			}

			EditorApplication.delayCall += () =>
			{
				if (!BuildPipeline.isBuildingPlayer)
				{
					RestorePluginCompatibility();
				}
			};
		}

		public void OnPreprocessBuild(BuildReport report)
		{
			var requestedId = ParseRequestedBackendId();
			if (requestedId.HasValue)
			{
				if (!PhysicsBackendSettings.TrySetCurrentBackendId(requestedId.Value))
				{
					throw new BuildFailedException(LogPrefix + "Failed to set physics backend id in " + PhysicsBackendSettings.AssetPath);
				}
				Debug.Log(LogPrefix + "Build physics backend set to id=" + requestedId.Value);
			}

			if (report.summary.platform != BuildTarget.StandaloneLinux64)
			{
				return;
			}

			if (!PhysicsBackendSettings.TryGetCurrentBackendId(out var finalId) || finalId == PhysicsBackendInfo.IntegrationId)
			{
				return;
			}

			try
			{
				var importer = AssetImporter.GetAtPath(NativePluginInstaller.PluginAssetPath) as PluginImporter;
				if (importer == null || !importer.GetCompatibleWithPlatform(BuildTarget.StandaloneLinux64))
				{
					return;
				}

				File.WriteAllText(MarkerPath, string.Empty);
				importer.SetCompatibleWithPlatform(BuildTarget.StandaloneLinux64, false);
				importer.SaveAndReimport();
				Debug.Log(LogPrefix + "Excluded " + PhysicsBackendInfo.NativeLibraryFileName + " from this build (PhysX 5 not selected).");
			}
			catch (Exception)
			{
				RestorePluginCompatibility();
				throw;
			}
		}

		public void OnPostprocessBuild(BuildReport report)
		{
			RestorePluginCompatibility();
		}

		private static void RestorePluginCompatibility()
		{
			try
			{
				var importer = AssetImporter.GetAtPath(NativePluginInstaller.PluginAssetPath) as PluginImporter;
				if (importer != null && !importer.GetCompatibleWithPlatform(BuildTarget.StandaloneLinux64))
				{
					importer.SetCompatibleWithPlatform(BuildTarget.StandaloneLinux64, true);
					importer.SaveAndReimport();
					Debug.Log(LogPrefix + "Restored " + PhysicsBackendInfo.NativeLibraryFileName + " Standalone Linux compatibility.");
				}
			}
			finally
			{
				if (File.Exists(MarkerPath))
				{
					File.Delete(MarkerPath);
				}
			}
		}

		private static uint? ParseRequestedBackendId()
		{
			var args = Environment.GetCommandLineArgs();
			for (var i = 0; i < args.Length; i++)
			{
				if (!string.Equals(args[i], CommandLineArg, StringComparison.OrdinalIgnoreCase))
				{
					continue;
				}

				if (i + 1 >= args.Length)
				{
					throw new BuildFailedException(LogPrefix + CommandLineArg + " requires a value: physx4 or physx5");
				}

				switch (args[i + 1].ToLowerInvariant())
				{
					case "physx4":
						return PhysicsBackendSettings.GetBuiltInPhysX4Id();
					case "physx5":
						return PhysicsBackendInfo.IntegrationId;
					default:
						throw new BuildFailedException(LogPrefix + "Invalid " + CommandLineArg + " value '" + args[i + 1] +
							"'. Expected physx4 or physx5.");
				}
			}

			return null;
		}
	}
}
