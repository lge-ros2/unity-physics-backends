/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEditor.SceneManagement;
using UnityEngine;
using PackageInfo = UnityEditor.PackageManager.PackageInfo;

namespace Lge.PhysicsBackend.PhysX5.Editor
{
	[InitializeOnLoad]
	public static class NativePluginInstaller
	{
		public const string PackageName = "com.lge-ros2.physics-backend.physx5";
		public const string PluginRootFolder = "Assets/Plugins/PhysX5Backend";
		public const string PluginFolder = PluginRootFolder + "/linux";
		public const string PluginAssetPath = PluginFolder + "/" + PhysicsBackendInfo.NativeLibraryFileName;

		private const string VersionFileName = "VERSION";
		private const string LogPrefix = "[PhysicsBackend] ";

		static NativePluginInstaller()
		{
			if (!IsLinuxEditor)
			{
				return;
			}

			UnityEditor.PackageManager.Events.registeringPackages += OnRegisteringPackages;

			// delayCall keeps the file/import work out of the domain reload / editor startup window.
			EditorApplication.delayCall += () => Install(false);
		}

		private static bool IsLinuxEditor => Application.platform == RuntimePlatform.LinuxEditor;

		public static void Reinstall()
		{
			Install(true);
		}

		private static void Install(bool force)
		{
			if (!IsLinuxEditor || BuildPipeline.isBuildingPlayer)
			{
				return;
			}

			var package = PackageInfo.FindForAssembly(typeof(NativePluginInstaller).Assembly);
			if (package == null)
			{
				Debug.LogWarning(LogPrefix + "Package info for " + PackageName + " not found; skipping native plugin install.");
				return;
			}

			var source = Path.Combine(package.resolvedPath, "Plugins~", "linux", PhysicsBackendInfo.NativeLibraryFileName);
			if (!File.Exists(source))
			{
				Debug.LogWarning(LogPrefix + PhysicsBackendInfo.NativeLibraryFileName + " not found in package (" + source +
					"); skipping native plugin install.");
				return;
			}

			var projectRoot = Directory.GetCurrentDirectory();
			var targetDir = Path.Combine(projectRoot, PluginFolder);
			var target = Path.Combine(targetDir, PhysicsBackendInfo.NativeLibraryFileName);
			var versionFile = Path.Combine(targetDir, VersionFileName);

			var updated = false;
			try
			{
				var sourceHash = ComputeSha256(source);
				var firstInstall = !File.Exists(target);

				if (force || firstInstall || !IsUpToDate(target, versionFile, source, sourceHash))
				{
					Directory.CreateDirectory(targetDir);
					ReplaceFile(source, target);
					File.WriteAllText(versionFile, "version=" + package.version + "\nsha256=" + sourceHash + "\n");
					updated = true;

					Debug.Log(LogPrefix + "Installed " + PhysicsBackendInfo.NativeLibraryFileName + " " + package.version +
						" to " + PluginFolder);
					if (firstInstall)
					{
						Debug.Log(LogPrefix + "Recommended: add '" + PluginRootFolder + "/' to your .gitignore.");
					}
				}
			}
			catch (Exception e)
			{
				Debug.LogError(LogPrefix + "Native plugin install failed: " + e);
				return;
			}

			if (updated)
			{
				AssetDatabase.ImportAsset(PluginRootFolder, ImportAssetOptions.ForceUpdate | ImportAssetOptions.ImportRecursive);
			}

			var importerChanged = ConfigureImporter();

			if (updated)
			{
				NotifyRestartRequired();
			}
			else if (importerChanged)
			{
				Debug.Log(LogPrefix + "Plugin importer settings updated.");
			}
		}

		private static bool IsUpToDate(string target, string versionFile, string source, string sourceHash)
		{
			if (!File.Exists(versionFile))
			{
				return false;
			}

			var installedHash = File.ReadAllLines(versionFile)
				.Where(line => line.StartsWith("sha256=", StringComparison.Ordinal))
				.Select(line => line.Substring("sha256=".Length).Trim())
				.FirstOrDefault();

			return string.Equals(installedHash, sourceHash, StringComparison.OrdinalIgnoreCase) &&
				new FileInfo(target).Length == new FileInfo(source).Length;
		}

		// Never overwrite the target in place: a loaded .so that is rewritten crashes the editor (SIGSEGV).
		// Copy to a temp file in the same folder, remove the old file (its inode stays mapped) and move the new one in.
		private static void ReplaceFile(string source, string target)
		{
			var tempFile = Path.Combine(Path.GetDirectoryName(target), "." + Path.GetFileName(target) + "." + Guid.NewGuid().ToString("N") + ".tmp");
			try
			{
				File.Copy(source, tempFile, true);
				if (File.Exists(target))
				{
					File.Delete(target);
				}
				File.Move(tempFile, target);
			}
			finally
			{
				if (File.Exists(tempFile))
				{
					File.Delete(tempFile);
				}
			}
		}

		private static string ComputeSha256(string path)
		{
			using (var stream = File.OpenRead(path))
			using (var sha = SHA256.Create())
			{
				var hash = sha.ComputeHash(stream);
				var sb = new StringBuilder(hash.Length * 2);
				foreach (var b in hash)
				{
					sb.Append(b.ToString("x2"));
				}
				return sb.ToString();
			}
		}

		private static bool ConfigureImporter()
		{
			var importer = AssetImporter.GetAtPath(PluginAssetPath) as PluginImporter;
			if (importer == null)
			{
				return false;
			}

			var matches =
				!importer.GetCompatibleWithAnyPlatform() &&
				importer.GetCompatibleWithEditor() &&
				importer.GetCompatibleWithPlatform(BuildTarget.StandaloneLinux64) &&
				importer.isPreloaded &&
				importer.GetEditorData("CPU") == "x86_64" &&
				importer.GetEditorData("OS") == "Linux" &&
				importer.GetPlatformData(BuildTarget.StandaloneLinux64, "CPU") == "x86_64";
			if (matches)
			{
				return false;
			}

			importer.SetCompatibleWithAnyPlatform(false);
			importer.SetCompatibleWithEditor(true);
			importer.SetEditorData("CPU", "x86_64");
			importer.SetEditorData("OS", "Linux");
			importer.SetCompatibleWithPlatform(BuildTarget.StandaloneLinux64, true);
			importer.SetPlatformData(BuildTarget.StandaloneLinux64, "CPU", "x86_64");
			importer.isPreloaded = true;
			importer.SaveAndReimport();
			return true;
		}

		private static void NotifyRestartRequired()
		{
			// No modal dialog: updates happen on every package change. The new library loads on the next editor start
			// (or via Tools/Physics Backend/Restart Editor).
			Debug.LogWarning(LogPrefix + "Native plugin installed/updated. Restart the editor to load it " +
				"(Tools/Physics Backend/Restart Editor).");
		}

		public static void RestartEditor()
		{
			if (EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
			{
				EditorApplication.OpenProject(Directory.GetCurrentDirectory());
			}
		}

		private static void OnRegisteringPackages(PackageRegistrationEventArgs args)
		{
			if (args.removed == null || !args.removed.Any(p => p.name == PackageName))
			{
				return;
			}

			try
			{
				var projectRoot = Directory.GetCurrentDirectory();
				var folder = Path.Combine(projectRoot, PluginFolder);
				if (Directory.Exists(folder))
				{
					Directory.Delete(folder, true);
				}
				DeleteIfExists(folder + ".meta");

				var rootFolder = Path.Combine(projectRoot, PluginRootFolder);
				if (Directory.Exists(rootFolder) && !Directory.EnumerateFileSystemEntries(rootFolder).Any())
				{
					Directory.Delete(rootFolder, true);
					DeleteIfExists(rootFolder + ".meta");
				}

				Debug.Log(LogPrefix + "Package removed; deleted " + PluginFolder);
				EditorApplication.delayCall += AssetDatabase.Refresh;
			}
			catch (Exception e)
			{
				Debug.LogWarning(LogPrefix + "Failed to clean up " + PluginFolder + ": " + e.Message);
			}
		}

		private static void DeleteIfExists(string path)
		{
			if (File.Exists(path))
			{
				File.Delete(path);
			}
		}
	}
}
