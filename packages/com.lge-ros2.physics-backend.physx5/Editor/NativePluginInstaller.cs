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

		private const string VersionFileName = "VERSION";
		private const string LogPrefix = "[PhysicsBackend] ";

		static NativePluginInstaller()
		{
			if (NativePlatform.Current == null)
			{
				return;
			}

			UnityEditor.PackageManager.Events.registeringPackages += OnRegisteringPackages;

			// delayCall keeps the file/import work out of the domain reload / editor startup window.
			EditorApplication.delayCall += () => Install(false);
		}

		public static void Reinstall()
		{
			Install(true);
		}

		private static void Install(bool force)
		{
			var current = NativePlatform.Current;
			if (current == null || BuildPipeline.isBuildingPlayer)
			{
				return;
			}

			var package = PackageInfo.FindForAssembly(typeof(NativePluginInstaller).Assembly);
			if (package == null)
			{
				Debug.LogWarning(LogPrefix + "Package info for " + PackageName + " not found; skipping native plugin install.");
				return;
			}

			// Every flavor shipped in the package is installed: the editor loads its own one, the other one is only
			// needed to build a standalone player for the other OS.
			var currentUpdated = false;
			foreach (var platform in NativePlatform.All)
			{
				var updated = InstallPlatform(platform, package, force, platform == current);
				currentUpdated |= updated && platform == current;
			}

			if (currentUpdated)
			{
				NotifyRestartRequired();
			}
		}

		private static bool InstallPlatform(NativePlatform platform, PackageInfo package, bool force, bool isEditorPlatform)
		{
			var source = Path.Combine(package.resolvedPath, "Plugins~", platform.Folder, platform.FileName);
			if (!File.Exists(source))
			{
				if (isEditorPlatform)
				{
					Debug.LogWarning(LogPrefix + platform.FileName + " not found in package (" + source +
						"); skipping native plugin install.");
				}
				return false;
			}

			var projectRoot = Directory.GetCurrentDirectory();
			var targetDir = Path.Combine(projectRoot, platform.AssetFolder);
			var target = Path.Combine(targetDir, platform.FileName);
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

					Debug.Log(LogPrefix + "Installed " + platform.FileName + " " + package.version + " to " + platform.AssetFolder);
					if (firstInstall)
					{
						Debug.Log(LogPrefix + "Recommended: add '" + PluginRootFolder + "/' to your .gitignore.");
					}
				}
				CleanupStaleFiles(targetDir);
			}
			catch (Exception e)
			{
				Debug.LogError(LogPrefix + "Native plugin install failed (" + platform.Folder + "): " + e);
				return false;
			}

			if (updated)
			{
				AssetDatabase.ImportAsset(platform.AssetFolder, ImportAssetOptions.ForceUpdate | ImportAssetOptions.ImportRecursive);
			}

			var importerChanged = ConfigureImporter(platform);
			if (importerChanged && !updated)
			{
				Debug.Log(LogPrefix + "Plugin importer settings updated (" + platform.Folder + ").");
			}

			return updated;
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

		// Never overwrite the target in place: a loaded library that is rewritten crashes the editor (SIGSEGV on Linux).
		// Copy to a temp file in the same folder, get the old file out of the way and move the new one in. Linux can
		// unlink a mapped file; Windows refuses to delete a loaded DLL but allows renaming it, so it is moved aside
		// to a hidden name (leading dot: ignored by the AssetDatabase) and removed by CleanupStaleFiles later.
		private static void ReplaceFile(string source, string target)
		{
			var folder = Path.GetDirectoryName(target);
			var tempFile = Path.Combine(folder, "." + Path.GetFileName(target) + "." + Guid.NewGuid().ToString("N") + ".tmp");
			try
			{
				File.Copy(source, tempFile, true);
				if (File.Exists(target))
				{
					try
					{
						File.Delete(target);
					}
					catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
					{
						var aside = Path.Combine(folder, "." + Path.GetFileName(target) + "." + Guid.NewGuid().ToString("N") + ".old");
						File.Move(target, aside);
					}
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

		// Leftovers of ReplaceFile: files that were still loaded back then, removable once the editor restarted.
		private static void CleanupStaleFiles(string folder)
		{
			if (!Directory.Exists(folder))
			{
				return;
			}

			foreach (var file in Directory.GetFiles(folder, ".*.old"))
			{
				try
				{
					File.Delete(file);
				}
				catch (Exception)
				{
					// Still loaded by this editor session; try again next time.
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

		private static bool ConfigureImporter(NativePlatform platform)
		{
			var importer = AssetImporter.GetAtPath(platform.AssetPath) as PluginImporter;
			if (importer == null)
			{
				return false;
			}

			var matches =
				!importer.GetCompatibleWithAnyPlatform() &&
				importer.GetCompatibleWithEditor() &&
				importer.GetCompatibleWithPlatform(platform.Target) &&
				importer.isPreloaded &&
				importer.GetEditorData("CPU") == "x86_64" &&
				importer.GetEditorData("OS") == platform.EditorOs &&
				importer.GetPlatformData(platform.Target, "CPU") == "x86_64";
			if (matches)
			{
				return false;
			}

			importer.SetCompatibleWithAnyPlatform(false);
			importer.SetCompatibleWithEditor(true);
			importer.SetEditorData("CPU", "x86_64");
			importer.SetEditorData("OS", platform.EditorOs);
			importer.SetCompatibleWithPlatform(platform.Target, true);
			importer.SetPlatformData(platform.Target, "CPU", "x86_64");
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
				foreach (var platform in NativePlatform.All)
				{
					var folder = Path.Combine(projectRoot, platform.AssetFolder);
					if (Directory.Exists(folder))
					{
						Directory.Delete(folder, true);
					}
					DeleteIfExists(folder + ".meta");
				}

				var rootFolder = Path.Combine(projectRoot, PluginRootFolder);
				if (Directory.Exists(rootFolder) && !Directory.EnumerateFileSystemEntries(rootFolder).Any())
				{
					Directory.Delete(rootFolder, true);
					DeleteIfExists(rootFolder + ".meta");
				}

				Debug.Log(LogPrefix + "Package removed; deleted " + PluginRootFolder);
				EditorApplication.delayCall += AssetDatabase.Refresh;
			}
			catch (Exception e)
			{
				Debug.LogWarning(LogPrefix + "Failed to clean up " + PluginRootFolder + ": " + e.Message);
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
