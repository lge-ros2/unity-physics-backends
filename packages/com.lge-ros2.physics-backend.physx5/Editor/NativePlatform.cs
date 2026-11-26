/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using UnityEditor;
using UnityEngine;

namespace Lge.PhysicsBackend.PhysX5.Editor
{
	// One entry per supported native library flavor. Both are installed so that an editor on either OS can build a
	// standalone player for the other one.
	public sealed class NativePlatform
	{
		public static readonly NativePlatform Linux = new NativePlatform(
			"linux", "libPhysX5Backend.so", "Linux", BuildTarget.StandaloneLinux64, RuntimePlatform.LinuxEditor);

		public static readonly NativePlatform Windows = new NativePlatform(
			"windows", "PhysX5Backend.dll", "Windows", BuildTarget.StandaloneWindows64, RuntimePlatform.WindowsEditor);

		public static readonly NativePlatform[] All = { Linux, Windows };

		// Folder name below Plugins~ in the package and below Assets/Plugins/PhysX5Backend in the project.
		public readonly string Folder;
		public readonly string FileName;
		// Value of the "OS" editor data of the PluginImporter.
		public readonly string EditorOs;
		public readonly BuildTarget Target;
		private readonly RuntimePlatform _editorPlatform;

		private NativePlatform(string folder, string fileName, string editorOs, BuildTarget target, RuntimePlatform editorPlatform)
		{
			Folder = folder;
			FileName = fileName;
			EditorOs = editorOs;
			Target = target;
			_editorPlatform = editorPlatform;
		}

		// The flavor the running editor loads; null on unsupported editors (e.g. macOS).
		public static NativePlatform Current
		{
			get
			{
				foreach (var platform in All)
				{
					if (platform._editorPlatform == Application.platform)
					{
						return platform;
					}
				}
				return null;
			}
		}

		public static NativePlatform ForBuildTarget(BuildTarget target)
		{
			foreach (var platform in All)
			{
				if (platform.Target == target)
				{
					return platform;
				}
			}
			return null;
		}

		public string AssetFolder => NativePluginInstaller.PluginRootFolder + "/" + Folder;

		public string AssetPath => AssetFolder + "/" + FileName;
	}
}
