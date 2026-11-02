/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using UnityEditor;
using UnityEngine;

namespace Lge.PhysicsBackend.PhysX5.Editor
{
	internal static class PhysicsBackendSettings
	{
		public const string AssetPath = "ProjectSettings/DynamicsManager.asset";
		public const string BackendIdProperty = "m_CurrentBackendId";
		public const uint BuiltInPhysX4DefaultId = 4072204805;

		private const string BuiltInName = "PhysX";

		public static bool TryGetCurrentBackendId(out uint id)
		{
			id = 0;
			var so = Load();
			var property = so?.FindProperty(BackendIdProperty);
			if (property == null)
			{
				return false;
			}

			id = property.uintValue;
			return true;
		}

		public static bool TrySetCurrentBackendId(uint id)
		{
			var so = Load();
			var property = so?.FindProperty(BackendIdProperty);
			if (property == null)
			{
				Debug.LogError("[PhysicsBackend] '" + BackendIdProperty + "' not found in " + AssetPath);
				return false;
			}

			if (property.uintValue == id)
			{
				return true;
			}

			property.uintValue = id;
			so.ApplyModifiedPropertiesWithoutUndo();
			EditorUtility.SetDirty(so.targetObject);
			AssetDatabase.SaveAssets();
			return true;
		}

		public static uint GetBuiltInPhysX4Id()
		{
			if (PhysicsBackendInfo.TryGetRegisteredIntegrations(out var infos))
			{
				foreach (var info in infos)
				{
					if (!info.isFallback && info.name == BuiltInName)
					{
						return info.id;
					}
				}
			}
			return BuiltInPhysX4DefaultId;
		}

		private static SerializedObject Load()
		{
			var assets = AssetDatabase.LoadAllAssetsAtPath(AssetPath);
			if (assets == null || assets.Length == 0 || assets[0] == null)
			{
				Debug.LogError("[PhysicsBackend] Failed to load " + AssetPath);
				return null;
			}
			return new SerializedObject(assets[0]);
		}
	}
}
