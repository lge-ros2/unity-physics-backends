/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace Lge.PhysicsBackend.PhysX5
{
	public static class PhysicsBackendInfo
	{
		public const uint IntegrationId = 0x4C504835;
		public const string BackendName = "PhysX 5";
		public const string NativeLibraryFileName = "libPhysX5Backend.so";

		private const string LogPrefix = "[PhysicsBackend] ";
		private const uint InvalidIntegrationId = 0;

		private static readonly object _lock = new object();
		private static MethodInfo _getInfosMethod;
		private static bool _getInfosResolved;

		public static IntegrationInfo Current => Physics.GetCurrentIntegrationInfo();

		public static bool IsActive
		{
			get
			{
				var current = Current;
				return !current.isFallback && current.id == IntegrationId;
			}
		}

		// Physics.GetIntegrationInfos() is internal in Unity 6000.6, so the list is read through reflection on the
		// private native binding. Returns false when that is not possible (e.g. stripped by IL2CPP).
		public static bool TryGetRegisteredIntegrations(out List<IntegrationInfo> infos)
		{
			infos = new List<IntegrationInfo>();

			var method = ResolveGetInfosMethod();
			if (method == null)
			{
				return false;
			}

			try
			{
				var args = new object[] { IntPtr.Zero, 0UL };
				method.Invoke(null, args);
				var pointer = (IntPtr)args[0];
				var count = (ulong)args[1];
				CopyInfos(pointer, count, infos);
				return true;
			}
			catch (Exception e)
			{
				Debug.LogWarning(LogPrefix + "Failed to enumerate integrations: " + e.Message);
				infos.Clear();
				return false;
			}
		}

		public static bool IsRegistered(uint id)
		{
			if (TryGetRegisteredIntegrations(out var infos))
			{
				foreach (var info in infos)
				{
					if (info.id == id)
					{
						return true;
					}
				}
				return false;
			}

			// Enumeration unavailable: only the current integration is known.
			return Current.id == id;
		}

		public static bool IsPhysX5Registered => IsRegistered(IntegrationId);

		public static void LogSummary()
		{
			var current = Current;
			var sb = new StringBuilder();
			sb.AppendLine(LogPrefix + "Registered integrations:");

			var enumerated = TryGetRegisteredIntegrations(out var infos);
			var physx5Registered = false;
			if (enumerated)
			{
				foreach (var info in infos)
				{
					sb.AppendLine("  - " + Format(info) + (info.id == current.id ? "  <== current" : string.Empty));
					physx5Registered |= (info.id == IntegrationId);
				}
			}
			else
			{
				sb.AppendLine("  (enumeration unavailable)");
				physx5Registered = (current.id == IntegrationId);
			}

			sb.AppendLine(LogPrefix + "Current: " + Format(current));
			sb.Append(LogPrefix + BackendName + " registered: " + physx5Registered + ", active: " + IsActive);
			Debug.Log(sb.ToString());

			if (current.isFallback || current.id == InvalidIntegrationId)
			{
				Debug.LogError(LogPrefix + "Current physics integration is a fallback/None ('" + current.name +
					"'). The selected backend failed to load.");
			}
		}

		private static string Format(in IntegrationInfo info)
		{
			return "id=" + info.id + " (0x" + info.id.ToString("X8") + ") name='" + info.name +
				"' fallback=" + info.isFallback + " desc='" + info.description + "'";
		}

		private static MethodInfo ResolveGetInfosMethod()
		{
			lock (_lock)
			{
				if (!_getInfosResolved)
				{
					_getInfosResolved = true;
					try
					{
						_getInfosMethod = typeof(Physics).GetMethod(
							"GetIntegrationInfos",
							BindingFlags.NonPublic | BindingFlags.Static,
							null,
							new[] { typeof(IntPtr).MakeByRefType(), typeof(ulong).MakeByRefType() },
							null);
					}
					catch (Exception)
					{
						_getInfosMethod = null;
					}
				}
				return _getInfosMethod;
			}
		}

		private static unsafe void CopyInfos(IntPtr pointer, ulong count, List<IntegrationInfo> infos)
		{
			if (pointer == IntPtr.Zero || count == 0)
			{
				return;
			}

			var p = (IntegrationInfo*)pointer;
			for (ulong i = 0; i < count; i++)
			{
				infos.Add(p[i]);
			}
		}
	}
}
