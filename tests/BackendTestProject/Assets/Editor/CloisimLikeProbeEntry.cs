/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using UnityEditor;
using UnityEditor.SceneManagement;

public static class CloisimLikeProbeEntry
{
	// batchmode: -executeMethod CloisimLikeProbeEntry.Run  (env CLOISIM_PROBE=1 CLOISIM_PROBE_QUIT=1)
	public static void Run()
	{
		EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
		EditorApplication.EnterPlaymode();
	}
}
