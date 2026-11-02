/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using System.Collections;
using Lge.PhysicsBackend.PhysX5;
using UnityEngine;

// Attach to an empty GameObject in a new scene. Builds a floor and a falling box at runtime.
public class BackendProbe : MonoBehaviour
{
	[SerializeField] private float _logDelaySeconds = 3f;

	private Transform _box;

	private void Start()
	{
		PhysicsBackendInfo.LogSummary();
		CreateScene();
		StartCoroutine(LogBoxHeight());
	}

	private void CreateScene()
	{
		var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
		floor.name = "Floor";
		floor.transform.position = new Vector3(0f, -0.5f, 0f);
		floor.transform.localScale = new Vector3(20f, 1f, 20f);

		var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
		box.name = "Box";
		box.transform.position = new Vector3(0f, 5f, 0f);
		box.AddComponent<Rigidbody>();
		_box = box.transform;

		var cam = Camera.main;
		if (cam == null)
		{
			var camObject = new GameObject("Main Camera");
			camObject.tag = "MainCamera";
			cam = camObject.AddComponent<Camera>();
		}
		cam.transform.position = new Vector3(0f, 3f, -10f);
		cam.transform.LookAt(new Vector3(0f, 1f, 0f));

		if (FindFirstObjectByType<Light>() == null)
		{
			var lightObject = new GameObject("Directional Light");
			lightObject.AddComponent<Light>().type = LightType.Directional;
			lightObject.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
		}
	}

	private IEnumerator LogBoxHeight()
	{
		yield return new WaitForSeconds(_logDelaySeconds);
		Debug.Log("[BackendProbe] Box y after " + _logDelaySeconds + "s: " + _box.position.y.ToString("F3") +
			" (expected ~0.5 when resting on the floor), active backend: " + PhysicsBackendInfo.Current.name);
	}
}
