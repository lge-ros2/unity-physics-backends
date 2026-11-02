/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using System.Collections;
using System.Text;
using UnityEngine;

// Headless repro of CLOiSim's "place a prop box on the ground" flow.
// Runs only when env CLOISIM_PROBE=1 (see Editor/CloisimLikeProbeEntry.cs).
public static class CloisimLikeProbe
{
	[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
	private static void Init()
	{
		if (System.Environment.GetEnvironmentVariable("CLOISIM_PROBE") != "1")
			return;

		var go = new GameObject("CloisimLikeProbe");
		go.AddComponent<Runner>();
	}

	private class Runner : MonoBehaviour
	{
		private IEnumerator Start()
		{
			var sb = new StringBuilder();
			var info = Physics.GetCurrentIntegrationInfo();
			sb.AppendLine("[Probe] backend=" + info.name + " id=" + info.id);

			// ground: non-convex static MeshCollider on a Plane mesh (CLOiSim ground_plane)
			var tmp = GameObject.CreatePrimitive(PrimitiveType.Plane);
			var planeMesh = tmp.GetComponent<MeshFilter>().sharedMesh;
			Destroy(tmp);

			var ground = new GameObject("ground_plane");
			ground.transform.position = Vector3.zero;
			ground.transform.localScale = new Vector3(10f, 1f, 10f);
			var mc = ground.AddComponent<MeshCollider>();
			mc.sharedMesh = planeMesh;
			mc.convex = false;

			// a box ground too, for comparison
			var boxGround = new GameObject("box_ground");
			boxGround.transform.position = new Vector3(100f, -0.5f, 0f);
			var bgc = boxGround.AddComponent<BoxCollider>();
			bgc.size = new Vector3(20f, 1f, 20f);

			yield return null;

			// raycast down onto both grounds
			var hit1 = Physics.Raycast(new Vector3(0f, 5f, 0f), Vector3.down, out var h1, 100f);
			sb.AppendLine("[Probe] raycast mesh ground: hit=" + hit1 + (hit1 ? " pt=" + h1.point + " n=" + h1.normal + " col=" + h1.collider.name : ""));
			var hit2 = Physics.Raycast(new Vector3(100f, 5f, 0f), Vector3.down, out var h2, 100f);
			sb.AppendLine("[Probe] raycast box ground:  hit=" + hit2 + (hit2 ? " pt=" + h2.point + " n=" + h2.normal + " col=" + h2.collider.name : ""));
			// from underneath (backface)
			var hit3 = Physics.Raycast(new Vector3(0f, -5f, 0f), Vector3.up, out var h3, 100f);
			sb.AppendLine("[Probe] raycast mesh ground from below: hit=" + hit3);

			// props like ObjectSpawning.CreateUnitProps: inactive object, collider+rigidbody, set pose, activate
			var matPhysics = new PhysicsMaterial("prop") { dynamicFriction = 0.5f, staticFriction = 0.5f, bounciness = 0f };
			var box = new GameObject("BOX");
			box.SetActive(false);
			var col = box.AddComponent<BoxCollider>();
			col.size = Vector3.one;
			col.sharedMaterial = matPhysics;
			var rb = box.AddComponent<Rigidbody>();
			rb.mass = 1f;
			rb.linearDamping = 2f;
			rb.angularDamping = 2f;
			rb.sleepThreshold = 0.05f;
			rb.interpolation = RigidbodyInterpolation.Interpolate;
			box.transform.position = new Vector3(0f, 2f, 0f);
			box.SetActive(true);
			rb.mass = 2f;
			rb.ResetCenterOfMass();
			rb.ResetInertiaTensor();

			// same on the box ground
			var box2 = Instantiate(box, new Vector3(100f, 2f, 0f), Quaternion.identity);
			box2.name = "BOX2";

			var sphere = new GameObject("SPHERE");
			sphere.transform.position = new Vector3(2f, 2f, 0f);
			sphere.AddComponent<SphereCollider>().radius = 0.5f;
			sphere.AddComponent<Rigidbody>();

			var cyl = new GameObject("CONVEX");
			cyl.transform.position = new Vector3(-2f, 2f, 0f);
			var cm = cyl.AddComponent<MeshCollider>();
			var tmp2 = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
			cm.sharedMesh = tmp2.GetComponent<MeshFilter>().sharedMesh;
			Destroy(tmp2);
			cm.convex = true;
			cyl.AddComponent<Rigidbody>();

			// offset + rotated mesh ground (CLOiSim models are posed at their root before colliders are added)
			var ground2 = new GameObject("ground2");
			ground2.transform.SetPositionAndRotation(new Vector3(200f, 1f, 0f), Quaternion.Euler(0f, 30f, 0f));
			ground2.transform.localScale = new Vector3(10f, 1f, 10f);
			var mc2 = ground2.AddComponent<MeshCollider>();
			mc2.sharedMesh = planeMesh;
			var box3 = GameObject.CreatePrimitive(PrimitiveType.Cube);
			box3.name = "BOX3";
			box3.transform.position = new Vector3(201f, 4f, 1f);
			box3.AddComponent<Rigidbody>();
			yield return null;
			var hit5 = Physics.Raycast(new Vector3(201f, 10f, 1f), Vector3.down, out var h5, 100f);
			sb.AppendLine("[Probe] raycast offset mesh ground: hit=" + hit5 + (hit5 ? " pt=" + h5.point + " col=" + h5.collider.name : ""));

			// exact BackendProbe scene (known to work earlier)
			var pfloor = GameObject.CreatePrimitive(PrimitiveType.Cube);
			pfloor.name = "PFloor";
			pfloor.transform.position = new Vector3(0f, -0.5f, 50f);
			pfloor.transform.localScale = new Vector3(20f, 1f, 20f);
			var pbox = GameObject.CreatePrimitive(PrimitiveType.Cube);
			pbox.name = "PBox";
			pbox.transform.position = new Vector3(0f, 5f, 50f);
			pbox.AddComponent<Rigidbody>();

			yield return null;
			var hit4 = Physics.Raycast(new Vector3(0f, 10f, 50f), Vector3.down, out var h4, 100f);
			sb.AppendLine("[Probe] raycast primitive floor: hit=" + hit4 + (hit4 ? " pt=" + h4.point + " col=" + h4.collider.name : ""));
			sb.AppendLine("[Probe] gravity=" + Physics.gravity + " simMode=" + Physics.simulationMode + " fixedDt=" + Time.fixedDeltaTime + " timeScale=" + Time.timeScale);

			yield return new WaitForSeconds(3f);

			foreach (var o in new[] { box, box2, sphere, cyl, pbox, box3 })
				sb.AppendLine("[Probe] " + o.name + " y=" + o.transform.position.y.ToString("F3") + " pos=" + o.transform.position);
			sb.AppendLine("[Probe] DONE");
			Debug.Log(sb.ToString());

#if UNITY_EDITOR
			if (System.Environment.GetEnvironmentVariable("CLOISIM_PROBE_QUIT") == "1")
				UnityEditor.EditorApplication.Exit(0);
#endif
		}
	}
}
