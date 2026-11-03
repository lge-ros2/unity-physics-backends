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

			// robot-like articulation: free root + two driven revolute wheels, resting on PFloor
			var robot = GameObject.CreatePrimitive(PrimitiveType.Cube);
			robot.name = "ROBOT";
			robot.transform.position = new Vector3(5f, 1f, -20f);
			robot.transform.localScale = new Vector3(0.5f, 0.2f, 0.4f);
			var rootAb = robot.AddComponent<ArticulationBody>();
			rootAb.mass = 5f;
			// same property sequence as CLOiSim's Import.Link.CreateArticulationBody
			rootAb.linearVelocity = Vector3.zero;
			rootAb.angularVelocity = Vector3.zero;
			rootAb.useGravity = false;
			rootAb.jointType = ArticulationJointType.FixedJoint;
			rootAb.linearDamping = 3;
			rootAb.angularDamping = 1;
			rootAb.jointFriction = 0f;
			rootAb.solverIterations = 0;
			rootAb.solverVelocityIterations = 0;
			rootAb.sleepThreshold = 1f;
			rootAb.Sleep();
			rootAb.matchAnchors = true;
			rootAb.anchorPosition = Vector3.zero;
			rootAb.anchorRotation = Quaternion.identity;
			rootAb.ResetCenterOfMass();
			rootAb.ResetInertiaTensor();
			rootAb.useGravity = true;
			var wheels = new ArticulationBody[2];
			for (var i = 0; i < 2; i++)
			{
				var w = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
				w.name = "WHEEL" + i;
				w.transform.SetParent(robot.transform, false);
				w.transform.position = new Vector3(5f, 0.8f, -20f + (i == 0 ? 0.25f : -0.25f));
				w.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
				w.transform.localScale = new Vector3(0.2f, 0.02f, 0.2f);
				var ab = w.AddComponent<ArticulationBody>();
				ab.mass = 0.5f;
				ab.jointType = ArticulationJointType.RevoluteJoint;
				ab.anchorRotation = Quaternion.identity;
				ab.twistLock = ArticulationDofLock.FreeMotion;
				var d = ab.xDrive;
				d.stiffness = 0f;
				d.damping = 100f;
				d.forceLimit = 1000f;
				d.targetVelocity = 360f;
				ab.xDrive = d;
				wheels[i] = ab;
			}

			// idle robot (wheels held at target velocity 0) with a fixed-joint caster: measures resting jitter
			var idle = GameObject.CreatePrimitive(PrimitiveType.Cube);
			idle.name = "IDLE";
			idle.transform.position = new Vector3(-5f, 0.6f, -20f);
			idle.transform.localScale = new Vector3(0.5f, 0.2f, 0.4f);
			var idleAb = idle.AddComponent<ArticulationBody>();
			idleAb.mass = 5f;
			idleAb.linearDamping = 3;
			idleAb.angularDamping = 1;
			idleAb.solverIterations = 0;
			idleAb.solverVelocityIterations = 0;
			idleAb.sleepThreshold = 0f;
			for (var i = 0; i < 2; i++)
			{
				var w = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
				w.name = "IWHEEL" + i;
				w.transform.SetParent(idle.transform, false);
				w.transform.position = new Vector3(-5f, 0.25f, -20f + (i == 0 ? 0.25f : -0.25f));
				w.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
				w.transform.localScale = new Vector3(0.3f, 0.02f, 0.3f);
				var ab = w.AddComponent<ArticulationBody>();
				ab.mass = 0.5f;
				ab.solverIterations = 0;
				ab.solverVelocityIterations = 0;
				ab.jointType = ArticulationJointType.RevoluteJoint;
				ab.anchorRotation = Quaternion.identity;
				ab.twistLock = ArticulationDofLock.FreeMotion;
				var d = ab.xDrive;
				d.stiffness = 0f;
				d.damping = 100f;
				d.forceLimit = 1000f;
				d.targetVelocity = 0f;
				ab.xDrive = d;
			}
			var caster = GameObject.CreatePrimitive(PrimitiveType.Sphere);
			caster.name = "CASTER";
			caster.transform.SetParent(idle.transform, false);
			caster.transform.position = new Vector3(-5.3f, 0.05f, -20f);
			caster.transform.localScale = Vector3.one * 0.1f;
			var casterAb = caster.AddComponent<ArticulationBody>();
			casterAb.mass = 0.1f;
			casterAb.jointType = ArticulationJointType.FixedJoint;

			// links with overlapping colliders: root + child + grandchild (non-adjacent to root); CLOiSim ignores such pairs
			ArticulationBody BuildOverlap(string name, float x, bool ignore)
			{
				var rootGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
				rootGo.name = name;
				rootGo.transform.position = new Vector3(x, 0.5f, 50f);
				rootGo.transform.localScale = new Vector3(0.5f, 0.2f, 0.4f);
				var rootBody = rootGo.AddComponent<ArticulationBody>();
				rootBody.mass = 5f;
				rootBody.sleepThreshold = 0f;
				rootBody.solverIterations = 0;
				rootBody.solverVelocityIterations = 0;
				var parent = rootGo;
				var cols = new System.Collections.Generic.List<Collider> { rootGo.GetComponent<Collider>() };
				for (var i = 1; i <= 2; i++)
				{
					var c = GameObject.CreatePrimitive(PrimitiveType.Cube);
					c.name = name + "_L" + i;
					c.transform.SetParent(parent.transform, false);
					c.transform.position = new Vector3(x + 0.1f * i, 0.5f, 50f);
					c.transform.localScale = new Vector3(0.3f, 0.2f, 0.3f);
					var cb = c.AddComponent<ArticulationBody>();
					cb.mass = 0.5f;
					cb.jointType = ArticulationJointType.FixedJoint;
					cb.solverIterations = 0;
					cb.solverVelocityIterations = 0;
					cols.Add(c.GetComponent<Collider>());
					parent = c;
				}
				if (ignore)
				{
					for (var i = 0; i < cols.Count; i++)
						for (var j = i + 1; j < cols.Count; j++)
							Physics.IgnoreCollision(cols[i], cols[j]);
				}
				return rootBody;
			}
			var ovl = BuildOverlap("OVL", -8f, false);
			var ovlIgnored = BuildOverlap("OVLI", 8f, true);

			yield return null;
			var hit4 = Physics.Raycast(new Vector3(0f, 10f, 50f), Vector3.down, out var h4, 100f);
			sb.AppendLine("[Probe] raycast primitive floor: hit=" + hit4 + (hit4 ? " pt=" + h4.point + " col=" + h4.collider.name : ""));
			sb.AppendLine("[Probe] gravity=" + Physics.gravity + " simMode=" + Physics.simulationMode + " fixedDt=" + Time.fixedDeltaTime + " timeScale=" + Time.timeScale);

			yield return new WaitForSeconds(3f);

			// jitter metric: peak-to-peak position / peak speeds of the idle robot over 2 s
			var tracked = new[] { idleAb, ovl, ovlIgnored };
			var minP = new Vector3[tracked.Length];
			var maxP = new Vector3[tracked.Length];
			var maxLin = new float[tracked.Length];
			var maxAng = new float[tracked.Length];
			for (var k = 0; k < tracked.Length; k++)
				minP[k] = maxP[k] = tracked[k].transform.position;
			for (var t = 0f; t < 2f; t += Time.fixedDeltaTime)
			{
				yield return new WaitForFixedUpdate();
				for (var k = 0; k < tracked.Length; k++)
				{
					var p = tracked[k].transform.position;
					minP[k] = Vector3.Min(minP[k], p);
					maxP[k] = Vector3.Max(maxP[k], p);
					maxLin[k] = Mathf.Max(maxLin[k], tracked[k].linearVelocity.magnitude);
					maxAng[k] = Mathf.Max(maxAng[k], tracked[k].angularVelocity.magnitude);
				}
			}
			for (var k = 0; k < tracked.Length; k++)
				sb.AppendLine("[Probe] " + tracked[k].name + " pos=" + tracked[k].transform.position + " p2p=" + (maxP[k] - minP[k]).ToString("F5")
					+ " maxLin=" + maxLin[k].ToString("F4") + " maxAng=" + maxAng[k].ToString("F4")
					+ " iters=" + tracked[k].solverIterations + "/" + tracked[k].solverVelocityIterations);

			foreach (var o in new[] { box, box2, sphere, cyl, pbox, box3 })
				sb.AppendLine("[Probe] " + o.name + " y=" + o.transform.position.y.ToString("F3") + " pos=" + o.transform.position);
			sb.AppendLine("[Probe] ROBOT y=" + robot.transform.position.y.ToString("F3") + " pos=" + robot.transform.position);
			foreach (var wb in wheels)
				sb.AppendLine("[Probe] " + wb.name + " jointPos=" + (wb.dofCount > 0 ? wb.jointPosition[0].ToString("F2") : "n/a") + " jointVel=" + (wb.dofCount > 0 ? wb.jointVelocity[0].ToString("F2") : "n/a") + " pos=" + wb.transform.position);
			sb.AppendLine("[Probe] DONE");
			Debug.Log(sb.ToString());

#if UNITY_EDITOR
			if (System.Environment.GetEnvironmentVariable("CLOISIM_PROBE_QUIT") == "1")
				UnityEditor.EditorApplication.Exit(0);
#endif
		}
	}
}
