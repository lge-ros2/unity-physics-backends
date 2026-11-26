/*
 * Copyright (c) 2026 LG Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using UnityEngine;

// Articulation layout of CLOiSim's CordZero_NR model, built with the same ArticulationBody property sequence as
// CLOiSim (Import.Link.CreateArticulationBody + Implement.Joint): revolute = SphericalJoint with locked swing axes,
// prismatic bumper with tiny limits, ball caster, fixed sensor links, suspension + wheel chain.
public static class NrLikeRobot
{
	// SDF (x forward, y left, z up) -> Unity (x right, y up, z forward)
	private static Vector3 ToUnity(float x, float y, float z) => new Vector3(-y, z, x);

	public static List<ArticulationBody> Build(Vector3 origin, float wheelTargetVelocityDeg)
	{
		var all = new List<ArticulationBody>();

		var baseLink = NewLink("NR_base_link", null, origin + ToUnity(0.03f, 0f, 0.06f), Quaternion.identity, all);
		AddBox(baseLink, new Vector3(0.28f, 0.08f, 0.34f) , Vector3.zero);
		baseLink.mass = 4f;
		baseLink.sleepThreshold = 1f;

		// bumper: prismatic along SDF x, limits [-0.008, 0], damping 20, infinite effort
		var bumper = NewLink("NR_bumper", baseLink, origin + ToUnity(0.15f, 0f, 0.06f), Quaternion.identity, all);
		AddBox(bumper, new Vector3(0.02f, 0.05f, 0.25f), Vector3.zero);
		bumper.mass = 0.05f;
		MakePrismatic(bumper, ToUnity(1f, 0f, 0f), -0.008f, 0f, 20f);

		// ball caster
		var caster = NewLink("NR_caster", baseLink, origin + ToUnity(0.13344f, 0f, 0.0073f), Quaternion.identity, all);
		var sc = caster.gameObject.AddComponent<SphereCollider>();
		sc.radius = 0.0073f;
		caster.mass = 0.01f;
		MakeBall(caster);

		foreach (var side in new[] { -1f, 1f })
		{
			var susp = NewLink("NR_susp" + side, baseLink, origin + ToUnity(0.049f, 0.115f * side, 0.033f), Quaternion.identity, all);
			AddBox(susp, new Vector3(0.05f, 0.04f, 0.03f), Vector3.zero);
			susp.mass = 0.3f;
			// SDF axis (0,1,0), limits [-0.2618, 0]
			MakeRevolute(susp, ToUnity(0f, 1f, 0f), true, SdfCurve(0f), SdfCurve(-0.2618f), 10f);

			var wheel = NewLink("NR_wheel" + side, susp, origin + ToUnity(-0.006f, 0.121f * side, 0.025f),
				Quaternion.Euler(0f, 0f, 90f), all);
			var cc = wheel.gameObject.AddComponent<CapsuleCollider>();
			cc.direction = 0;
			cc.radius = 0.0355f;
			cc.height = 0.03f;
			wheel.mass = 0.25f;
			// SDF axis (0,0,1) -> Unity up; free spin with velocity drive
			// real NR: wheel damping 0, forceLimit 15; Motor.CheckDriveType then switches the drive to Velocity
			MakeRevolute(wheel, ToUnity(0f, 0f, 1f), false, 0f, 0f, 0f);
			var d = wheel.yDrive;
			d.forceLimit = 15f;
			d.driveType = ArticulationDriveType.Velocity;
			d.targetVelocity = wheelTargetVelocityDeg;
			wheel.yDrive = d;
		}

		// fixed sensor links
		for (var i = 0; i < 4; i++)
		{
			var sensor = NewLink("NR_cliff" + i, baseLink, origin + ToUnity(0.153f, -0.0425f + 0.028f * i, 0.0206f), Quaternion.identity, all);
			sensor.mass = 0.01f;
			var box = sensor.gameObject.AddComponent<BoxCollider>();
			box.size = Vector3.one * 0.01f;
			MakeFixed(sensor);
		}

		SetInertia(baseLink, 0.02763698f, 0.03428207f, 0.05365596f);
		SetInertia(bumper, 0.0003430371f, 0.0001156282f, 0.0004268078f);
		SetInertia(caster, 0.000001296f, 0.000001296f, 0.000001296f);
		foreach (var ab in all)
		{
			if (ab.name.StartsWith("NR_susp"))
				SetInertia(ab, 0.0015f, 0.012f, 0.0015f);
			else if (ab.name.StartsWith("NR_wheel"))
				SetInertia(ab, 0.0002f, 0.0002f, 0.00037f);
		}

		return all;
	}

	private static float SdfCurve(float v) => -v * Mathf.Rad2Deg;

	private static ArticulationBody NewLink(string name, ArticulationBody parent, Vector3 pos, Quaternion rot,
		List<ArticulationBody> all)
	{
		var go = new GameObject(name);
		if (parent != null)
			go.transform.SetParent(parent.transform, false);
		go.transform.SetPositionAndRotation(pos, rot);

		var ab = go.AddComponent<ArticulationBody>();
		ab.linearVelocity = Vector3.zero;
		ab.angularVelocity = Vector3.zero;
		ab.useGravity = true;
		ab.jointType = ArticulationJointType.FixedJoint;
		ab.mass = 0.5f;
		ab.linearDamping = 3;
		ab.angularDamping = 1;
		ab.jointFriction = 0f;
		ab.solverIterations = 0;
		ab.solverVelocityIterations = 0;
		ab.sleepThreshold = 1f;
		ab.matchAnchors = true;
		ab.anchorPosition = Vector3.zero;
		ab.anchorRotation = Quaternion.identity;
		ab.ResetCenterOfMass();
		ab.ResetInertiaTensor();
		// CLOiSim keeps every link disabled until all poses are applied (Import.SpecifyPose re-enables them)
		ab.enabled = false;
		all.Add(ab);
		return ab;
	}

	// CLOiSim sets the SDF inertia explicitly (SDF ixx,iyy,izz -> Unity iyy,izz,ixx)
	private static void SetInertia(ArticulationBody ab, float ixx, float iyy, float izz)
	{
		ab.automaticCenterOfMass = true;
		ab.inertiaTensor = new Vector3(iyy, izz, ixx);
		ab.inertiaTensorRotation = Quaternion.identity;
		ab.automaticInertiaTensor = false;
	}

	public static void EnableAll(List<ArticulationBody> links)
	{
		foreach (var ab in links)
			ab.enabled = true;
	}

	private static void AddBox(ArticulationBody ab, Vector3 size, Vector3 center)
	{
		var box = ab.gameObject.AddComponent<BoxCollider>();
		box.size = size;
		box.center = center;
	}

	private static void ApplyDominantAxis(ArticulationBody ab, Vector3 axis, out int dominant)
	{
		var ax = Mathf.Abs(axis.x);
		var ay = Mathf.Abs(axis.y);
		var az = Mathf.Abs(axis.z);
		if (ax >= ay && ax >= az)
		{
			ab.anchorRotation *= Quaternion.FromToRotation(Vector3.right, axis);
			dominant = 0;
		}
		else if (ay >= ax && ay >= az)
		{
			ab.anchorRotation *= Quaternion.FromToRotation(Vector3.up, axis);
			dominant = 1;
		}
		else
		{
			ab.anchorRotation *= Quaternion.FromToRotation(Vector3.forward, axis);
			dominant = 2;
		}
	}

	private static void MakeFixed(ArticulationBody ab)
	{
		ab.jointType = ArticulationJointType.FixedJoint;
		ab.linearDamping = 2;
		ab.angularDamping = 2;
		ab.jointFriction = 0;
		ab.solverIterations = 0;
		ab.solverVelocityIterations = 0;
	}

	private static void MakeBall(ArticulationBody ab)
	{
		ab.jointType = ArticulationJointType.SphericalJoint;
		ab.linearDamping = 3;
		ab.angularDamping = 1;
		ab.swingYLock = ArticulationDofLock.FreeMotion;
		ab.swingZLock = ArticulationDofLock.FreeMotion;
		ab.twistLock = ArticulationDofLock.FreeMotion;
		ab.jointFriction = 0f;
	}

	private static void MakeRevolute(ArticulationBody ab, Vector3 axis, bool limited, float lowerDeg, float upperDeg, float damping)
	{
		ab.jointType = ArticulationJointType.SphericalJoint;
		ab.linearDamping = 1.5f;
		ab.angularDamping = 2;

		var drive = new ArticulationDrive();
		if (limited)
		{
			drive.lowerLimit = Mathf.Min(lowerDeg, upperDeg);
			drive.upperLimit = Mathf.Max(lowerDeg, upperDeg);
		}
		drive.forceLimit = float.MaxValue;
		drive.stiffness = 0f;
		drive.damping = damping;
		ab.jointFriction = 0f;
		ab.maxJointVelocity = float.MaxValue;

		ApplyDominantAxis(ab, axis.normalized, out var dominant);
		var lockMode = limited ? ArticulationDofLock.LimitedMotion : ArticulationDofLock.FreeMotion;
		switch (dominant)
		{
			case 0:
				ab.xDrive = drive;
				ab.twistLock = lockMode;
				ab.swingYLock = ArticulationDofLock.LockedMotion;
				ab.swingZLock = ArticulationDofLock.LockedMotion;
				break;
			case 1:
				ab.yDrive = drive;
				ab.twistLock = ArticulationDofLock.LockedMotion;
				ab.swingYLock = lockMode;
				ab.swingZLock = ArticulationDofLock.LockedMotion;
				break;
			default:
				ab.zDrive = drive;
				ab.twistLock = ArticulationDofLock.LockedMotion;
				ab.swingYLock = ArticulationDofLock.LockedMotion;
				ab.swingZLock = lockMode;
				break;
		}
	}

	private static void MakePrismatic(ArticulationBody ab, Vector3 axis, float lower, float upper, float damping)
	{
		ab.jointType = ArticulationJointType.PrismaticJoint;
		ab.linearDamping = 1.5f;
		ab.angularDamping = 1;

		var drive = new ArticulationDrive
		{
			lowerLimit = lower,
			upperLimit = upper,
			forceLimit = float.MaxValue,
			stiffness = 0f,
			damping = damping,
		};
		ab.jointFriction = 0f;
		ab.maxJointVelocity = float.MaxValue;

		ApplyDominantAxis(ab, axis.normalized, out var dominant);
		ab.linearLockX = dominant == 0 ? ArticulationDofLock.LimitedMotion : ArticulationDofLock.LockedMotion;
		ab.linearLockY = dominant == 1 ? ArticulationDofLock.LimitedMotion : ArticulationDofLock.LockedMotion;
		ab.linearLockZ = dominant == 2 ? ArticulationDofLock.LimitedMotion : ArticulationDofLock.LockedMotion;
		switch (dominant)
		{
			case 0: ab.xDrive = drive; break;
			case 1: ab.yDrive = drive; break;
			default: ab.zDrive = drive; break;
		}
	}
}
