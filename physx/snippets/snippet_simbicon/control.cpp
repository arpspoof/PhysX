#include "globals.h"
#include "euler.h"
#include "articulationTree.h"

PxReal twistTarget, swing1Target, swing2Target;

extern Articulation ar;

void driveSpherical(Joint *j) {
	j->joint->setDriveTarget(PxArticulationAxis::eTWIST, twistTarget);
	j->joint->setDriveTarget(PxArticulationAxis::eSWING1, swing1Target);
	j->joint->setDriveTarget(PxArticulationAxis::eSWING2, swing2Target);
}

void driveRevolute(Joint *j) {
	auto rev = (RevoluteJoint*)j;
	PxReal target = 0;
	switch (rev->axis) {
	case PxArticulationAxis::eTWIST:
		target = twistTarget; break;
	case PxArticulationAxis::eSWING1:
		target = swing1Target; break;
	case PxArticulationAxis::eSWING2:
		target = swing2Target; break;
	}
	rev->joint->setDriveTarget(rev->axis, target);
}

/*

chest			0, 1, 2
right_hip		3, 4, 5
left_hip		6, 7, 8
neck			9, 10, 11

*/

void control5(PxReal /*dt*/) {
	gArticulation->copyInternalStateToCache(*gCache, PxArticulationCache::eALL);
	PxReal *vs = gCache->jointVelocity;
	PxVec3 redu(vs[9], vs[10], vs[11]);
	redu.normalize();
	printf("reduc: %f %f %f\n", redu[0], redu[1], redu[2]);
/*	for (int i = 0; i < 100; i++) {
		cout << maxim[i] << " ";
	}
	cout << endl;*/
//	gArticulation->applyCache(*gCache, PxArticulationCache::eALL);
//	PxVec3 prevF(forces[9], forces[10], forces[11]);
	twistTarget = 1.0f;
	swing1Target = 1.f;
	swing2Target = 2.0f;
	driveSpherical(ar.jointMap["neck"]);
	PxQuat tq(1.f, PxVec3(1, 0, 0));
	PxVec3 sw(0, 1, 2);
	PxQuat ts(sw.magnitude(), sw.getNormalized());
	PxQuat hy = ts * tq;
	PxVec3 img(hy.x, hy.y, hy.z);
	img.normalize();
	printf("axis : %f %f %f\n", img[0], img[1], img[2]);
/*	gArticulation->copyInternalStateToCache(*gCache, PxArticulationCache::eALL);
	forces = gCache->jointForce;
	PxVec3 postF(forces[9], forces[10], forces[11]);
	printf("prev: %f, %f, %f\npost: %f, %f, %f\n",
		prevF[0], prevF[1], prevF[2],
		postF[0], postF[1], postF[2]);*/
//	driveRevolute(ar.jointMap["right_ankle"]);
}

void control(PxReal /*dt*/) {
/*	twistTarget = 1.0f;
	swing1Target = 1.f;
	swing2Target = 2.0f;*/
	//driveSpherical(ar.jointMap["neck"]);
	/*	gArticulation->copyInternalStateToCache(*gCache, PxArticulationCache::eALL);
	forces = gCache->jointForce;
	PxVec3 postF(forces[9], forces[10], forces[11]);
	printf("prev: %f, %f, %f\npost: %f, %f, %f\n",
	prevF[0], prevF[1], prevF[2],
	postF[0], postF[1], postF[2]);*/
	//	driveRevolute(ar.jointMap["right_ankle"]);
}

static bool gClosing = true;

void control2(PxReal dt) {
	PxReal driveValue = gDriveJoint->getDriveTarget(PxArticulationAxis::eZ);

	if (gClosing && driveValue < -1.2f)
		gClosing = false;
	else if (!gClosing && driveValue > 0.f)
		gClosing = true;

	if (gClosing)
		driveValue -= dt * 0.25f;
	else
		driveValue += dt * 0.25f;

	gDriveJoint->setDriveTarget(PxArticulationAxis::eZ, driveValue);
}
