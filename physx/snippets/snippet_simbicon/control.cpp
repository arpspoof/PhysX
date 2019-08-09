#include "globals.h"
#include "euler.h"
#include "config.h"
#include "articulationTree.h"
#include "simbicon.h"

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
right_shoulder	12, 13, 14
left_shoulder	15, 16, 17	
right_knee		18	
left_knee		19
right_elbow		20
left_elbow		21
right_ankle		22
left_ankle		23

*/

// jointPosition is twist + swing exp map [qSwing * qTwist]

PxQuat getQuat(PxReal t, PxReal s1, PxReal s2) {
	PxVec3 s(0, s1, s2);
	return PxQuat(s.magnitude(), s.getNormalized()) * PxQuat(t, PxVec3(1, 0, 0));
}
PxQuat getQuat(PxVec3 v) {
	return getQuat(v[0], v[1], v[2]);
}

PxQuat getRelQuat(PxQuat from, PxQuat to) {
	return to * from.getConjugate();
}

PxVec3 getPos(PxQuat q) {
	PxQuat qT = PxQuat(q.x, 0, 0, q.w).getNormalized();
	PxQuat qS = q * qT.getConjugate();
	PxVec3 twist = PxVec3(1, 0, 0) * 2 * (PxReal)atan2(qT.x, qT.w);
	PxVec3 swingImg(qS.x, qS.y, qS.z);
	PxVec3 swing = swingImg / swingImg.magnitude() * 2 * (PxReal)atan2(swingImg.magnitude(), qS.w);
	return twist + swing;
}

std::vector<string> nameList{
	"chest",
};

vector<float> kps, kds, fls;
vector<float> targetPositions, targetVelocities;

void initControl() {
	for (string &name : nameList) {
		PxU32 nDof = ar.linkMap[name]->link->getInboundJointDof();
		PxReal kp = getConfigF("P_KP_" + name);
		PxReal kd = getConfigF("P_KD_" + name);
		PxReal fl = getConfigF("P_FL_" + name);
		for (PxU32 i = 0; i < nDof; i++) {
			kps.push_back(kp);
			kds.push_back(kd);
			fls.push_back(fl);
		}
	}
//	assert(kps.size() == 24);
	for (int i = 0; i < 24; i++) {
		targetPositions.push_back(0);
		targetVelocities.push_back(0);
	}
}

int counter = 0;

void control(PxReal /*dt*/, int /*contactFlag*/) {
	gArticulation->copyInternalStateToCache(*gCache, PxArticulationCache::eALL);

	targetPositions = vector<float>(24, 0.f);
	targetVelocities = vector<float>(24, 0.f);

//	simbicon_tick(dt, contactFlag);
//	simbicon_setTargets();

/*	PxReal *positions = gCache->jointPosition;
	PxReal *velocities = gCache->jointVelocity;
	PxReal *forces = gCache->jointForce;

	for (int i = 0; i < 24; i++) {
		forces[i] = kps[i] * (targetPositions[i] - positions[i]) + kds[i] * (targetVelocities[i] - velocities[i]);
		if (forces[i] > fls[i] ) {
		//	forces[i] = fls[i] ;
		}
	}

	simbicon_updateForces();*/

	counter++;

	float t = getConfigF("T_t"), a = getConfigF("T_a"), b = getConfigF("T_b");

	PxReal *forces = gCache->jointForce;
	PxVec3 testF(t, a, b);
	testF.normalize();

	PxVec3 v = ar.linkMap["chest"]->link->getAngularVelocity();
	PxQuat qNeck = ar.linkMap["chest"]->link->getGlobalPose().q;
	PxQuat qChest = ar.linkMap["root"]->link->getGlobalPose().q;
	PxQuat qLocal = qChest.getConjugate() * qNeck;

	testF = qLocal.getConjugate().rotate(testF);

	if (1 || counter <= 500) {
		forces[0] = testF.x * 4;
		forces[1] = testF.y * 4;
		forces[2] = testF.z * 4;
	}
	else {
		forces[0] = 0;
		forces[1] = 0;
		forces[2] = 0;
	}

	PxReal *velocities = gCache->jointVelocity;
	PxReal *acc = gCache->jointAcceleration;


	PxReal *p = gCache->jointPosition;

	PxVec3 vlo(velocities[0], velocities[1], velocities[2]);

	vlo = qLocal.rotate(vlo);

	printf("a0 = %f,%f,%f; v1 = %f,%f,%f; p = %f,%f,%f\n", 
		acc[0], acc[1], acc[2], vlo[0], vlo[1], vlo[2], p[0], p[1], p[2]);

	//v = qNeck.getConjugate().rotate(v);
	//printf("v = %f, %f, %f; q = %f, %f, %f, %f\n", v[0], v[1], v[2], qLocal.w, qLocal.x, qLocal.y, qLocal.z);

	gArticulation->applyCache(*gCache, PxArticulationCache::eFORCE);
}
