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
	PxQuat swingQuat = s.isZero() ? PxQuat(0, 0, 0, 1) : PxQuat(s.magnitude(), s.getNormalized());
	return swingQuat * PxQuat(t, PxVec3(1, 0, 0));
}
PxQuat getQuat(PxVec3 v) {
	return getQuat(v[0], v[1], v[2]);
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
	"chest", "right_hip", "left_hip", "neck", "right_shoulder", "left_shoulder",
	"right_knee", "left_knee", "right_elbow", "left_elbow", "right_ankle", "left_ankle"
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
	assert(kps.size() == 24);
	for (int i = 0; i < 24; i++) {
		targetPositions.push_back(0);
		targetVelocities.push_back(0);
	}
}

PxQuat getPositionDifference(PxVec3 after, PxVec3 before) {
	return getQuat(after) * getQuat(before).getConjugate();
}

void printFar(PxReal *arr, int n) {
	printf("");
	for (int i = 0; i < n - 1; i++) {
		printf("%f; ", arr[i]);
	}
	printf("%f\n", arr[n - 1]);
}

void control(PxReal dt, int contactFlag) {
	gArticulation->copyInternalStateToCache(*gCache, PxArticulationCache::eALL);

	targetPositions = vector<float>(24, 0.f);
	targetVelocities = vector<float>(24, 0.f);

	simbicon_tick(dt, contactFlag);
	simbicon_setTargets();

	PxReal *positions = gCache->jointPosition;
	PxReal *velocities = gCache->jointVelocity;
	PxReal *forces = gCache->jointForce;

	memset(forces, 0, sizeof(PxReal) * gArticulation->getDofs());

/*	for (int i = 0; i < 24; i++) {
		forces[i] = kps[i] * (targetPositions[i] - positions[i]) + kds[i] * (targetVelocities[i] - velocities[i]);
		if (forces[i] > fls[i] ) {
		//	forces[i] = fls[i] ;
		}
	}*/

	for (auto &kvp : ar.jointMap) {
		auto &joint = kvp.second;

		int nDof = joint->nDof;
		int cacheIndex = joint->cacheIndex;

		if (nDof == 0) continue;

		if (nDof == 3) {
			PxVec3 position(
				positions[cacheIndex],
				positions[cacheIndex + 1], 
				positions[cacheIndex + 2]
			);
			PxVec3 velocity(
				velocities[cacheIndex],
				velocities[cacheIndex + 1],
				velocities[cacheIndex + 2]
			);
			PxVec3 targetPosition(
				targetPositions[cacheIndex],
				targetPositions[cacheIndex + 1],
				targetPositions[cacheIndex + 2]
			);
			PxVec3 targetVelocity(
				targetVelocities[cacheIndex],
				targetVelocities[cacheIndex + 1],
				targetVelocities[cacheIndex + 2]
			);
			PxVec3 kp(
				kps[cacheIndex],
				kps[cacheIndex + 1],
				kps[cacheIndex + 2]
			);
			PxVec3 kd(
				kds[cacheIndex],
				kds[cacheIndex + 1],
				kds[cacheIndex + 2]
			);

			PxQuat localRotation = getQuat(position);
			PxVec3 velocityInParentFrame = localRotation.rotate(velocity);

			PxQuat posDifference = getPositionDifference(targetPosition, position);
			PxVec3 axis;
			PxReal angle;
			posDifference.toRadiansAndUnitAxis(angle, axis);

			PxVec3 forceInParentFrame =
				PxMat33::createDiagonal(kp) * axis * angle -
				PxMat33::createDiagonal(kd) * velocityInParentFrame;
			PxVec3 forceInChildFrame = localRotation.getConjugate().rotate(forceInParentFrame);

			forces[cacheIndex] = forceInChildFrame[0];
			forces[cacheIndex + 1] = forceInChildFrame[1];
			forces[cacheIndex + 2] = forceInChildFrame[2];
		}
		else if (nDof == 1) {
			PxReal position = positions[cacheIndex];
			PxReal velocity = velocities[cacheIndex];
			PxReal targetPosition = targetPositions[cacheIndex];
			PxReal kp = kps[cacheIndex];
			PxReal kd = kds[cacheIndex];
			forces[cacheIndex] = kp * (targetPosition - position) - kd * velocity;
		}
		else {
			printf("no controller defined for Dof %d\n", nDof);
			assert(false);
		}
	}

	simbicon_updateForces();

	gArticulation->applyCache(*gCache, PxArticulationCache::eFORCE);

	PxU32 nDof = gArticulation->getDofs();

	printf("joint force:\n");
	printFar(forces, nDof);

	gArticulation->applyCache(*gCache, PxArticulationCache::eFORCE);

	printf("mass matrix:\n");
	PxArticulationCache* tmp = gArticulation->createCache();
	gArticulation->commonInit();
	gArticulation->computeGeneralizedMassMatrix(*tmp);

	for (PxU32 i = 0; i < nDof; i++) {
		for (PxU32 j = 0; j < nDof; j++) {
			printf("%f ", tmp->massMatrix[i * nDof + j]);
		}
		printf(";\n");
	}

	PxReal *acc = gCache->jointAcceleration;
	printf("joint acceleration:\n");
	printFar(acc, nDof);

	PxArticulationCache* tmp2 = gArticulation->createCache();
	gArticulation->copyInternalStateToCache(*tmp2, PxArticulationCache::eVELOCITY);
	gArticulation->computeCoriolisAndCentrifugalForce(*tmp2);
	printf("corr and centrifugal:\n");
	printFar(tmp2->jointForce, nDof);

	PxArticulationCache* tmp3 = gArticulation->createCache();
	gArticulation->computeGeneralizedGravityForce(*tmp3);
	printf("gravity:\n");
	printFar(tmp3->jointForce, nDof);

}
