#include "globals.h"
#include "euler.h"
#include "config.h"
#include "articulationTree.h"
#include "simbicon.h"

#include <../../../eigen/Eigen/Dense>

using namespace Eigen;

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

PxArticulationCache *tmpcache, *tmpcache2, *tmpcache3, *tmpcache4;

void initControl() {
	PxU32 totalDof = gArticulation->getDofs();

	kps = vector<float>(totalDof);
	kds = vector<float>(totalDof);
	fls = vector<float>(totalDof);
	targetPositions = vector<float>(totalDof, 0);
	targetVelocities = vector<float>(totalDof, 0);

	for (auto &kvp : ar.jointMap) {
		auto &joint = kvp.second;
		const string &name = kvp.first;

		int nDof = joint->nDof;
		int cacheIndex = joint->cacheIndex;

		for (int i = 0; i < nDof; i++) {
			kps[cacheIndex + i] = getConfigF("P_KP_" + name);
			kds[cacheIndex + i] = getConfigF("P_KD_" + name);
			fls[cacheIndex + i] = getConfigF("P_FL_" + name);
		}
	}

	tmpcache = gArticulation->createCache();
	tmpcache2 = gArticulation->createCache();
	tmpcache3 = gArticulation->createCache();
	tmpcache4 = gArticulation->createCache();
}

PxQuat getPositionDifference(PxVec3 after, PxVec3 before) {
	return getQuat(after) * getQuat(before).getConjugate();
}

void printFar(PxReal *arr, int n) {
	printf("[ ");
	for (int i = 0; i < n - 1; i++) {
		printf("%f, ", arr[i]);
	}
	printf("%f ]\n", arr[n - 1]);
}

void printFar(double *arr, int n) {
	printf("[ ");
	for (int i = 0; i < n - 1; i++) {
		printf("%f, ", arr[i]);
	}
	printf("%f ]\n", arr[n - 1]);
}

VectorXd calcAcc(24);

void control(PxReal dt, int /*contactFlag*/) {
	gArticulation->copyInternalStateToCache(*gCache, PxArticulationCache::eALL);

	PxVec3 extforceaddneck = PxVec3(70, 0, 0);
	PxVec3 extforceaddRHip = PxVec3(70, 0, 60);
	ar.linkMap["neck"]->link->addForce(extforceaddneck);
	ar.linkMap["right_hip"]->link->addForce(extforceaddRHip);

	//////////////////////////////////////
	PxU32 nnDof = gArticulation->getDofs();

	gArticulation->commonInit();
	gArticulation->computeGeneralizedMassMatrix(*tmpcache);

	gArticulation->copyInternalStateToCache(*tmpcache2, PxArticulationCache::eVELOCITY);
	gArticulation->computeCoriolisAndCentrifugalForce(*tmpcache2);

	gArticulation->computeGeneralizedGravityForce(*tmpcache3);

	tmpcache4->externalForces[ar.linkMap["neck"]->link->getLinkIndex()].force = extforceaddneck;
	tmpcache4->externalForces[ar.linkMap["right_hip"]->link->getLinkIndex()].force = extforceaddRHip;
	gArticulation->computeGeneralizedExternalForce(*tmpcache4);

	VectorXd centrifugalCoriolisGravityExternal(nnDof);
	for (PxU32 i = 0; i < nnDof; i++) {
		centrifugalCoriolisGravityExternal(i) = -tmpcache2->jointForce[i] - tmpcache3->jointForce[i] - tmpcache4->jointForce[i];
	}
	MatrixXd H(nnDof, nnDof);
	for (PxU32 i = 0; i < nnDof; i++) {
		for (PxU32 j = i; j < nnDof; j++) {
			H(i, j) = H(j, i) = tmpcache->massMatrix[i * nnDof + j];
		}
	}

	/////////////////////////////////////////

	targetPositions = vector<float>(24, 0.f);
	targetVelocities = vector<float>(24, 0.f);

//	simbicon_tick(dt, contactFlag);
//	simbicon_setTargets();

	PxReal *positions = gCache->jointPosition;
	PxReal *velocities = gCache->jointVelocity;
	PxReal *forces = gCache->jointForce;

	memset(forces, 0, sizeof(PxReal) * gArticulation->getDofs());

	VectorXd proportionalTorquePlusQDotDeltaT(nnDof);
	VectorXd derivativeTorque(nnDof);

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
			PxVec3 targetPosition(
				targetPositions[cacheIndex],
				targetPositions[cacheIndex + 1],
				targetPositions[cacheIndex + 2]
			);
			PxVec3 kp(
				kps[cacheIndex],
				kps[cacheIndex + 1],
				kps[cacheIndex + 2]
			);

			PxQuat localRotation = getQuat(position);

			PxQuat posDifference = getPositionDifference(targetPosition, position);
			PxVec3 axis;
			PxReal angle;
			posDifference.toRadiansAndUnitAxis(angle, axis);

			PxVec3 proportionalForceInParentFrame = PxMat33::createDiagonal(kp) * axis * angle;
			PxVec3 proportionalForceInChildFrame = localRotation.getConjugate().rotate(proportionalForceInParentFrame);

			proportionalTorquePlusQDotDeltaT(cacheIndex) = proportionalForceInChildFrame[0] - 
				dt * velocities[cacheIndex] * kps[cacheIndex];
			proportionalTorquePlusQDotDeltaT(cacheIndex + 1) = proportionalForceInChildFrame[1] -
				dt * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
			proportionalTorquePlusQDotDeltaT(cacheIndex + 2) = proportionalForceInChildFrame[2] -
				dt * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

			derivativeTorque(cacheIndex) = -kds[cacheIndex] * velocities[cacheIndex];
			derivativeTorque(cacheIndex + 1) = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
			derivativeTorque(cacheIndex + 2) = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];

			H(cacheIndex, cacheIndex) += kds[cacheIndex] * dt;
			H(cacheIndex + 1, cacheIndex + 1) += kds[cacheIndex + 1] * dt;
			H(cacheIndex + 2, cacheIndex + 2) += kds[cacheIndex + 2] * dt;
		}
		else if (nDof == 1) {
			PxReal position = positions[cacheIndex];
			PxReal velocity = velocities[cacheIndex];
			PxReal targetPosition = targetPositions[cacheIndex];
			PxReal kp = kps[cacheIndex];
			proportionalTorquePlusQDotDeltaT(cacheIndex) = kp * (targetPosition - position - dt * velocity);
			derivativeTorque(cacheIndex) = -kds[cacheIndex] * velocity;
			H(cacheIndex, cacheIndex) += kds[cacheIndex] * dt;
		}
		else {
			printf("no controller defined for Dof %d\n", nDof);
			assert(false);
		}
	}

	VectorXd qDotDot = H.llt().solve(centrifugalCoriolisGravityExternal + proportionalTorquePlusQDotDeltaT + derivativeTorque);
	for (PxU32 i = 0; i < nnDof; i++) {
		forces[i] = (PxReal)(proportionalTorquePlusQDotDeltaT(i) + derivativeTorque(i) - dt * kds[i] * qDotDot(i));
	}

	// PD
/*	for (PxU32 i = 0; i < nnDof; i++) {
		forces[i] = (PxReal)(proportionalTorquePlusQDotDeltaT(i) + derivativeTorque(i) + dt * kps[i] * velocities[i]);
	}*/

	gArticulation->applyCache(*gCache, PxArticulationCache::eFORCE);
}
