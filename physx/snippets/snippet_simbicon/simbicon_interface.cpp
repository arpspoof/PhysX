#include "simbicon_interface.h"
#include "articulationTree.h"
#include "globals.h"

#include <vector>

using namespace std;

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

extern vector<float> targetPositions;
extern Articulation ar;

void setLHipTarget(vec3 pos) {
	targetPositions[6] = pos[0];
	targetPositions[7] = pos[1];
	targetPositions[8] = pos[2];
}
void setRHipTarget(vec3 pos) {
	targetPositions[3] = pos[0];
	targetPositions[4] = pos[1];
	targetPositions[5] = pos[2];
}

void setLKneeTarget(float pos) {
	targetPositions[19] = pos;
}
void setRKneeTarget(float pos) {
	targetPositions[18] = pos;
}

void setLAnkleTarget(float pos) {
	targetPositions[23] = pos;
}
void setRAnkleTarget(float pos) {
	targetPositions[22] = pos;
}

void setChestTarget(vec3 pos) {
	targetPositions[0] = pos[0];
	targetPositions[1] = pos[1];
	targetPositions[2] = pos[2];
}

vec3 getCOMGlobalPos() {
	auto link = ar.linkMap["root"]->link;
	return link->getGlobalPose().transform(link->getCMassLocalPose().p);
}
vec3 getLFootCOMGlobalPos() {
	auto link = ar.linkMap["left_ankle"]->link;
	return link->getGlobalPose().transform(link->getCMassLocalPose().p);
}
vec3 getRFootCOMGlobalPos() {
	auto link = ar.linkMap["right_ankle"]->link;
	return link->getGlobalPose().transform(link->getCMassLocalPose().p);
}

// Note: method to compute root force can be different
quat getBaseOri() {
	return ar.linkMap["base"]->link->getGlobalPose().q;
}
vec3 getBaseAngularVelocity() {
	return ar.linkMap["base"]->link->getAngularVelocity();
}

vec3 getLinearVelocity() {
	return ar.linkMap["base"]->link->getLinearVelocity();
}

vec3 getChestForce() {
    return vec3(0, 0, 0);
	//PxReal *forces = gCache->jointForce;
	//return vec3(forces[0], forces[1], forces[2]);
}
vec3 getLHipForce() {
    return vec3(0, 0, 0);
	//PxReal *forces = gCache->jointForce;
	//return vec3(forces[6], forces[7], forces[8]);
}
vec3 getRHipForce() {
    return vec3(0, 0, 0);
	//PxReal *forces = gCache->jointForce;
	//return vec3(forces[3], forces[4], forces[5]);
}

static PxQuat getQuat(PxReal t, PxReal s1, PxReal s2) {
	PxVec3 s(0, s1, s2);
	PxQuat swingQuat = s.isZero() ? PxQuat(0, 0, 0, 1) : PxQuat(s.magnitude(), s.getNormalized());
	return swingQuat * PxQuat(t, PxVec3(1, 0, 0));
}

quat getChestOriLocal() {
    return PxQuat(0, 0, 0, 1);
	//PxReal *positions = gCache->jointPosition;
	//return getQuat(positions[0], positions[1], positions[2]);
}

quat getLHipOriLocal() {
    return PxQuat(0, 0, 0, 1);
	//PxReal *positions = gCache->jointPosition;
	//return getQuat(positions[6], positions[7], positions[8]);
}

quat getRHipOriLocal() {
    return PxQuat(0, 0, 0, 1);
	//PxReal *positions = gCache->jointPosition;
	//return getQuat(positions[3], positions[4], positions[5]);
}

void setLHipForce(vec3 f) {
    return;
	////PxReal *forces = gCache->jointForce;
	//forces[6] = f[0];
	//forces[7] = f[1];
	//forces[8] = f[2];
}
void setRHipForce(vec3 f) {
    return;
	//PxReal *forces = gCache->jointForce;
	//forces[3] = f[0];
	//forces[4] = f[1];
	//forces[5] = f[2];
}

quat getQuat(float angle, vec3 axis) {
	return quat(angle, axis);
}
