#include "robot.h"
#include "config.h"
#include "globals.h"

#include <iostream>

using namespace physx;
using namespace std;

PxArticulationLink *base, *root;
PxArticulationLink *chest, *test;

PxArticulationJointReducedCoordinate *jRootChest;

static void genSphereLink(PxArticulationLink *&link, PxArticulationLink *parent, PxTransform transform,
	PxReal r, PxReal m) {
	link = gArticulation->createLink(parent, transform);
	PxRigidActorExt::createExclusiveShape(*link, /*PxSphereGeometry(r)*/PxBoxGeometry(r, r ,r), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*link, m / (4.0f / 3.0f*PxPi*r*r*r));
}

/*static void genCapsuleLink(PxArticulationLink *&link, PxArticulationLink *parent, PxTransform transform,
	PxReal r, PxReal l, PxReal m) {
	link = gArticulation->createLink(parent, transform);
	PxRigidActorExt::createExclusiveShape(*link, PxCapsuleGeometry(r, l / 2), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*link, m / (4.0f / 3.0f*PxPi*r*r*r + 4 * PxPi*r*r*l));
}*/

static void genSphericalJoint(PxArticulationJointReducedCoordinate *&joint, PxArticulationLink *link,
	PxTransform parentPose, PxTransform childPose) {
	joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
	joint->setJointType(PxArticulationJointType::eSPHERICAL);
	joint->setParentPose(parentPose);
	joint->setChildPose(childPose);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
	joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
	joint->setLimit(PxArticulationAxis::eTWIST, -1.3f, 1.3f);
}

static PxQuat rtz(PxPi / 2, PxVec3(0, 0, 1));
static PxQuat rtzinv(-PxPi / 2, PxVec3(0, 0, 1));

static PxTransform getJointPose(PxVec3 offset) {
	return PxTransform(rtzinv.rotate(offset));
}

void loadChest(PxVec3 offset) {
	PxVec3 jointOffsetParent = PxVec3(0.f, 0.944604f, 0.f);
	PxVec3 jointOffsetChild = -PxVec3(0.f, 0.48f, 0.f);
	PxVec3 linkOffset = offset + jointOffsetParent - jointOffsetChild;
	genSphereLink(chest, root, PxTransform(linkOffset, rtz), 0.44f, 14.f);
	genSphericalJoint(jRootChest, chest, getJointPose(jointOffsetParent), getJointPose(jointOffsetChild));

	PxReal kp = getConfigF("T_KP");
	PxReal kd = getConfigF("T_KD");

	jRootChest->setDrive(PxArticulationAxis::eTWIST, kp, kd, 100);
	jRootChest->setDrive(PxArticulationAxis::eSWING1, kp, kd, 100);
	jRootChest->setDrive(PxArticulationAxis::eSWING2, kp, kd, 100);
}

void loadRoot() {
	PxReal rootHeight = getConfigF("T_ROOT_HEIGHT");
	PxVec3 rootOffset(0.f, rootHeight, 0.f); 

	base = gArticulation->createLink(NULL, PxTransform(rootOffset, rtz));
	PxRigidBodyExt::updateMassAndInertia(*base, 1.0f);

	genSphereLink(root, base, PxTransform(rootOffset, rtz), 0.36f, 600.0f);
	auto jBaseRoot = static_cast<PxArticulationJointReducedCoordinate*>(root->getInboundJoint());
	jBaseRoot->setJointType(PxArticulationJointType::eFIX);
	jBaseRoot->setParentPose(PxTransform(PxVec3(0, 0, 0)));
	jBaseRoot->setChildPose(PxTransform(PxVec3(0, 0, 0)));

	loadChest(rootOffset);
}