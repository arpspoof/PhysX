#include "robot.h"
#include "config.h"
#include "globals.h"

#include <iostream>

using namespace physx;
using namespace std;

PxArticulationLink *base, *root;
PxArticulationLink *chest, *neck;
PxArticulationLink *rHip, *rKnee, *rAnkle;
PxArticulationLink *lHip, *lKnee, *lAnkle;

PxArticulationJointReducedCoordinate *jRootChest, *jRootRHip, *jRootLHip;
PxArticulationJointReducedCoordinate *jChestNeck;


static void genSphereLink(PxArticulationLink *&link, PxArticulationLink *parent, PxTransform transform,
	PxReal r, PxReal m) {
	link = gArticulation->createLink(parent, transform);
	PxRigidActorExt::createExclusiveShape(*link, /*PxSphereGeometry(r)*/PxBoxGeometry(r, r ,r), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*link, m / (4.0f / 3.0f*PxPi*r*r*r));
}

static void genCapsuleLink(PxArticulationLink *&link, PxArticulationLink *parent, PxTransform transform,
	PxReal r, PxReal l, PxReal m) {
	link = gArticulation->createLink(parent, transform);
	PxRigidActorExt::createExclusiveShape(*link, PxCapsuleGeometry(r, l / 2), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*link, m / (4.0f / 3.0f*PxPi*r*r*r + 4 * PxPi*r*r*l));
}

static void genSphericalJoint(PxArticulationJointReducedCoordinate *&joint, string name,
	PxArticulationLink *link, PxTransform parentPose, PxTransform childPose) {
	joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
	joint->setJointType(PxArticulationJointType::eSPHERICAL);
	joint->setParentPose(parentPose);
	joint->setChildPose(childPose);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
	joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
	joint->setLimit(PxArticulationAxis::eTWIST, -1.3f, 1.3f);

	PxReal kp = getConfigF("P_KP_" + name);
	PxReal kd = getConfigF("P_KD_" + name);

	joint->setDrive(PxArticulationAxis::eTWIST, kp, kd, 100);
	joint->setDrive(PxArticulationAxis::eSWING1, kp, kd, 100);
	joint->setDrive(PxArticulationAxis::eSWING2, kp, kd, 100);
}

static PxQuat rtz(PxPi / 2, PxVec3(0, 0, 1));
static PxQuat rtzinv(-PxPi / 2, PxVec3(0, 0, 1));

static PxTransform getJointPose(PxVec3 offset) {
	return PxTransform(rtzinv.rotate(offset));
}

static void loadNeck(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, 0.895576f, 0.f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, 0.7f, 0.f);
	genSphereLink(neck, chest, PxTransform(linkOffset, rtz), 0.41f, 2.f);
	genSphericalJoint(jChestNeck, "jChestNeck", neck,
		getJointPose(jointOffset - prevLinkOffset),
		getJointPose(jointOffset - linkOffset)
	);
}

static void loadRHip(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, 0.f, 0.339548f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, -0.84f, 0.f);
	genCapsuleLink(rHip, root, PxTransform(linkOffset, rtz), 0.22f, 1.2f, 4.5f);
	genSphericalJoint(jRootRHip, "jRootRHip", rHip,
		getJointPose(jointOffset - prevLinkOffset),
		getJointPose(jointOffset - linkOffset)
	);
}

static void loadLHip(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, 0.f, -0.339548f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, -0.84f, 0.f);
	genCapsuleLink(lHip, root, PxTransform(linkOffset, rtz), 0.22f, 1.2f, 4.5f);
	genSphericalJoint(jRootLHip, "jRootLHip", lHip,
		getJointPose(jointOffset - prevLinkOffset),
		getJointPose(jointOffset - linkOffset)
	);
}

static void loadChest(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, 0.944604f, 0.f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, 0.48f, 0.f);
	genSphereLink(chest, root, PxTransform(linkOffset, rtz), 0.44f, 14.f);
	genSphericalJoint(jRootChest, "jRootChest", chest, 
		getJointPose(jointOffset - prevLinkOffset), 
		getJointPose(jointOffset - linkOffset)
	);
	loadNeck(jointOffset, linkOffset);
}

void loadRoot() {
	PxReal baseHeight = getConfigF("T_BASE_HEIGHT");
	PxVec3 baseOffset(0.f, baseHeight, 0.f);

	base = gArticulation->createLink(NULL, PxTransform(baseOffset, rtz));
	PxRigidBodyExt::updateMassAndInertia(*base, 1.0f);

	PxVec3 rootLinkOffset = baseOffset + PxVec3(0.f, 0.28f, 0.f);
	PxVec3 rootJointOffset = baseOffset;

	genSphereLink(root, base, PxTransform(rootLinkOffset, rtz), 0.36f, 600.0f);
	auto jBaseRoot = static_cast<PxArticulationJointReducedCoordinate*>(root->getInboundJoint());
	jBaseRoot->setJointType(PxArticulationJointType::eFIX);
	jBaseRoot->setParentPose(PxTransform(rootJointOffset - baseOffset));
	jBaseRoot->setChildPose(PxTransform(rootJointOffset - rootLinkOffset));

	loadChest(rootJointOffset, rootJointOffset);
	loadRHip(rootJointOffset, rootJointOffset);
	loadLHip(rootJointOffset, rootJointOffset);
}