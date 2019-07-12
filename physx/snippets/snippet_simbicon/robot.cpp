#include "robot.h"
#include "config.h"
#include "globals.h"
#include "articulationTree.h"

#include <iostream>
#include <string>

using namespace physx;
using namespace std;

PxArticulationLink *base, *root;
PxArticulationLink *chest, *neck;
PxArticulationLink *rHip, *rKnee, *rAnkle;
PxArticulationLink *lHip, *lKnee, *lAnkle;

PxArticulationJointReducedCoordinate *jRootChest, *jRootRHip, *jRootLHip;
PxArticulationJointReducedCoordinate *jChestNeck;
PxArticulationJointReducedCoordinate *jRHipRKnee, *jLHipLKnee;

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

	PxReal twistLimit = getConfigF("C_TWIST_LIMIT");
	joint->setLimit(PxArticulationAxis::eTWIST, -twistLimit, twistLimit);

	PxReal kp = getConfigF("P_KP_" + name);
	PxReal kd = getConfigF("P_KD_" + name);

	joint->setDrive(PxArticulationAxis::eTWIST, kp, kd, 100);
	joint->setDrive(PxArticulationAxis::eSWING1, kp, kd, 100);
	joint->setDrive(PxArticulationAxis::eSWING2, kp, kd, 100);
}

static void genRevoluteJoint(PxArticulationJointReducedCoordinate *&joint, string name,
	PxArticulationLink *link, PxArticulationAxis::Enum axis,
	PxTransform parentPose, PxTransform childPose) {
	joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setParentPose(parentPose);
	joint->setChildPose(childPose);
	joint->setMotion(axis, PxArticulationMotion::eFREE);

	PxReal kp = getConfigF("P_KP_" + name);
	PxReal kd = getConfigF("P_KD_" + name);

	joint->setDrive(axis, kp, kd, 100);
}

static PxQuat rtz(PxPi / 2, PxVec3(0, 0, 1));
static PxQuat rtzinv(-PxPi / 2, PxVec3(0, 0, 1));

static PxTransform getJointPose(PxVec3 offset) {
	return PxTransform(rtzinv.rotate(offset));
}

void loadNeck(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, 0.895576f, 0.f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, 0.7f, 0.f);
	genSphereLink(neck, chest, PxTransform(linkOffset, rtz), 0.41f, 2.f);
	genSphericalJoint(jChestNeck, "neck", neck,
		getJointPose(jointOffset - prevLinkOffset),
		getJointPose(jointOffset - linkOffset)
	);
}

void loadRKnee(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, -1.686184f, 0.f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, -0.8f, 0.f);
	genCapsuleLink(rKnee, rHip, PxTransform(linkOffset, rtz), 0.20f, 1.24f, 3.f);
	genRevoluteJoint(jRHipRKnee, "jRHipRKnee", rKnee, PxArticulationAxis::eSWING2,
		getJointPose(jointOffset - prevLinkOffset),
		getJointPose(jointOffset - linkOffset)
	);
}

void loadLKnee(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, -1.686184f, 0.f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, -0.8f, 0.f);
	genCapsuleLink(lKnee, lHip, PxTransform(linkOffset, rtz), 0.20f, 1.24f, 3.f);
	genRevoluteJoint(jLHipLKnee, "jLHipLKnee", lKnee, PxArticulationAxis::eSWING2,
		getJointPose(jointOffset - prevLinkOffset),
		getJointPose(jointOffset - linkOffset)
	);
}

void loadRHip(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, 0.f, 0.339548f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, -0.84f, 0.f);
	genCapsuleLink(rHip, root, PxTransform(linkOffset, rtz), 0.22f, 1.2f, 4.5f);
	genSphericalJoint(jRootRHip, "jRootRHip", rHip,
		getJointPose(jointOffset - prevLinkOffset),
		getJointPose(jointOffset - linkOffset)
	);
	loadRKnee(jointOffset, linkOffset);
}

void loadLHip(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, 0.f, -0.339548f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, -0.84f, 0.f);
	genCapsuleLink(lHip, root, PxTransform(linkOffset, rtz), 0.22f, 1.2f, 4.5f);
	genSphericalJoint(jRootLHip, "jRootLHip", lHip,
		getJointPose(jointOffset - prevLinkOffset),
		getJointPose(jointOffset - linkOffset)
	);
	loadLKnee(jointOffset, linkOffset);
}

void loadChest(PxVec3 prevJointOffset, PxVec3 prevLinkOffset) {
	PxVec3 jointOffset = prevJointOffset + PxVec3(0.f, 0.944604f, 0.f);
	PxVec3 linkOffset = jointOffset + PxVec3(0.f, 0.48f, 0.f);
	genSphereLink(chest, root, PxTransform(linkOffset, rtz), 0.44f, 14.f);
	genSphericalJoint(jRootChest, "chest", chest, 
		getJointPose(jointOffset - prevLinkOffset), 
		getJointPose(jointOffset - linkOffset)
	);
	loadNeck(jointOffset, linkOffset);
}

void loadRoot8() {
	PxReal baseHeight = getConfigF("T_BASE_HEIGHT");
	PxVec3 baseOffset(0.f, baseHeight, 0.f);

	base = gArticulation->createLink(NULL, PxTransform(baseOffset, rtz));
	PxRigidBodyExt::updateMassAndInertia(*base, 1.0f);

	PxVec3 rootJointOffset = baseOffset;
	PxVec3 rootLinkOffset = baseOffset + PxVec3(0.f, 0.28f, 0.f);

	genSphereLink(root, base, PxTransform(rootLinkOffset, rtz), 0.36f, 6.0f);
	auto jBaseRoot = static_cast<PxArticulationJointReducedCoordinate*>(root->getInboundJoint());
	jBaseRoot->setJointType(PxArticulationJointType::eFIX);
	jBaseRoot->setParentPose(getJointPose(rootJointOffset - baseOffset));
	jBaseRoot->setChildPose(getJointPose(rootJointOffset - rootLinkOffset));

	loadChest(rootJointOffset, rootLinkOffset);
//	loadRHip(rootJointOffset, rootLinkOffset);
//	loadLHip(rootJointOffset, rootLinkOffset);
}

// Link bodies
NULLLinkBody bodyBase;
SphereLinkBody bodyRoot(6.f, 0.36f);
SphereLinkBody bodyChest(14.f, 0.48f);
SphereLinkBody bodyNeck(2.f, 0.41f);
CapsuleLinkBody bodyHip(4.5f, 0.22f, 1.2f);
CapsuleLinkBody bodyKnee(3.f, 0.2f, 1.24f);

// Descriptions
NULLDescriptionNode descrBase("base", &bodyBase);
FixedDescriptionNode descrRoot("root", "root", &bodyRoot, 
	PxVec3(0, 0.28f, 0), PxVec3(0, 0, 0));
SpericalDescriptionNode descrChest("chest", "chest", &bodyChest, 
	PxVec3(0, 0.48f, 0), PxVec3(0, 0.944604f, 0));
SpericalDescriptionNode descrNeck("neck", "neck", &bodyNeck, 
	PxVec3(0, 0.7f, 0), PxVec3(0, 0.895576f, 0));
SpericalDescriptionNode descrRHip("right_hip", "right_hip", &bodyHip, 
	PxVec3(0, -0.84f, 0), PxVec3(0, 0, 0.339548f));
SpericalDescriptionNode descrLHip("left_hip", "left_hip", &bodyHip, 
	PxVec3(0, -0.84f, 0), PxVec3(0, 0, -0.339548f));
RevoluteDescriptionNode descrRKnee("right_knee", "right_knee", &bodyKnee, PxArticulationAxis::eSWING2,
	PxVec3(0, -0.8f, 0), PxVec3(0, -1.686184f, 0));
RevoluteDescriptionNode descrLKnee("left_knee", "left_knee", &bodyKnee, PxArticulationAxis::eSWING2,
	PxVec3(0, -0.8f, 0), PxVec3(0, -1.686184f, 0));

Articulation ar;

void loadRoot() {
	ArticulationTree arTree;

	arTree.addNULLDescriptionNode(descrBase);
	arTree.setRoot("base");

	arTree.addFixedDescriptionNode(descrRoot);
	arTree.connect("base", "root");

	arTree.addSpericalDescriptionNode(descrChest);
	arTree.connect("root", "chest");

	arTree.addSpericalDescriptionNode(descrNeck);
	arTree.connect("chest", "neck");

	arTree.addSpericalDescriptionNode(descrRHip);
	arTree.connect("root", "right_hip");

	arTree.addSpericalDescriptionNode(descrLHip);
	arTree.connect("root", "left_hip");

	arTree.addRevoluteDescriptionNode(descrRKnee);
	arTree.connect("right_hip", "right_knee");

	arTree.addRevoluteDescriptionNode(descrLKnee);
	arTree.connect("left_hip", "left_knee");

	arTree.buildArticulation(ar);
}
