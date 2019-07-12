#include "articulationTree.h"
#include "globals.h"
#include "config.h"

using namespace physx;

static PxQuat rtz(PxPi / 2, PxVec3(0, 0, 1));
static PxQuat rtzinv(-PxPi / 2, PxVec3(0, 0, 1));

static PxTransform getJointPose(PxVec3 offset) {
	return PxTransform(rtzinv.rotate(offset));
}

Joint::Joint(Link *link, PxTransform parentPose, PxTransform childPose) {
	joint = static_cast<PxArticulationJointReducedCoordinate*>(link->link->getInboundJoint());
	joint->setParentPose(parentPose);
	joint->setChildPose(childPose);
}

SphericalJoint::SphericalJoint(Link *link, PxTransform parentPose, PxTransform childPose)
	: Joint(link, parentPose, childPose) {
	joint->setJointType(PxArticulationJointType::eSPHERICAL);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
	joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);

	PxReal twistLimit = getConfigF("C_TWIST_LIMIT");
	joint->setLimit(PxArticulationAxis::eTWIST, -twistLimit, twistLimit);
}

void SphericalJoint::enableDrive(std::string name) {
	PxReal kp = getConfigF("P_KP_" + name);
	PxReal kd = getConfigF("P_KD_" + name);

	joint->setDrive(PxArticulationAxis::eTWIST, kp, kd, 100);
	joint->setDrive(PxArticulationAxis::eSWING1, kp, kd, 100);
	joint->setDrive(PxArticulationAxis::eSWING2, kp, kd, 100);
}

RevoluteJoint::RevoluteJoint(Link *link, PxArticulationAxis::Enum axis,
	PxTransform parentPose, PxTransform childPose)
	: Joint(link, parentPose, childPose), axis(axis) {
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setMotion(axis, PxArticulationMotion::eFREE);
}

void RevoluteJoint::enableDrive(std::string name) {
	PxReal kp = getConfigF("P_KP_" + name);
	PxReal kd = getConfigF("P_KD_" + name);

	joint->setDrive(axis, kp, kd, 100);
}

Link::Link(PxTransform transform, LinkBody *body) {
	link = gArticulation->createLink(parentLink ? parentLink->link : NULL, transform);
	PxRigidActorExt::createExclusiveShape(*link, body->getGeometry(), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*link, body->getDensity());
}

Link* ArticulationDescriptionNode::createLink(Articulation& ar,
	PxVec3 parentJointPos, PxVec3 parentLinkPos) const {
	PxVec3 jointPos = parentJointPos + posOffsetJointToParentJoint;
	PxVec3 linkPos = jointPos + posOffsetLinkToInboundJoint;
	Link *link = ar.addLink(linkName, PxTransform(linkPos, rtz), body);
	Joint *joint = createJoint(ar, link,
		getJointPose(jointPos - parentLinkPos),
		getJointPose(jointPos - linkPos)
	);
	link->inboundJoint = joint;
	link->globalPositionOffset = linkPos;
	joint->globalPositionOffset = jointPos;
	return link;
}

Joint* SpericalDescriptionNode::createJoint(Articulation& ar, Link *link,
	physx::PxTransform parentPose,
	physx::PxTransform childPose) const {
	return ar.addSpericalJoint(jointName, link, parentPose, childPose);
}

Joint* RevoluteDescriptionNode::createJoint(Articulation& ar, Link *link,
	physx::PxTransform parentPose,
	physx::PxTransform childPose) const {
	return ar.addRevoluteJoint(jointName, link, axis, parentPose, childPose);
}
