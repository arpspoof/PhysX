#include "globals.h"
#include "euler.h"

static bool gClosing = true;

extern PxArticulationJointReducedCoordinate* joint12;
extern PxArticulationLink* link1;
extern PxArticulationLink* link2;

PxReal twistTarget, swing1Target, swing2Target;

extern PxArticulationJointReducedCoordinate *jRootChest, *jRootRHip, *jRootLHip;
extern PxArticulationJointReducedCoordinate *jChestNeck;
extern PxArticulationJointReducedCoordinate *jRHipRKnee, *jLHipLKnee;

void control(PxReal /*dt*/) {
//	jLHipLKnee->setDriveTarget(PxArticulationAxis::eTWIST, twistTarget);
//	jLHipLKnee->setDriveTarget(PxArticulationAxis::eSWING1, swing1Target);
	jLHipLKnee->setDriveTarget(PxArticulationAxis::eSWING2, swing2Target);
}

void control5(PxReal /*dt*/) {
	joint12->setDriveTarget(PxArticulationAxis::eTWIST, twistTarget);
	joint12->setDriveTarget(PxArticulationAxis::eSWING1, swing1Target);
	joint12->setDriveTarget(PxArticulationAxis::eSWING2, swing2Target);

	PxTransform t1 = link1->getGlobalPose();
	PxTransform t2 = link2->getGlobalPose();
	PxQuat rel = t1.q.getConjugate() * t2.q;

	/*PxQuat testTwist(twistTarget, PxVec3(1, 0, 0));
	PxVec3 swingComp(0, swing1Target, swing2Target);
	PxQuat testSwing(swingComp.magnitude(), swingComp.getNormalized());
	PxQuat test = testSwing * testTwist;*/

	/*PxVec3 compo(twistTarget, swing1Target, swing2Target);
	PxQuat test(compo.magnitude(), compo.getNormalized());*/

	PxQuat test =
		PxQuat(swing2Target, PxVec3(0, 0, 1)) *
		PxQuat(swing1Target, PxVec3(0, 1, 0)) *
		PxQuat(twistTarget, PxVec3(1, 0, 0));

	printf("[%.5f,%.5f,%.5f,%.5f]\t[%.5f,%.5f,%.5f,%.5f]\n",
		test.w, test.x, test.y, test.z,
		rel.w, rel.x, rel.y, rel.z);
}

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
