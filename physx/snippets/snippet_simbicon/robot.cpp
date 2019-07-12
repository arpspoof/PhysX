#include "robot.h"
#include "config.h"
#include "globals.h"
#include "articulationTree.h"

#include <iostream>
#include <string>

using namespace physx;
using namespace std;

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
