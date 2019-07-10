#pragma once

#include "PxPhysicsAPI.h"

using namespace physx;

extern PxDefaultAllocator		gAllocator;
extern PxDefaultErrorCallback	gErrorCallback;

extern PxFoundation*			gFoundation;
extern PxPhysics*				gPhysics;

extern PxDefaultCpuDispatcher*	gDispatcher;
extern PxScene*				gScene;

extern PxMaterial*				gMaterial;

extern PxPvd*                  gPvd;

extern PxArticulationReducedCoordinate*		gArticulation;
extern PxArticulationJointReducedCoordinate*	gDriveJoint;

void loader();
void control(PxReal dt);
void keyHandler(unsigned char key, const PxTransform& /*camera*/);
