#pragma once

#include <PxPhysicsAPI.h>

typedef physx::PxVec3 vec3;
typedef physx::PxQuat quat;

#define SBC_PI (physx::PxPi)

//////////////////////////////

void setLHipTarget(vec3 pos);
void setRHipTarget(vec3 pos);

void setLKneeTarget(float pos);
void setRKneeTarget(float pos);

void setLAnkleTarget(float pos);
void setRAnkleTarget(float pos);

void setChestTarget(vec3 pos);

vec3 getCOMGlobalPos();
vec3 getLFootCOMGlobalPos();
vec3 getRFootCOMGlobalPos();

// Note: method to compute root force can be different
quat getBaseOri();
vec3 getBaseAngularVelocity();

vec3 getLinearVelocity();

vec3 getChestForce();
vec3 getLHipForce();
vec3 getRHipForce();

void setLHipForce(vec3 f);
void setRHipForce(vec3 f);

quat getQuat(float angle, vec3 axis);
