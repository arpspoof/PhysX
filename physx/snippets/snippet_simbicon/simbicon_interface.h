#pragma once

#include <PxPhysicsAPI.h>

typedef physx::PxVec3 vec3;
typedef physx::PxQuat quat;

//////////////////////////////

void setLHipOri(vec3 pos);
void setRHipOri(vec3 pos);

void setLKneeOri(float pos);
void setRKneeOri(float pos);

void setLAnkleOri(float pos);
void setRAnkleOri(float pos);

vec3 getCOMGlobalPos();
vec3 getLFootCOMGlobalPos();
vec3 getRFootCOMGlobalPos();

// Note: method to compute root force can be different
vec3 getBaseOri();
vec3 getBaseAngularVelocity();
