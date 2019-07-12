#pragma once

#include <PxPhysicsAPI.h>

class LinkBody {
public:
	float mass;
	virtual float getDensity() const = 0;
	virtual physx::PxGeometry getGeometry() const = 0;
protected:
	LinkBody(float mass) :mass(mass) {}
};

class SphereLinkBody : public LinkBody {
public:
	float radius;
	float getDensity() const override {
		return mass / (4.0f / 3.0f*physx::PxPi*radius*radius*radius);
	}
	SphereLinkBody(float mass, float radius)
		:LinkBody(mass), radius(radius) {
	}
	physx::PxGeometry getGeometry() const override {
		return physx::PxSphereGeometry(radius);
	}
};

class CapsuleLinkBody : public LinkBody {
public:
	float radius;
	float length;
	float getDensity() const override {
		return mass / (
			4.0f / 3.0f*physx::PxPi*radius*radius*radius +
			4 * physx::PxPi*radius*radius*length
		);
	}
	CapsuleLinkBody(float mass, float radius, float length)
		:LinkBody(mass), radius(radius), length(length) {
	}
	physx::PxGeometry getGeometry() const override {
		return physx::PxCapsuleGeometry(radius, length / 2);
	}
};
