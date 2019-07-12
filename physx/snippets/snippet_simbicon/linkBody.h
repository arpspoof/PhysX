#pragma once

#include <PxPhysicsAPI.h>
#include <cassert>

class LinkBody {
public:
	bool hasGeometry;
	float mass;
	physx::PxGeometry *geometry;
	virtual physx::PxGeometry& getGeometry() const {
		return *geometry;
	}
	virtual float getDensity() const = 0;
	virtual ~LinkBody() {
		delete geometry;
	}
protected:
	LinkBody(float mass, physx::PxGeometry *geometry) 
		:mass(mass), hasGeometry(true), geometry(geometry) {}
};

class NULLLinkBody : public LinkBody {
public:
	float getDensity() const override {
		return 1;
	}
	NULLLinkBody() :LinkBody(1, NULL) {
		hasGeometry = false;
	}
	physx::PxGeometry& getGeometry() const override {
		assert(false);
		return *geometry;
	}
};

class SphereLinkBody : public LinkBody {
public:
	float radius;
	float getDensity() const override {
		return mass / (4.0f / 3.0f*physx::PxPi*radius*radius*radius);
	}
	SphereLinkBody(float mass, float radius)
		:LinkBody(mass, 
			/*new physx::PxSphereGeometry(radius)),*/
			new physx::PxBoxGeometry(radius, radius, radius)),
		radius(radius) {
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
		:LinkBody(mass, new physx::PxCapsuleGeometry(radius, length / 2)), 
		radius(radius), length(length) {
	}
};