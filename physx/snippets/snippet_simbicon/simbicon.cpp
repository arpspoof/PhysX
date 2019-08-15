#include "simbicon.h"
#include "simbicon_interface.h"
#include "config.h"
#include <stdio.h>

float timer = 0;

/*
	0: Lift left foot, wait timer
	1: Left foot strike, wait contact
	2: Lift right foot, wait timer
	3: Right foot strike, wait contact
*/
int state = 1;

void simbicon_tick(float dt, int contact) {
	if (state & 1) {
		timer = 0;
		if (state == 1 && contact & SIMBICON_LFOOT_CONTACT ||
			state == 3 && contact & SIMBICON_RFOOT_CONTACT) {
			state = (state + 1) % 4;
		//	printf("contact detected %d, state move to %d\n", contact, state);
		}
	}
	else {
		timer += dt;
		if (timer >= SIMBICON_TIMER) {
			timer = 0;
			state = (state + 1) % 4;
		//	printf("time out, state move to %d\n", state);
		}
	}
}

void simbicon_setTargets() {
	float swingHipSwing = 0.75f;
	float swingHipStrike = -0.2f;
	float swingKneeSwing = -1.1f;
	float swingKneeStrike = -0.05f;
	float stanceKneeSwing = -0.05f;
	float stanceKneeStrike = -0.1f;
	float swingAnkleSwing = 0.6f;
	float swingAnkleStrike = 0.05f;
	float stanceAnkle = 0.0f;

	setChestTarget(vec3(0, 0, -0.2f));

	vec3 stanceFootCOM = state < 2 ? getRFootCOMGlobalPos() : getLFootCOMGlobalPos();
	vec3 stanceFootToCenterCOM = getCOMGlobalPos() - stanceFootCOM;
	vec3 comV = getLinearVelocity();

	float cdSagital = getConfigF("P_CD_SAGITAL");
	float cvSagital = getConfigF("P_CV_SAGITAL");
	float cdCoronal = getConfigF("P_CD_CORONAL");
	float cvCoronal = getConfigF("P_CV_CORONAL");

	float balanceSagital = stanceFootToCenterCOM.x * cdSagital + comV.x * cvSagital;
	float balanceCoronal = stanceFootToCenterCOM.z * cdCoronal + comV.z * cvCoronal;

	float swingHipSagitalGlobal = (state & 1 ? swingHipStrike : swingHipSwing) + balanceSagital;
	float swingHipCoronalGlobal = -balanceCoronal;

	quat swingHipGlobal =
		quat(swingHipSagitalGlobal, vec3(0, 0, 1)) *
		quat(swingHipCoronalGlobal, vec3(0, -1, 0));

	vec3 axis(swingHipGlobal.x, swingHipGlobal.y, swingHipGlobal.z);
	float angle = 2 * atan2(axis.magnitude(), swingHipGlobal.w);
	vec3 expMap = axis.getNormalized() * angle;
	expMap[0] = 0;

	expMap = vec3(0, -swingHipCoronalGlobal, swingHipSagitalGlobal);

	switch (state) {
	case 0:
		setLHipTarget(expMap);
		setLKneeTarget(swingKneeSwing);
		setLAnkleTarget(swingAnkleSwing);
		setRKneeTarget(stanceKneeSwing);
		setRAnkleTarget(stanceAnkle);
		break;
	case 1:
		setLHipTarget(expMap);
		setLKneeTarget(swingKneeStrike);
		setLAnkleTarget(swingAnkleStrike);
		setRKneeTarget(stanceKneeStrike);
		setRAnkleTarget(stanceAnkle);
		break;
	case 2:
		setRHipTarget(expMap);
		setRKneeTarget(swingKneeSwing);
		setRAnkleTarget(swingAnkleSwing);
		setLKneeTarget(stanceKneeSwing);
		setLAnkleTarget(stanceAnkle);
		break;
	case 3:
		setRHipTarget(expMap);
		setRKneeTarget(swingKneeStrike);
		setRAnkleTarget(swingAnkleStrike);
		setLKneeTarget(stanceKneeStrike);
		setLAnkleTarget(stanceAnkle);
		break;
	}
}

/*void simbicon_updateForces() {
	quat rootOri = getQuat(SBC_PI / 2, vec3(0, 0, 1)) * getBaseOri().getConjugate();
	vec3 rootAngularV = getBaseAngularVelocity();

	vec3 rootOriAxis(rootOri.x, rootOri.y, rootOri.z);
	float rootOriAngle = 2 * atan2(rootOriAxis.magnitude(), rootOri.w);
	rootOriAxis.normalize();

	float kp = getConfigF("P_KP_root");
	float kd = getConfigF("P_KD_root");

	vec3 rootForce = getBaseOri().getConjugate().rotate(rootOriAngle * kp * rootOriAxis - kd * rootAngularV);

	vec3 chestForce = getChestForce();
	vec3 swingHipForce = state < 2 ? getLHipForce() : getRHipForce();

	vec3 stanceHipForce = -(rootForce + chestForce + swingHipForce);

	if (state < 2) {
		setRHipForce(stanceHipForce);
	}
	else {
		setLHipForce(stanceHipForce);
	}
}*/

void simbicon_updateForces() {
	quat rootOri = getBaseOri();
	vec3 rootAngularV = getBaseAngularVelocity();

	float kp = getConfigF("P_KP_root");
	float kd = getConfigF("P_KD_root");

	quat rtz(0, 0, 0.7071068f, 0.7071068f);
	quat targetDifference = rtz * rootOri.getConjugate();

	vec3 targetDiffAxis;
	float targetDiffAngle;
	targetDifference.toRadiansAndUnitAxis(targetDiffAngle, targetDiffAxis);

	vec3 rootForceGlobal = targetDiffAngle * kp * targetDiffAxis - kd * rtz.getConjugate().rotate(rootAngularV);
	vec3 rootForceInRootFrame = rootOri.getConjugate().rotate(rootForceGlobal);

	vec3 chestForceInRootFrame = getChestOriLocal().rotate(getChestForce());

	quat swingHipOriLocal = state < 2 ? getLHipOriLocal() : getRHipOriLocal();
	vec3 swingHipForceInRootFrame = swingHipOriLocal.rotate(state < 2 ? getLHipForce() : getRHipForce());

	vec3 stanceHipForceInRootFrame = 
		-(rootForceInRootFrame + chestForceInRootFrame + swingHipForceInRootFrame);

	quat stanceHipOriLocal = state < 2 ? getRHipOriLocal() : getLHipOriLocal();
	vec3 stanceHipForceInLocalFrame = stanceHipOriLocal.getConjugate().rotate(stanceHipForceInRootFrame);

	if (state < 2) {
		setRHipForce(stanceHipForceInLocalFrame);
	}
	else {
		setLHipForce(stanceHipForceInLocalFrame);
	}
}
