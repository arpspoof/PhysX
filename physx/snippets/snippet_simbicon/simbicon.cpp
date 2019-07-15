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
			printf("contact detected %d, state move to %d\n", contact, state);
		}
	}
	else {
		timer += dt;
		if (timer >= SIMBICON_TIMER) {
			timer = 0;
			state = (state + 1) % 4;
			printf("time out, state move to %d\n", state);
		}
	}
}

void simbicon_setTargets() {
	float swingHipSwing = 0.8f;
	float swingHipStrike = -0.1f;
	float swingKneeSwing = -1.1f;
	float swingKneeStrike = -0.05f;
	float stanceKneeSwing = -0.05f;
	float stanceKneeStrike = -0.1f;
	float swingAnkleSwing = 0.6f;
	float swingAnkleStrike = 0.05f;
	float stanceAnkle = 0.0f;

	setChestTarget(vec3(0, 0, -0.1f));

	vec3 stanceFootCOM = state < 2 ? getRFootCOMGlobalPos() : getLFootCOMGlobalPos();
	vec3 stanceFootToCenterCOM = getCOMGlobalPos() - stanceFootCOM;
	vec3 comV = getLinearVelocity();

	float cdSagital = getConfigF("P_CD_SAGITAL");
	float cvSagital = getConfigF("P_CV_SAGITAL");
	float cdCoronal = getConfigF("P_CD_CORONAL");
	float cvCoronal = getConfigF("P_CV_CORONAL");

	float balanceSagital = stanceFootToCenterCOM.x * cdSagital + comV.x * cvSagital;
	float balanceCoronal = stanceFootToCenterCOM.z * cdCoronal + comV.z * cvCoronal;

	switch (state) {
	case 0:
		setLHipTarget(vec3(0, balanceCoronal, swingHipSwing + balanceSagital));
		setLKneeTarget(swingKneeSwing);
		setLAnkleTarget(swingAnkleSwing);
		setRKneeTarget(stanceKneeSwing);
		setRAnkleTarget(stanceAnkle);
		break;
	case 1:
		setLHipTarget(vec3(0, balanceCoronal, swingHipStrike + balanceSagital));
		setLKneeTarget(swingKneeStrike);
		setLAnkleTarget(swingAnkleStrike);
		setRKneeTarget(stanceKneeStrike);
		setRAnkleTarget(stanceAnkle);
		break;
	case 2:
		setRHipTarget(vec3(0, balanceCoronal, swingHipSwing + balanceSagital));
		setRKneeTarget(swingKneeSwing);
		setRAnkleTarget(swingAnkleSwing);
		setLKneeTarget(stanceKneeSwing);
		setLAnkleTarget(stanceAnkle);
		break;
	case 3:
		setRHipTarget(vec3(0, balanceCoronal, swingHipStrike + balanceSagital));
		setRKneeTarget(swingKneeStrike);
		setRAnkleTarget(swingAnkleStrike);
		setLKneeTarget(stanceKneeStrike);
		setLAnkleTarget(stanceAnkle);
		break;
	}
}

void simbicon_updateForces() {
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
}
