#include "simbicon.h"
#include "simbicon_interface.h"
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

}

void simbicon_updateForces() {

}
