#include "globals.h"
#include "config.h"

using namespace physx;

extern PxReal twistTarget, swing1Target, swing2Target;

void incre(PxReal& x, PxReal v)
{
	x += v;
	while (x > PxPi) x -= 2 * PxPi;
	while (x < -PxPi) x += 2 * PxPi;
}

void keyHandler(unsigned char key, const PxTransform& /*camera*/)
{
	switch (key) {
	case '1':
		writeConfigFile();
		break;
/*	case 'z': incre(twistTarget, 0.05); printf("twist %.2f\n", twistTarget); break;
	case 'x': incre(swing1Target, 0.05); printf("swing1 %.2f\n", swing1Target); break;
	case 'c': incre(swing2Target, 0.05); printf("swing2 %.2f\n", swing2Target); break;
	case 'v':
		setConfigF("T_KD", getConfigF("T_KD") + 0.01f); 
		printf("kd %.2f\n", getConfigF("T_KD"));
		extern void setKPKD();
		setKPKD();
		break;
	case 'b':
		setConfigF("T_KD", getConfigF("T_KD") - 0.01f);
		printf("kd %.2f\n", getConfigF("T_KD"));
		extern void setKPKD();
		setKPKD();
		break;*/
	}
}