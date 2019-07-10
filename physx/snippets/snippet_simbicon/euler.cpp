// COMPILE: g++ -o quat2EulerTest quat2EulerTest.cpp 
#include "euler.h"

static void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
	res[0] = atan2(r11, r12);
	res[1] = acos(r21);
	res[2] = atan2(r31, r32);
}

static void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
	res[0] = atan2(r31, r32);
	res[1] = asin(r21);
	res[2] = atan2(r11, r12);
}

// note: 
// return values of res[] depends on rotSeq.
// i.e.
// for rotSeq zyx, 
// x = res[0], y = res[1], z = res[2]
// for rotSeq xyz
// z = res[0], y = res[1], x = res[2]
// ...
void quaternion2Euler(const Quaternion& q, double res[], RotSeq rotSeq)
{
	switch (rotSeq) {
	case zyx:
		threeaxisrot(2 * (q.x*q.y + q.w*q.z),
			q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
			-2 * (q.x*q.z - q.w*q.y),
			2 * (q.y*q.z + q.w*q.x),
			q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
			res);
		break;

	case zyz:
		twoaxisrot(2 * (q.y*q.z - q.w*q.x),
			2 * (q.x*q.z + q.w*q.y),
			q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
			2 * (q.y*q.z + q.w*q.x),
			-2 * (q.x*q.z - q.w*q.y),
			res);
		break;

	case zxy:
		threeaxisrot(-2 * (q.x*q.y - q.w*q.z),
			q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
			2 * (q.y*q.z + q.w*q.x),
			-2 * (q.x*q.z - q.w*q.y),
			q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
			res);
		break;

	case zxz:
		twoaxisrot(2 * (q.x*q.z + q.w*q.y),
			-2 * (q.y*q.z - q.w*q.x),
			q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
			2 * (q.x*q.z - q.w*q.y),
			2 * (q.y*q.z + q.w*q.x),
			res);
		break;

	case yxz:
		threeaxisrot(2 * (q.x*q.z + q.w*q.y),
			q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
			-2 * (q.y*q.z - q.w*q.x),
			2 * (q.x*q.y + q.w*q.z),
			q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
			res);
		break;

	case yxy:
		twoaxisrot(2 * (q.x*q.y - q.w*q.z),
			2 * (q.y*q.z + q.w*q.x),
			q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
			2 * (q.x*q.y + q.w*q.z),
			-2 * (q.y*q.z - q.w*q.x),
			res);
		break;

	case yzx:
		threeaxisrot(-2 * (q.x*q.z - q.w*q.y),
			q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
			2 * (q.x*q.y + q.w*q.z),
			-2 * (q.y*q.z - q.w*q.x),
			q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
			res);
		break;

	case yzy:
		twoaxisrot(2 * (q.y*q.z + q.w*q.x),
			-2 * (q.x*q.y - q.w*q.z),
			q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
			2 * (q.y*q.z - q.w*q.x),
			2 * (q.x*q.y + q.w*q.z),
			res);
		break;

	case xyz:
		threeaxisrot(-2 * (q.y*q.z - q.w*q.x),
			q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
			2 * (q.x*q.z + q.w*q.y),
			-2 * (q.x*q.y - q.w*q.z),
			q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
			res);
		break;

	case xyx:
		twoaxisrot(2 * (q.x*q.y + q.w*q.z),
			-2 * (q.x*q.z - q.w*q.y),
			q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
			2 * (q.x*q.y - q.w*q.z),
			2 * (q.x*q.z + q.w*q.y),
			res);
		break;

	case xzy:
		threeaxisrot(2 * (q.y*q.z + q.w*q.x),
			q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
			-2 * (q.x*q.y - q.w*q.z),
			2 * (q.x*q.z + q.w*q.y),
			q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
			res);
		break;

	case xzx:
		twoaxisrot(2 * (q.x*q.z - q.w*q.y),
			2 * (q.x*q.y + q.w*q.z),
			q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
			2 * (q.x*q.z + q.w*q.y),
			-2 * (q.x*q.y - q.w*q.z),
			res);
		break;
	default:
		std::cout << "Unknown rotation sequence" << std::endl;
		break;
	}
}
