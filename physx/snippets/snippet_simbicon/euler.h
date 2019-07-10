// COMPILE: g++ -o quat2EulerTest quat2EulerTest.cpp 
#include <iostream>
#include <cmath> 

using namespace std;

///////////////////////////////
// Quaternion struct
// Simple incomplete quaternion struct for demo purpose
///////////////////////////////
struct Quaternion {
	Quaternion() :x(0), y(0), z(0), w(1) {};
	Quaternion(double x, double y, double z, double w) :x(x), y(y), z(z), w(w) {};

	void normalize() {
		double norm = std::sqrt(x*x + y * y + z * z + w * w);
		x /= norm;
		y /= norm;
		z /= norm;
		w /= norm;
	}

	double norm() {
		return std::sqrt(x*x + y * y + z * z + w * w);
	}

	double x;
	double y;
	double z;
	double w;

};

///////////////////////////////
// Quaternion to Euler
///////////////////////////////
enum RotSeq { zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy, xzx };

// note: 
// return values of res[] depends on rotSeq.
// i.e.
// for rotSeq zyx, 
// x = res[0], y = res[1], z = res[2]
// for rotSeq xyz
// z = res[0], y = res[1], x = res[2]
// ...
void quaternion2Euler(const Quaternion& q, double res[], RotSeq rotSeq);

