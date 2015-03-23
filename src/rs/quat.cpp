#include <iostream>
#include <rs/Quat>

using namespace rs;

Quat::Quat(void) {
	_v[0] = 0;
	_v[1] = 0;
	_v[2] = 0;
	_v[3] = 1;
}

Quat::Quat(double x, double y, double z, double w) {
	_v[0] = x;
	_v[1] = y;
	_v[2] = z;
	_v[3] = w;
}

Quat::Quat(const Quat &q) {
	_v[0] = q[0];
	_v[1] = q[1];
	_v[2] = q[2];
	_v[3] = q[3];
}

/**********************************************************
	public functions
 **********************************************************/
void Quat::multiply(double x, double y, double z, double *o) const {
	double uv[3] = {_v[1]*z - _v[2]*y, _v[2]*x - _v[0]*z, _v[0]*y - _v[1]*x};
	double uuv[3] = {2*(_v[1]*uv[2] - _v[2]*uv[1]), 2*(_v[2]*uv[0] - _v[0]*uv[2]), 2*(_v[0]*uv[1] - _v[1]*uv[0])};
	o[0] = x + 2*_v[3]*uv[0] + uuv[0];
	o[1] = y + 2*_v[3]*uv[1] + uuv[1];
	o[2] = z + 2*_v[3]*uv[2] + uuv[2];
}

const Quat Quat::multiply(const Quat &q1) {
	double x = q1[3]*_v[0] + q1[0]*_v[3] + q1[1]*_v[2] - q1[2]*_v[1];
	double y = q1[3]*_v[1] - q1[0]*_v[2] + q1[1]*_v[3] + q1[2]*_v[0];
	double z = q1[3]*_v[2] + q1[0]*_v[1] - q1[1]*_v[0] + q1[2]*_v[3];
	_v[3] = q1[3]*_v[3] - q1[0]*_v[0] - q1[1]*_v[1] - q1[2]*_v[2];
	_v[2] = z;
	_v[1] = y;
	_v[0] = x;

	return (*this);
}

const Quat Quat::multiply(double x, double y, double z, double w) {
	double x1 = _v[3]*x + _v[0]*w + _v[1]*z - _v[2]*y;
	double y1 = _v[3]*y - _v[0]*z + _v[1]*w + _v[2]*x;
	double z1 = _v[3]*z + _v[0]*y - _v[1]*x + _v[2]*w;
	_v[3] = _v[3]*w - _v[0]*x - _v[1]*y - _v[2]*z;
	_v[2] = z1;
	_v[1] = y1;
	_v[0] = x1;

	return (*this);
}

