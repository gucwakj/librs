#include <iostream>

#include <rs/Pos>
#include <rs/Quat>

using namespace rs;

Quat::Quat(void) {
	_v[0] = 0;
	_v[1] = 0;
	_v[2] = 0;
	_v[3] = 1;
}

Quat::Quat(float x, float y, float z, float w) {
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
const Pos Quat::multiply(const Pos &p) {
	float uv[3] = {_v[1]*p[2] - _v[2]*p[1], _v[2]*p[0] - _v[0]*p[2], _v[0]*p[1] - _v[1]*p[0]};
	float uuv[3] = {2*(_v[1]*uv[2] - _v[2]*uv[1]), 2*(_v[2]*uv[0] - _v[0]*uv[2]), 2*(_v[0]*uv[1] - _v[1]*uv[0])};
	return Pos(p[0] + 2*_v[3]*uv[0] + uuv[0], p[1] + 2*_v[3]*uv[1] + uuv[1], p[2] + 2*_v[3]*uv[2] + uuv[2]);
}

const Pos Quat::multiply(float x, float y, float z) const {
	float uv[3] = {_v[1]*z - _v[2]*y, _v[2]*x - _v[0]*z, _v[0]*y - _v[1]*x};
	float uuv[3] = {2*(_v[1]*uv[2] - _v[2]*uv[1]), 2*(_v[2]*uv[0] - _v[0]*uv[2]), 2*(_v[0]*uv[1] - _v[1]*uv[0])};
	return Pos(x + 2*_v[3]*uv[0] + uuv[0], y + 2*_v[3]*uv[1] + uuv[1], z + 2*_v[3]*uv[2] + uuv[2]);
}

const Quat Quat::multiply(const Quat &q) {
	float x = _v[3]*q[0] + _v[0]*q[3] + _v[1]*q[2] - _v[2]*q[1];
	float y = _v[3]*q[1] - _v[0]*q[2] + _v[1]*q[3] + _v[2]*q[0];
	float z = _v[3]*q[2] + _v[0]*q[1] - _v[1]*q[0] + _v[2]*q[3];
	_v[3] = _v[3]*q[3] - _v[0]*q[0] - _v[1]*q[1] - _v[2]*q[2];
	_v[2] = z;
	_v[1] = y;
	_v[0] = x;

	return (*this);
}

const Quat Quat::multiply(float x, float y, float z, float w){
	float x1 = _v[3]*x + _v[0]*w + _v[1]*z - _v[2]*y;
	float y1 = _v[3]*y - _v[0]*z + _v[1]*w + _v[2]*x;
	float z1 = _v[3]*z + _v[0]*y - _v[1]*x + _v[2]*w;
	_v[3] = _v[3]*w - _v[0]*x - _v[1]*y - _v[2]*z;
	_v[2] = z1;
	_v[1] = y1;
	_v[0] = x1;

	return (*this);
}

void Quat::print(const char *name) {
	std::cerr << name << ": " << _v[0] << " " << _v[1] << " " << _v[2] << " " << _v[3] << std::endl;
}

void Quat::print(const char *name) const {
	std::cerr << name << ": " << _v[0] << " " << _v[1] << " " << _v[2] << " " << _v[3] << std::endl;
}

