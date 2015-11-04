#include <iostream>

#include <rs/Pos>

using namespace rs;

Pos::Pos(float x, float y, float z) {
	_v[0] = x;
	_v[1] = y;
	_v[2] = z;
}

Pos::Pos(const Pos &p) {
	_v[0] = p[0];
	_v[1] = p[1];
	_v[2] = p[2];
}

/**********************************************************
	public functions
 **********************************************************/
const Pos Pos::add(const Pos &p) {
	_v[0] += p[0];
	_v[1] += p[1];
	_v[2] += p[2];

	return (*this);
}

const Pos Pos::add(float x, float y, float z) {
	_v[0] += x;
	_v[1] += y;
	_v[2] += z;

	return (*this);
}

void Pos::print(const char *name) {
	std::cerr << name << ": " << _v[0] << " " << _v[1] << " " << _v[2] << std::endl;
}

void Pos::print(const char *name) const {
	std::cerr << name << ": " << _v[0] << " " << _v[1] << " " << _v[2] << std::endl;
}

