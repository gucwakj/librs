#include <rs/Pos>

using namespace rs;

Pos::Pos(void) {
	_v[0] = 0;
	_v[1] = 0;
	_v[2] = 0;
}

Pos::Pos(double x, double y, double z) {
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

const Pos Pos::add(double x, double y, double z) {
	_v[0] += x;
	_v[1] += y;
	_v[2] += z;

	return (*this);
}

