#include <rs/Vec>

using namespace rs;

Vec::Vec(void) {
}

Vec::Vec(double a, double b) {
	_v.push_back(a);
	_v.push_back(b);
}

Vec::Vec(double a, double b, double c) {
	_v.push_back(a);
	_v.push_back(b);
	_v.push_back(c);
}

Vec::Vec(double a, double b, double c, double d) {
	_v.push_back(a);
	_v.push_back(b);
	_v.push_back(c);
	_v.push_back(d);
}

Vec::Vec(const Vec &v) {
	for (int i = 0; i < v.size(); i++) {
		_v.push_back(v[i]);
	}
}

Vec::~Vec(void) {
	_v.clear();
}

/**********************************************************
	public functions
 **********************************************************/
void Vec::add(double a) {
	_v.push_back(a);
}

int Vec::size(void) const {
	return _v.size();
}

