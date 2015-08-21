#include <iostream>

#include <rs/Vec>

using namespace rs;

Vec::Vec(void) {
}

Vec::Vec(int size) {
	_v.resize(size);
}

Vec::Vec(double a) {
	_v.push_back(a);
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
	for (unsigned int i = 0; i < v.size(); i++) {
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

void Vec::allocate(int size) {
	_v.resize(size);
	for (unsigned int i = 0; i < _v.size(); i++) {
		_v[i] = 0;
	}
}

void Vec::print(const char *name) {
	std::cerr << name << ": ";
	for (unsigned int i = 0; i < _v.size(); i++) {
		std::cerr << _v[i] << " ";
	}
	std::cerr << std::endl;
}

void Vec::print(const char *name) const {
	std::cerr << name << ": ";
	for (unsigned int i = 0; i < _v.size(); i++) {
		std::cerr << _v[i] << " ";
	}
	std::cerr << std::endl;
}

unsigned int Vec::size(void) const {
	return _v.size();
}

