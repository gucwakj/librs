#include <iostream>

#include <rs/Vec>

using namespace rs;

Vec::Vec(void) {
}

Vec::Vec(short size) {
	_v.resize(size);
}

Vec::Vec(float a) {
	_v.push_back(a);
}

Vec::Vec(float a, float b) {
	_v.push_back(a);
	_v.push_back(b);
}

Vec::Vec(float a, float b, float c) {
	_v.push_back(a);
	_v.push_back(b);
	_v.push_back(c);
}

Vec::Vec(float a, float b, float c, float d) {
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

