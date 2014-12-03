#ifndef RS_TYPES_HPP_
#define RS_TYPES_HPP_

namespace rs {

	// 3 dimensional vector
	struct Vec3 {
		Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
		double x, y, z;
	};

} // namespace rs

#endif // RS_TYPES_HPP_

