#ifndef RSROBOTS_ROBOT_HPP_
#define RSROBOTS_ROBOT_HPP_

#include <cmath>
#include <iostream>
#include <vector>

#include <rs/enum.hpp>
#include <rs/types.hpp>
#include <rsRobots/rgbhashtable.h>

namespace rsRobots {

	class Robot {
		// public functions
		public:
			Robot(int);
			virtual ~Robot(void);

			void getOffsetPos(int, const double*, double*);
			void getRGB(double*);
			void setTrace(bool);

		// data
		protected:
			bool _trace;
			std::vector<rs::Vec3> _offset;
			double _body_length;
			double _body_height;
			double _body_radius;
			double _body_width;
			double _wheel_depth;
			double _wheel_radius;
			double _radius;
			double _rgb[3];
			int _form;
	};

} // namespace rsRobots

#endif // RSROBOTS_ROBOT_HPP_

