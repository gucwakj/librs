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

			int getForm(void);
			int getID(void);
			void getOffsetPos(int, const double*, double*);
			double* getRGB(void);
			void setTrace(bool);

		// data
		protected:
			bool _trace;
			double _body_length;
			double _body_height;
			double _body_radius;
			double _body_width;
			double _radius;			// TODO: wtf this causes a segfault
			double _wheel_depth;
			double _wheel_radius;
			double _rgb[3];
			int _form;
			int _id;
			std::vector<rs::Vec3> _offset;
	};

} // namespace rsRobots

#endif // RSROBOTS_ROBOT_HPP_

