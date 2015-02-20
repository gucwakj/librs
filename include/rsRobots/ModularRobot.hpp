#ifndef RSROBOTS_MODULARROBOT_HPP_
#define RSROBOTS_MODULARROBOT_HPP_

#include <rs/enum.hpp>
#include <rsRobots/robot.hpp>

namespace rsRobots {

	class ModularRobot : virtual public Robot {
		// public functions
		public:
			ModularRobot(void) : Robot(rs::ROBOT) {};
			virtual ~ModularRobot(void) {};

		// for inherited classes
		public:
			virtual void getConnBodyOffset(int, const double*, const double*, double*, double*) {};
			virtual void getConnFaceOffset(int, int, const double*, const double*, double*, double*) {};

		// data
		protected:
			double _conn_depth;
			double _conn_height;
			double _conn_radius;
	};

} // namepsace rsRobots

#endif // RSROBOTS_MODULARROBOT_HPP_
