#ifndef RSROBOTS_MODULARROBOT_HPP_
#define RSROBOTS_MODULARROBOT_HPP_

#include <rsRobots/robot.hpp>

namespace rsRobots {

	class ModularRobot : virtual public Robot {
		// public functions
		public:
			ModularRobot(void) : Robot(rs::ROBOT) {};
			virtual ~ModularRobot(void) {};

		// data
		protected:
			double _conn_depth;
			double _conn_height;
			double _conn_radius;
	};

} // namepsace rsRobots

#endif // RSROBOTS_MODULARROBOT_HPP_

