#ifndef RSROBOTS_MINDSTORMS_HPP_
#define RSROBOTS_MINDSTORMS_HPP_

#include <rsRobots/robot.hpp>

namespace rsRobots {

	class Mindstorms : virtual public Robot {
		// public functions
		public:
			Mindstorms(void);
			virtual ~Mindstorms(void) {};

		// inherited functions
		public:
			virtual void getRobotBodyOffset(int, const double*, const double*, double*, double*);
	};

} // namespace rsRobots

#endif // RSROBOTS_MINDSTORMS_HPP_

