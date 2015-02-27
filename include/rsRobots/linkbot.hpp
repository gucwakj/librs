#ifndef RSROBOTS_LINKBOT_HPP_
#define RSROBOTS_LINKBOT_HPP_

#include <rsRobots/ModularRobot.hpp>

namespace rsRobots {

	class Linkbot : virtual public ModularRobot {
		// public functions
		public:
			Linkbot(int = -1);
			virtual ~Linkbot(void) {};

			double getWheelRatio(int);

		// inherited functions
		public:
			virtual void getConnBodyOffset(int, const double*, const double*, double*, double*);
			virtual void getConnFaceOffset(int, int, const double*, const double*, double*, double*);
			virtual void getRobotBodyOffset(int, const double*, const double*, double*, double*);
			virtual void getRobotFaceOffset(int, const double*, const double*, double*, double*);

		// data
		protected:
			double _bigwheel_radius;
			double _bridge_length;
			double _cubic_length;
			double _face_depth;
			double _face_radius;
			double _omni_length;
			double _smallwheel_radius;
			double _tinywheel_radius;
			int _disabled;
	};

	class LinkbotI : public Linkbot {
		public:
			LinkbotI(void) : Robot(rs::LINKBOTI), Linkbot(rsLinkbot::JOINT2) {};
	};

	class LinkbotL : public Linkbot {
		public:
			LinkbotL(void) : Robot(rs::LINKBOTL), Linkbot(rsLinkbot::JOINT3) {};
	};

	class LinkbotT : public Linkbot {
		public:
			LinkbotT(void) : Robot(rs::LINKBOTT) {};
	};

} // namespace rsRobots

#endif // RSROBOTS_LINKBOT_HPP_

