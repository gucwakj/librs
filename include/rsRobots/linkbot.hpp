#ifndef RSROBOTS_LINKBOT_HPP_
#define RSROBOTS_LINKBOT_HPP_

#include <rsRobots/modularRobot.hpp>

namespace rsRobots {

	class LinkbotT : virtual public ModularRobot {
		// public functions
		public:
			LinkbotT(int = -1);
			virtual ~LinkbotT(void) {};

			void getOffsetQuat(int, const double*, double*);

		// enumerations
		public:
			enum robot_pieces_e {
				BODY,
				FACE1,
				FACE2,
				FACE3,
				NUM_PARTS
			};

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

	class LinkbotI : public LinkbotT {
		public:
			LinkbotI(void) : Robot(rs::LINKBOTI), LinkbotT(rs::JOINT2) {};
	};

	class LinkbotL : public LinkbotT {
		public:
			LinkbotL(void) : Robot(rs::LINKBOTL), LinkbotT(rs::JOINT3) {};
	};

} // namespace rsRobots

#endif // RSROBOTS_LINKBOT_HPP_

