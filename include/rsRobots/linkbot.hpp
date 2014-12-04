#ifndef RSROBOTS_LINKBOT_HPP_
#define RSROBOTS_LINKBOT_HPP_

#include <vector>

#include <rsRobots/robot.hpp>
namespace rsScene {
	class Scene;
}

namespace rsRobots {

	class LinkbotT : virtual public Robot {
			friend class rsScene::Scene;
		// api
		public:
			LinkbotT(int);
			virtual ~LinkbotT(void);

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
			double _conn_depth;
			double _conn_height;
			double _conn_radius;
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
			LinkbotI(void) : Robot(rs::LINKBOTI), LinkbotT(rs::JOINT2) {}
	};

	class LinkbotL : public LinkbotT {
		public:
			LinkbotL(void) : Robot(rs::LINKBOTL), LinkbotT(rs::JOINT3) {}
	};

} // namespace rsRobots

#endif // RSROBOTS_LINKBOT_HPP_

