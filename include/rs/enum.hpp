#ifndef RS_ENUM_HPP_
#define RS_ENUM_HPP_

namespace rs {

	// robot forms
	enum Form {
		CUBUS,
		LINKBOTI,
		LINKBOTL,
		LINKBOTT,
		MOBOT,
		NXT,
		ROBOT,
		NUM_TYPES
	};

	// drawing objects (ground and graphic)
	enum Drawing {
		BOX,
		CYLINDER,
		DOT,
		LINE,
		SPHERE,
		TEXT
	};

} // namespace rs

namespace rsLinkbot {

	// shapes
	enum Preconfig {
		ROBOT,
		BOW,
		EXPLORER,
		FOURBOTDRIVE,
		FOURWHEELDRIVE,
		FOURWHEELEXPLORER,
		GROUPBOW,
		INCHWORM,
		LIFT,
		OMNIDRIVE,
		SNAKE,
		STAND,
		NUM_PRECONFIG
	};

	// connectors
	enum Connector {
		BIGWHEEL,
		BRIDGE,
		CASTER,
		CUBE,
		FACEPLATE,
		GRIPPER,
		OMNIPLATE,
		SIMPLE,
		SMALLWHEEL,
		TINYWHEEL,
		WHEEL,
		NUM_CONNECTORS
	};

	// joints
	enum Joint {
		JOINT1,
		JOINT2,
		JOINT3,
		NUM_JOINTS
	};

} // namespace rsLinkbot

/*namespace rsMobot {

	// shapes
	enum Preconfig {
		EXPLORER,
		LIFT,
		NUM_PRECONFIG
	};

	// connectors
	enum Connector {
		BIGWHEEL,
		CASTER,
		SIMPLE,
		SMALLWHEEL,
		SQUARE,
		TANK,
		WHEEL,
		NUM_CONNECTORS
	};

	// joints
	enum Joint {
		JOINT1,
		JOINT2,
		JOINT3,
		JOINT4,
		NUM_JOINTS
	};

} // namespace rsMobot

namespace rsCubus {

	// shapes
	enum Preconfig {
		NUM_PRECONFIG
	};

	// connectors
	enum Connector {
		SIMPLE,
		NUM_CONNECTORS
	};

	// joints
	enum Joint {
		JOINT1,
		JOINT2,
		JOINT3,
		JOINT4,
		JOINT5,
		JOINT6,
		NUM_JOINTS
	};

} // namespace rsCubus*/

#endif // RS_ENUM_HPP_

