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

	// bodies
	enum Body {
		BODY,
		FACE1,
		FACE2,
		FACE3,
		NUM_PARTS
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

	// preconfigs
	enum Preconfig {
		INDIVIDUAL,
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

} // namespace rsLinkbot

/*namespace rsMobot {

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

	// preconfigs
	enum Preconfig {
		EXPLORER,
		LIFT,
		NUM_PRECONFIG
	};

} // namespace rsMobot

namespace rsCubus {

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

	// preconfigs
	enum Preconfig {
		NUM_PRECONFIG
	};

} // namespace rsCubus*/

#endif // RS_ENUM_HPP_

