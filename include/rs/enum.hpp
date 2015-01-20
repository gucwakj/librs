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

	// robot joints
	enum JointID {
		JOINT1,
		JOINT2,
		JOINT3,
		JOINT4,
		JOINT5,
		JOINT6
	};

	// robot connectors
	enum Connector {
		BIGWHEEL,
		BRIDGE,
		CASTER,
		CUBE,
		FACEPLATE,
		GRIPPER,
		L,
		OMNIDRIVE,
		SIMPLE,
		SMALLWHEEL,
		SQUARE,
		TANK,
		TINYWHEEL,
		WHEEL,
		NUM_CONNECTORS
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

	/*enum Connector {
		BIGWHEEL,
		BRIDGE,
		CASTER,
		CUBE,
		FACEPLATE,
		GRIPPER,
		OMNIDRIVE,
		SIMPLE,
		SMALLWHEEL,
		TINYWHEEL,
		WHEEL,
		NUM_CONNECTORS
	};*/

} // namespace rsLinkbot

/*namespace rsMobot {

	enum Preconfig {
		EXPLORER,
		LIFT,
		NUM_PRECONFIG
	};

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

} // namespace rsMobot

namespace rsCubus {

	enum Preconfig {
		NUM_PRECONFIG
	};

	enum Connector {
		SIMPLE,
		NUM_CONNECTORS
	};

} // namespace rsCubus*/

#endif // RS_ENUM_HPP_

