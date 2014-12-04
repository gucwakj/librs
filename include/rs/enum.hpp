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

	// TODO: put connectors into rsRobots classes
	// cant be done yet with xml implementation
	/*
	namespace Linkbot {
		enum Connector {
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
		};
	} // namespace Linkbot

	namespace Mobot {
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
	} // namespace Mobot

	namespace Cubus {
		enum Connector {
			SIMPLE,
			NUM_CONNECTORS
		};
	} // namespace Cubus
	*/

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

#endif // RS_ENUM_HPP_

