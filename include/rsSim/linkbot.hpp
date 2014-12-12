#ifndef RSSIM_LINKBOT_HPP_
#define RSSIM_LINKBOT_HPP_

#include <rs/macros.hpp>
#include <rs/enum.hpp>
#include <rsRobots/linkbot.hpp>
#include <rsSim/modularRobot.hpp>
#include <rsSim/robot.hpp>

namespace rsSim {

	// class
	class CLinkbotT : public rsRobots::LinkbotT, virtual public ModularRobot {
			friend class Sim;
		// public api
		public:
			CLinkbotT(int = -1);
			virtual ~CLinkbotT();

			int accelJointAngleNB(rs::JointID, double, double);
			int accelJointCycloidalNB(rs::JointID, double, double);
			int accelJointHarmonicNB(rs::JointID, double, double);
			int accelJointSmoothNB(rs::JointID, double, double, double, double);
			int accelJointTimeNB(rs::JointID, double, double);
			int accelJointToMaxSpeedNB(rs::JointID, double);
			int accelJointToVelocityNB(rs::JointID, double, double);
			int closeGripper(void);
			int closeGripperNB(void);
			int driveAccelCycloidalNB(double, double, double);
			int driveAccelDistanceNB(double, double, double);
			int driveAccelHarmonicNB(double, double, double);
			int driveAccelSmoothNB(double, double, double, double, double);
			int driveAccelTimeNB(double, double, double);
			int driveAccelToMaxSpeedNB(double, double);
			int driveAccelToVelocityNB(double, double, double);
			int driveForeverNB(void);
			int driveForwardNB(double);
			int drivexyTo(double, double, double, double);
			int drivexyToSmooth(double, double, double, double, double, double, double, double);
			int getJointAngles(double&, double&, double&, int = 10);
			int getJointAnglesInstant(double&, double&, double&);
			int getJointSpeeds(double&, double&, double&);
			int getJointSpeedRatios(double&, double&, double&);
			int move(double, double, double);
			int moveNB(double, double, double);
			int moveTo(double, double, double);
			int moveToNB(double, double, double);
			int moveToByTrackPos(double, double, double);
			int moveToByTrackPosNB(double, double, double);
			int openGripper(double);
			int openGripperNB(double);
			int recordAngles(double[], double[], double[], double[], int, double, int = 1);
			int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
			int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
			int setJointSpeeds(double, double, double);
			int setJointSpeedRatios(double, double, double);
			int turnLeftNB(double, double, double);
			int turnRightNB(double, double, double);

		// inherited functions from ModularRobot class
		private:
			virtual int addConnector(int, int, double);
			virtual int build(rsXML::Robot*, dMatrix3, double*, dBodyID, rsXML::Conn*) { return 0; };
			virtual int fixBodyToConnector(dBodyID, int);
			virtual int fixConnectorToBody(int, dBodyID, int = -1);
			virtual int getConnectorParams(int, int, dMatrix3, double*);
			virtual int getFaceParams(int, dMatrix3, double*);

		// inherited functions from Robot class
		private:
			virtual int build(int, const double*, const double*, const double*, int);
			virtual int buildIndividual(double, double, double, dMatrix3, double*);
			virtual double getAngle(int);
			virtual int initParams(int);
			virtual int initDims(void);
			virtual void simPreCollisionThread(void);
			virtual void simPostCollisionThread(void);

		// private functions
		private:
			int add_connector_daisy(int, int, double, int, int);						// add daisy chained connector
			int build_bigwheel(Connector*, int, int = -1, int = -1);					// build big wheel connector
			int build_body(dGeomID*, double, double, double, dMatrix3, double);			// build body of linkbot
			int build_bridge(Connector*, int, int = -1, int = -1);						// build bridge connector
			int build_caster(Connector*, int, int, int = -1, int = -1);					// build caster connector
			int build_cube(Connector*, int, int = -1, int = -1);						// build cube connector
			int build_face(int, dGeomID*, double, double, double, dMatrix3, double);	// build face of linkbot
			int build_faceplate(Connector*, int, int = -1, int = -1);					// build faceplate connector
			int build_gripper(Connector*, int);											// build gripper connector
			int build_omnidrive(Connector*, int, int = -1, int = -1);					// build omnidrive connector
			int build_simple(Connector*, int, int = -1, int = -1);						// build simple connector
			int build_smallwheel(Connector*, int, int = -1, int = -1);					// build small wheel connector
			int build_tinywheel(Connector*, int, int = -1, int = -1);					// build tiny wheel connector
			int build_wheel(Connector*, int, double, int = -1, int = -1);				// build custom wheel connector
			static void* closeGripperNBThread(void*);									// thread to close gripper

		// private data
		private:
			// robot body parts
			enum robot_pieces_e {
				BODY,
				FACE1,
				FACE2,
				FACE3,
				NUM_PARTS
			};

			// dimensions
			double	_bridge_length,
					_cubic_length,
					_face_depth,
					_face_radius,
					_omni_length,
					_tinywheel_radius;
	};

	// class
	class CLinkbotI : public rsRobots::LinkbotI, public CLinkbotT {
		public:
			CLinkbotI(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::Robot(rs::JOINT1, rs::JOINT3) {}
	};

	// class
	class CLinkbotL : public CLinkbotT {
		public:
			CLinkbotL(void) : rsRobots::Robot(rs::LINKBOTL), rsSim::Robot(rs::JOINT1, rs::JOINT2) {}
	};

	// motion threading
	struct LinkbotMove {
		rsSim::CLinkbotT *robot;
		char *expr;
		double x, y, radius, trackwidth;
		double (*func)(double x);
		int i;
	};

} // namespace rsSim

/*
class DLLIMPORT CLinkbotTGroup : public Group<rsSim::CLinkbotT> {
	// public api
	public:
		CLinkbotTGroup(void) : Group<rsSim::CLinkbotT>() {};
		virtual ~CLinkbotTGroup(void) {};

		inline int accelJointAngleNB(rs::JointID, double, double);
		inline int accelJointCycloidalNB(rs::JointID, double, double);
		inline int accelJointHarmonicNB(rs::JointID, double, double);
		inline int accelJointSmoothNB(rs::JointID, double, double, double, double);
		inline int accelJointTimeNB(rs::JointID, double, double);
		inline int accelJointToMaxSpeedNB(rs::JointID, double);
		inline int accelJointToVelocityNB(rs::JointID, double, double);
		inline int closeGripper(void);
		inline int closeGripperNB(void);
		inline int driveAccelCycloidalNB(double, double, double);
		inline int driveAccelDistanceNB(double, double, double);
		inline int driveAccelHarmonicNB(double, double, double);
		inline int driveAccelSmoothNB(double, double, double, double, double);
		inline int driveAccelTimeNB(double, double, double);
		inline int driveAccelToMaxSpeedNB(double, double);
		inline int driveAccelToVelocityNB(double, double, double);
		inline int move(double, double, double);
		inline int moveNB(double, double, double);
		inline int moveTo(double, double, double);
		inline int moveToNB(double, double, double);
		inline int moveToByTrackPos(double, double, double);
		inline int moveToByTrackPosNB(double, double, double);
		inline int openGripper(double);
		inline int openGripperNB(double);
		inline int setJointSpeeds(double, double, double);
		inline int setJointSpeedRatios(double, double, double);
};
class DLLIMPORT CLinkbotIGroup : public CLinkbotTGroup {};
class DLLIMPORT CLinkbotLGroup : public CLinkbotTGroup {};
#include "linkbotgroup.tpp"
*/

#endif // RSSIM_LINKBOT_HPP_

