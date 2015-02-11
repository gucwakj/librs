#ifndef RSSIM_LINKBOT_HPP_
#define RSSIM_LINKBOT_HPP_

#include <rs/macros.hpp>
#include <rs/enum.hpp>
#include <rsRobots/linkbot.hpp>
#include <rsSim/modularRobot.hpp>
#include <rsSim/robot.hpp>

namespace rsSim {

	// class
	class LinkbotT : virtual public rsRobots::LinkbotT, virtual public ModularRobot {
			friend class Sim;
		// public api
		public:
			LinkbotT(int = -1);
			virtual ~LinkbotT();

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
		protected:
			virtual int addConnector(int, int, double, int, int);
			virtual int build(int, const double*, const double*, const double*, dBodyID, int, int);
			virtual int fixBodyToConnector(dBodyID, int);
			virtual int fixConnectorToBody(int, dBodyID, int = -1);

		// inherited functions from Robot class
		protected:
			virtual int build(int, const double*, const double*, const double*, int);
			virtual int buildIndividual(const double*, const double*, const double*);
			virtual double getAngle(int);
			virtual void init_params(void);
			virtual void simPreCollisionThread(void);
			virtual void simPostCollisionThread(void);

		// private functions
		private:
			void build_body(dGeomID*, const double*, const double*);
			void build_bridge(Connector*);
			void build_caster(Connector*, int);
			void build_cube(Connector*);
			void build_face(int, dGeomID*, const double*, const double*);
			void build_faceplate(Connector*);
			void build_gripper(Connector*, int);
			void build_omnidrive(Connector*);
			void build_simple(Connector*);
			void build_wheel(Connector*, double);
			static void* closeGripperNBThread(void*);
	};

	// class
	class LinkbotI : public rsRobots::LinkbotI, public LinkbotT {
		public:
			LinkbotI(void) : LinkbotT(rs::JOINT2), rsRobots::Robot(rs::LINKBOTI), rsSim::Robot(rs::JOINT1, rs::JOINT3) {}
	};

	// class
	class LinkbotL : public LinkbotT {
		public:
			LinkbotL(void) : LinkbotT(rs::JOINT3), rsRobots::Robot(rs::LINKBOTL), rsSim::Robot(rs::JOINT1, rs::JOINT2) {}
	};

	// motion threading
	struct LinkbotMove {
		rsSim::LinkbotT *robot;
		char *expr;
		double x, y, radius, trackwidth;
		double (*func)(double x);
		int i;
	};

} // namespace rsSim

/*
class DLLIMPORT LinkbotTGroup : public Group<rsSim::LinkbotT> {
	// public api
	public:
		LinkbotTGroup(void) : Group<rsSim::LinkbotT>() {};
		virtual ~LinkbotTGroup(void) {};

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
class DLLIMPORT CLinkbotIGroup : public LinkbotTGroup {};
class DLLIMPORT CLinkbotLGroup : public LinkbotTGroup {};
#include "linkbotgroup.tpp"
*/

#endif // RSSIM_LINKBOT_HPP_

