#ifndef RSSIM_LINKBOT_HPP_
#define RSSIM_LINKBOT_HPP_

#include <tuple>

#include <rsRobots/Linkbot>
#include <rsSim/modularRobot.hpp>

namespace rsSim {

	// class
	class Linkbot : virtual public rsRobots::Linkbot, virtual public ModularRobot {
			// TODO: remove
			friend class Sim;
		// public api
		public:
			Linkbot(void);
			virtual ~Linkbot(void);

			int accelJointAngleNB(rsLinkbot::Joint, double, double);
			int accelJointCycloidalNB(rsLinkbot::Joint, double, double);
			int accelJointHarmonicNB(rsLinkbot::Joint, double, double);
			int accelJointSmoothNB(rsLinkbot::Joint, double, double, double, double);
			int accelJointTimeNB(rsLinkbot::Joint, double, double);
			int accelJointToMaxSpeedNB(rsLinkbot::Joint, double);
			int accelJointToVelocityNB(rsLinkbot::Joint, double, double);
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

		public:
			virtual void getCoM(double&, double&, double&);
			void addForce(int, double, double, double);

		// inherited functions from ModularRobot class
		protected:
			virtual int addConnector(int, int, double, int, int);
			virtual int build(const double*, const double*, const double*, dBodyID, int, int);
			virtual int fixBodyToConnector(dBodyID, int);
			virtual int fixConnectorToBody(int, dBodyID, int = -1);

		// inherited functions from Robot class
		protected:
			virtual int build(const double*, const double*, const double*, int);
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

		private:
			std::vector<std::tuple<int, double, double, double>> _f;		// forces to add to bodies
	};

	// class
	class LinkbotI : public Linkbot {
		public:
			LinkbotI(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {}
	};

	// class
	class LinkbotL : public Linkbot {
		public:
			LinkbotL(void) : rsRobots::Robot(rs::LINKBOTL), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT2) {}
	};

	// class
	class LinkbotT : public Linkbot {
		public:
			LinkbotT(void) : rsRobots::Robot(rs::LINKBOTT), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {}
	};

	// motion threading
	struct LinkbotMove {
		rsSim::Linkbot *robot;
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

		inline int accelJointAngleNB(rsLinkbot::Joint, double, double);
		inline int accelJointCycloidalNB(rsLinkbot::Joint, double, double);
		inline int accelJointHarmonicNB(rsLinkbot::Joint, double, double);
		inline int accelJointSmoothNB(rsLinkbot::Joint, double, double, double, double);
		inline int accelJointTimeNB(rsLinkbot::Joint, double, double);
		inline int accelJointToMaxSpeedNB(rsLinkbot::Joint, double);
		inline int accelJointToVelocityNB(rsLinkbot::Joint, double, double);
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

