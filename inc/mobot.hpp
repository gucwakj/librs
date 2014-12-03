#ifndef MOBOT_HPP_
#define MOBOT_HPP_

#include "robosim.hpp"

class DLLIMPORT CMobot : public ModularRobot {
#ifdef ENABLE_GRAPHICS
		friend class mobotNodeCallback;
		friend class Graphics;
#endif // ENABLE_GRAPHICS

	// public api
	public:
		CMobot(void);
		virtual ~CMobot(void);

		int getJointAngles(double&, double&, double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&, double&, double&);
		int getJointSpeeds(double&, double&, double&, double&);
		int getJointSpeedRatios(double&, double&, double&, double&);
		int motionArch(double);
		int motionArchNB(double);
		int motionDistance(double, double);
		int motionDistanceNB(double, double);
		int motionInchwormLeft(int);
		int motionInchwormLeftNB(int);
		int motionInchwormRight(int);
		int motionInchwormRightNB(int);
		int motionRollBackward(double);
		int motionRollBackwardNB(double);
		int motionRollForward(double);
		int motionRollForwardNB(double);
		int motionSkinny(double);
		int motionSkinnyNB(double);
		int motionStand(void);
		int motionStandNB(void);
		int motionTumbleLeft(int);
		int motionTumbleLeftNB(int);
		int motionTumbleRight(int);
		int motionTumbleRightNB(int);
		int motionTurnLeft(double);
		int motionTurnLeftNB(double);
		int motionTurnRight(double);
		int motionTurnRightNB(double);
		int motionUnstand(void);
		int motionUnstandNB(void);
		int motionWait(void);
		int move(double, double, double, double);
		int moveNB(double, double, double, double);
		int moveTo(double, double, double, double);
		int moveToNB(double, double, double, double);
		int moveToByTrackPos(double, double, double, double);
		int moveToByTrackPosNB(double, double, double, double);
		int recordAngles(double[], double[], double[], double[], double[], int, double, int = 1);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int setJointSpeeds(double, double, double, double);
		int setJointSpeedRatios(double, double, double, double);

	// inherited functions from ModularRobot class
	private:
		virtual int addConnector(int, int, double);
		virtual int build(XMLRobot*, dMatrix3, double*, dBodyID, XMLConn*);
		virtual int fixBodyToConnector(dBodyID, int);
		virtual int fixConnectorToBody(int, dBodyID, int = -1);
		virtual int getConnectorParams(int, int, dMatrix3, double*);
		virtual int getFaceParams(int, dMatrix3, double*);

	// inherited functions from Robot class
	private:
		virtual int build(XMLRobot*, int = 1);
		virtual int buildIndividual(double, double, double, dMatrix3, double*);
		virtual double getAngle(int);
		virtual int initParams(int, int);
		virtual int initDims(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

	// private functions
	private:
		int build_bigwheel(Connector*, int);							// build big wheel connector
		int build_body(int, double, double, double, dMatrix3, double);	// build body of mobot
		int build_caster(Connector*, int);								// build caster connector
		int build_center(double, double, double, dMatrix3);				// build center body of mobot
		int build_endcap(int, double, double, double, dMatrix3);		// build endcap of mobot
		int build_simple(Connector*, int);								// build simple connector
		int build_smallwheel(Connector*, int);							// build small wheel connector
		int build_square(Connector*, int);								// build square connector
		int build_tank(Connector*, int);								// build tank connector
		int build_wheel(Connector*, int, double);						// build custom wheel connector
		static void* motionArchThread(void*);							// thread for arching
		static void* motionDistanceThread(void*);						// thread for driving a distance
		static void* motionInchwormLeftThread(void*);					// thread for inchworming left
		static void* motionInchwormRightThread(void*);					// thread for inchworming right
		static void* motionRollBackwardThread(void*);					// thread for rolling backward
		static void* motionRollForwardThread(void*);					// thread for rolling forward
		static void* motionSkinnyThread(void*);							// thread for skinny driving
		static void* motionStandThread(void*);							// thread for standing
		static void* motionTurnLeftThread(void*);						// thread for turning left
		static void* motionTurnRightThread(void*);						// thread for turning right
		static void* motionTumbleLeftThread(void*);						// thread for for tumbling left
		static void* motionTumbleRightThread(void*);					// thread for tumbling right
		static void* motionUnstandThread(void*);						// thread for unstanding

	// private data
	private:
		// robot body parts
		enum robot_pieces_e {
			CENTER,
			ENDCAP_L,
			BODY_L,
			BODY_R,
			ENDCAP_R,
			NUM_PARTS
		};

		// dimensions
		double	_body_end_depth,
				_body_inner_width_left,
				_body_inner_width_right,
				_body_mount_center,
				_center_length,
				_center_height,
				_center_offset,
				_center_radius,
				_center_width,
				_end_depth,
				_end_height,
				_end_radius,
				_end_width,
				_tank_height,
				_tank_depth;
};

class DLLIMPORT CMobotGroup : public Group<CMobot> {
	// public api
	public:
		inline CMobotGroup(void);
		inline virtual ~CMobotGroup(void);

		inline int motionArch(double);
		inline int motionArchNB(double);
		inline int motionDistance(double, double);
		inline int motionDistanceNB(double, double);
		inline int motionInchwormLeftNB(int);
		inline int motionInchwormRight(int);
		inline int motionInchwormRightNB(int);
		inline int motionRollBackward(double);
		inline int motionRollBackwardNB(double);
		inline int motionRollForwardNB(double);
		inline int motionSkinny(double);
		inline int motionSkinnyNB(double);
		inline int motionRollForward(double);
		inline int motionInchwormLeft(int);
		inline int motionStand(void);
		inline int motionStandNB(void);
		inline int motionTurnLeft(double);
		inline int motionTurnLeftNB(double);
		inline int motionTurnRight(double);
		inline int motionTurnRightNB(double);
		inline int motionTumbleRight(int);
		inline int motionTumbleRightNB(int);
		inline int motionTumbleLeft(int);
		inline int motionTumbleLeftNB(int);
		inline int motionUnstand(void);
		inline int motionUnstandNB(void);
		inline int motionWait(void);
		inline int move(double, double, double, double);
		inline int moveNB(double, double, double, double);
		inline int moveTo(double, double, double, double);
		inline int moveToNB(double, double, double, double);
		inline int moveToByTrackPos(double, double, double, double);
		inline int moveToByTrackPosNB(double, double, double, double);
		inline int setJointSpeeds(double, double, double, double);
		inline int setJointSpeedRatios(double, double, double, double);

	// private functions
	private:
		inline static void* motionArchThread(void*);
		inline static void* motionDistanceThread(void*);
		inline static void* motionInchwormLeftThread(void*);
		inline static void* motionInchwormRightThread(void*);
		inline static void* motionRollBackwardThread(void*);
		inline static void* motionRollForwardThread(void*);
		inline static void* motionSkinnyThread(void*);
		inline static void* motionStandThread(void*);
		inline static void* motionTurnLeftThread(void*);
		inline static void* motionTurnRightThread(void*);
		inline static void* motionTumbleRightThread(void*);
		inline static void* motionTumbleLeftThread(void*);
		inline static void* motionUnstandThread(void*);

	// private data
	private:
		THREAD_T *_thread;
		int _motion;
};
#include "mobotgroup.tpp"

// motion threading
struct MobotMove {
	CMobot *robot;
	double d;
	int i;
};

// simulation
//extern RoboSim *g_sim;

#endif  // MOBOT_HPP_

