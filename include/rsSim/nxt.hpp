#ifndef NXT_HPP_
#define NXT_HPP_

#include "robosim.hpp"

class DLLIMPORT CNXT : virtual public Robot {
#ifdef ENABLE_GRAPHICS
		friend class nxtNodeCallback;
		friend class Graphics;
#endif // ENABLE_GRAPHICS

	// public api
	public:
		CNXT(void);
		virtual ~CNXT(void);

		int getJointAngles(double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&);
		int getJointSpeeds(double&, double&);
		int getJointSpeedRatios(double&, double&);
		int move(double, double);
		int moveNB(double, double);
		int moveTo(double, double);
		int moveToNB(double, double);
		int moveToByTrackPos(double, double);
		int moveToByTrackPosNB(double, double);
		int recordAngles(double[], double[], double[], int, double, int = 1);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int setJointSpeeds(double, double);
		int setJointSpeedRatios(double, double);

	// inherited functions from Robot class
    private:
		int build(XMLRobot*, int = 1);
		int buildIndividual(double, double, double, dMatrix3, double*);
		double getAngle(int);
		int initParams(int, int);
		int initDims(void);
		void simPreCollisionThread(void);
		void simPostCollisionThread(void);

	// private functions
    private:
		int build_body(double, double, double, dMatrix3, double);		// build body of linkbot
		int build_wheel(int, double, double, double, dMatrix3, double);	// build wheels of nxt

	// private data
	private:
		// robot body parts
		enum robot_pieces_e {
			BODY,
			WHEEL1,
			WHEEL2,
			NUM_PARTS
		};
};

class DLLIMPORT CNXTGroup : public Group<CNXT> {
	public:
		CNXTGroup(void) : Group<CNXT>() {};
		virtual ~CNXTGroup(void) {};

		inline int move(double, double);
		inline int moveNB(double, double);
		inline int moveTo(double, double);
		inline int moveToNB(double, double);
		inline int moveToByTrackPos(double, double);
		inline int moveToByTrackPosNB(double, double);
		inline int setJointSpeeds(double, double);
		inline int setJointSpeedRatios(double, double);
};
#include "nxtgroup.tpp"

// simulation
//extern RoboSim *g_sim;

#endif // NXT_HPP_

