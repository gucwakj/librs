#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#include <Shlwapi.h>
#else
#include <pthread.h>
#include <unistd.h>
#endif // _WIN32
#include <ode/ode.h>
#include <cctype>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <ctime>
#include <vector>

#include "macros.hpp"
#include "rgbhashtable.h"

// forward declare friends
class RoboSim;
class RSGUI;

class DLLIMPORT Robot {
		friend class RoboSim;
		friend class RSGUI;

	// common public api
	public:
		Robot(robotJointId_t, robotJointId_t);
		virtual ~Robot(void);

		int blinkLED(double, int);
		int connect(char* = NULL, int = 3);
		int delay(double);
		int delaySeconds(double);
		int disableRecordDataShift(void);
		int disconnect(void);
		int driveBackward(double);
		int driveBackwardNB(double);
		int driveDistance(double, double);
		int driveDistanceNB(double, double);
		int driveForever(void);
		virtual int driveForeverNB(void);
		int driveForward(double);
		virtual int driveForwardNB(double);
		int driveTime(double);
		int driveTimeNB(double);
		int drivexy(double, double, double, double);
		int drivexyNB(double, double, double, double);
		virtual int drivexyTo(double, double, double, double);
		int drivexyToNB(double, double, double, double);
		int drivexyToFunc(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncNB(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncSmooth(double, double, int, double (*func)(double), double, double);
		int drivexyToPoly(double, double, int, char*, double, double);
		int drivexyToPolyNB(double, double, int, char*, double, double);
		virtual int drivexyToSmooth(double, double, double, double, double, double, double, double);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getAccelerometerData(double&, double&, double&);
		int getBatteryVoltage(double&);
		int getDistance(double&, double);
		int getFormFactor(int&);
		int getID(void);
		int getJointAngle(robotJointId_t, double&, int = 10);
		int getJointAngleInstant(robotJointId_t, double&);
		int getJointMaxSpeed(robotJointId_t, double&);
		int getJointSafetyAngle(double&);
		int getJointSafetyAngleTimeout(double&);
		int getJointSpeed(robotJointId_t, double&);
		int getJointSpeedRatio(robotJointId_t, double&);
		int getLEDColorName(char[]);
		int getLEDColorRGB(int&, int&, int&);
		int getxy(double&, double&);
		int holdJoint(robotJointId_t);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int moveForeverNB(void);
		int moveJoint(robotJointId_t, double);
		int moveJointNB(robotJointId_t, double);
		int moveJointByPowerNB(robotJointId_t, int);
		int moveJointForeverNB(robotJointId_t);
		int moveJointTime(robotJointId_t, double);
		int moveJointTimeNB(robotJointId_t, double);
		int moveJointTo(robotJointId_t, double);
		int moveJointToNB(robotJointId_t, double);
		int moveJointToByTrackPos(robotJointId_t, double);
		int moveJointToByTrackPosNB(robotJointId_t, double);
		int moveJointWait(robotJointId_t);
		int moveTime(double);
		int moveTimeNB(double);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngle(robotJointId_t, double[], double[], int, double, int = 1);
		int recordAngleBegin(robotJointId_t, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordAngleEnd(robotJointId_t, int&);
		int recordAnglesEnd(int&);
		int recordDistanceBegin(robotJointId_t, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int recordDistanceEnd(robotJointId_t, int&);
		int recordDistanceOffset(double);
		int recordDistancesEnd(int&);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordxyEnd(int&);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int, double);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int);
		int setLEDColor(char*);
		int setLEDColorRGB(int, int, int);
		int setJointSafetyAngle(double);
		int setJointSafetyAngleTimeout(double);
		int setJointSpeed(robotJointId_t, double);
		int setJointSpeedRatio(robotJointId_t, double);
		int setSpeed(double, double);
		int systemTime(double&);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double, double, double);
		virtual int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		virtual int turnRightNB(double, double, double);

	// condensed argument versions of function calls
	protected:
		int moveNB(double*);
		int moveToNB(double*);
		int recordAngles(double*, double**, int, double, int);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t*&, double, int = 1);

	// utility functions for inherited and friend classes
	protected:
		int addToSim(dWorldID&, dSpaceID&, int, int, RoboSim*);
		double convert(double, int);
		int doze(double);
		int fixBodyToGround(dBodyID);
		dBodyID getBodyID(int);
		double getCenter(int);
		double getRotation(int, int);
		double mod_angle(double, double, double);
		int noisy(double*, int, double);
		static void* simPreCollisionThreadEntry(void*);
		static void* simPostCollisionThreadEntry(void*);

	// virual functions for inherited classes
	protected:
		virtual int build(XMLRobot*, int = 0) { return 0; };
		virtual int buildIndividual(double, double, double, dMatrix3, double*) { return 0; };
		virtual double getAngle(int) { return 0; };
		virtual int initParams(int, int) { return 0; };
		virtual int initDims(void) { return 0; };
		virtual void simPreCollisionThread(void) { return; };
		virtual void simPostCollisionThread(void) { return; };

	// data members
	protected:
		// motor motion directions
		typedef enum motor_state_e {
			NEUTRAL = 0,
			HOLD,
			POSITIVE,
			NEGATIVE,
		} motorState_t;
		// motor motion profiles
		typedef enum motor_mode_e {
			ACCEL_CONSTANT = 0,
			ACCEL_CYCLOIDAL,
			ACCEL_HARMONIC,
			CONTINUOUS,
			SEEK,
		} motorMode_t;

		// recording
		struct Recording {
			Robot *robot;			// robot
			robotJointId_t id;		// joint to record
			int num;				// number of points
			int msecs;				// ms between data points
			double *time;			// array for time
			double **ptime;			// pointer to time array
			double **angle;			// array of angles
			double ***pangle;		// point to array of angles
		};
		// motor accelerations
		struct Accel {
			double init;			// motion initial angle
			double start;			// motion start time
			double period;			// motion period
			double run;				// number of motions
		};
		// motor
		struct Motor {
			bool success;			// trigger for motion completion
			bool record;			// recording in progress
			bool record_active;		// actively recording a new point
			dJointID id;			// motors
			double alpha;			// angular acceleration
			double encoder;			// encoder resolution
			double goal;			// goal theta value
			double offset;			// offset from zero for resetting
			double omega;			// angular rate
			double omega_max;		// maximum rate
			double **record_angle;	// recording angles from thread
			double safety_angle;	// safety angle
			double safety_timeout;	// safety timeout
			double tau_max;			// maximum force
			double theta;			// theta
			int mode;				// modes
			int record_num;			// recording data points
			int starting;			// starting movement
			int state;				// state
			int stopping;			// stopping movement
			int timeout;			// mode timeout
			Accel accel;			// acceleration variables
			MUTEX_T success_mutex;	// motion successful mutex
			COND_T success_cond;	// motion successful condition
		};

		dBodyID *_body;				// body parts
		dGeomID **_geom;			// geometries of each body part
		dJointID *_joint;			// joints between body parts
		dSpaceID _space;			// space for this robot
		dWorldID _world;			// world for all robots
		RoboSim *_sim;				// simulation instance
		std::vector<Motor> _motor;	// motors
		std::vector<Vec3> _offset;	// body position offsets from center
		bool _motion;				// motion in progress
		double _accel[3];			// accelerometer data
		double _body_length;		// dimension: body length
		double _body_height;		// dimension: body height
		double _body_radius;		// dimension: body radius
		double _body_width;			// dimension: body width
		double _center[3];			// offset of body from initial (x,y,z)
		double _distOffset;			// offset for recorded distance
		double _radius;				// wheel radius
		double _rgb[3];				// rgb of 'led'
		double _speed;				// linear velocity of the robot
		double _trackwidth;			// trackwidth of robot
		double _wheel_depth;		// dimension: wheel depth
		double _wheel_radius;		// dimension: custom wheel radius
		int _connected;				// connected to controller
		int _disabled;				// which joint is disabled
		int _dof;					// number of DOF
		int *_enabled;				// list of enabled motors
		int _id;					// robot id
		int _pos;					// position in simulation
		int _seed;					// seed for random number generation
		int _shift_data;			// shift recorded data or not
		int _g_shift_data;			// globally shift data for robot
		int _g_shift_data_en;		// globally shift data for robot enable/disable flag
		int _trace;					// tracing on or off
		int _type;					// type of robot
		MUTEX_T _active_mutex;		// active recording
		COND_T _active_cond;		// active recording
		MUTEX_T _goal_mutex;		// goal value being written
		MUTEX_T _motion_mutex;		// motion in progress
		COND_T _motion_cond;		// motion in progress
		MUTEX_T _recording_mutex;	// recording data point
		COND_T _recording_cond;		// recording data  point
		MUTEX_T _success_mutex;		// completed step
		COND_T _success_cond;		// completed step
		MUTEX_T _theta_mutex;		// theta value being written

	// private functions
	private:
		bool is_shift_enabled(void);
		double normal(double);
		double uniform(void);
		static void* driveTimeNBThread(void*);
		static void* drivexyToThread(void*);
		static void* drivexyToFuncThread(void*);
		static void* drivexyToPolyThread(void*);
		static void* moveJointTimeNBThread(void*);
		static void* moveTimeNBThread(void*);
		static void* recordAngleThread(void*);
		static void* recordAngleBeginThread(void*);
		static void* recordAnglesThread(void*);
		static void* recordAnglesBeginThread(void*);
		static void* recordxyBeginThread(void*);

	// private data
	private:
		robotJointId_t _leftWheel;		// joint for left wheel
		robotJointId_t _rightWheel;		// joint for right wheel
};

template<class T> class Group {
	// public api
	public:
		Group(void) {};
		virtual ~Group(void) {};
		inline virtual int addRobot(T&);
		inline virtual int addRobots(T[], int);

		inline int blinkLED(double, int);
		inline int connect(void);
		inline int disconnect(void);
		inline int driveBackward(double);
		inline int driveBackwardNB(double);
		inline int driveDistance(double, double);
		inline int driveDistanceNB(double, double);
		inline int driveForever(void);
		inline int driveForeverNB(void);
		inline int driveForward(double);
		inline int driveForwardNB(double);
		inline int driveTime(double);
		inline int driveTimeNB(double);
		inline int holdJoint(robotJointId_t);
		inline int holdJoints(void);
		inline int holdJointsAtExit(void);
		inline int isMoving(void);
		inline int isNotMoving(void);
		inline int moveForeverNB(void);
		inline int moveJoint(robotJointId_t, double);
		inline int moveJointNB(robotJointId_t, double);
		inline int moveJointByPowerNB(robotJointId_t, int);
		inline int moveJointForeverNB(robotJointId_t);
		inline int moveJointTime(robotJointId_t, double);
		inline int moveJointTimeNB(robotJointId_t, double);
		inline int moveJointTo(robotJointId_t, double);
		inline int moveJointToNB(robotJointId_t, double);
		inline int moveJointToByTrackPos(robotJointId_t, double);
		inline int moveJointToByTrackPosNB(robotJointId_t, double);
		inline int moveJointWait(robotJointId_t);
		inline int moveTime(double);
		inline int moveTimeNB(double);
		inline int moveToZero(void);
		inline int moveToZeroNB(void);
		inline int moveWait(void);
		inline int relaxJoint(robotJointId_t id);
		inline int relaxJoints(void);
		inline int resetToZero(void);
		inline int resetToZeroNB(void);
		inline int setBuzzerFrequency(int, double);
		inline int setBuzzerFrequencyOff(void);
		inline int setBuzzerFrequencyOn(int);
		inline int setLEDColor(char*);
		inline int setLEDColorRGB(int, int, int);
		inline int setJointSafetyAngle(double);
		inline int setJointSafetyAngleTimeout(double);
		inline int setJointSpeed(robotJointId_t, double);
		inline int setJointSpeedRatio(robotJointId_t, double);
		inline int setSpeed(double, double);
		inline int traceOff(void);
		inline int traceOn(void);
		inline int turnLeft(double, double, double);
		inline int turnLeftNB(double, double, double);
		inline int turnRight(double, double, double);
		inline int turnRightNB(double, double, double);

	// data members
	protected:
		std::vector<T*> _robots;
		double _d;
		int _i;
};
class DLLIMPORT RobotGroup : public Group<Robot> {};
#include "robotgroup.tpp"

// motion threading
struct RobotMove {
	Robot *robot;
	char *expr;
	double x, y, radius, trackwidth;
	double (*func)(double x);
	int i;
};

// simulation
//extern RoboSim *g_sim;

#endif // ROBOT_HPP_

