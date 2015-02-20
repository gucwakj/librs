#ifndef RSSIM_ROBOT_HPP_
#define RSSIM_ROBOT_HPP_

#include <ode/ode.h>

#include <rs/macros.hpp>
#include <rsRobots/robot.hpp>

namespace rsSim {

	// forward declarations
	class Sim;

	// typedefs
	typedef std::vector<dBodyID> BodyList;

	// class
	class Robot : virtual public rsRobots::Robot {
		// public functions
		public:
			Robot(int, int);
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
			int getJointAngle(int, double&, int = 10);
			int getJointAngleInstant(int, double&);
			int getJointMaxSpeed(int, double&);
			int getJointSafetyAngle(double&);
			int getJointSafetyAngleTimeout(double&);
			int getJointSpeed(int, double&);
			int getJointSpeedRatio(int, double&);
			int getLEDColorName(char[]);
			int getLEDColorRGB(int&, int&, int&);
			int getxy(double&, double&);
			int holdJoint(int);
			int holdJoints(void);
			int holdJointsAtExit(void);
			int isConnected(void);
			int isMoving(void);
			int isNotMoving(void);
			int moveForeverNB(void);
			int moveJoint(int, double);
			int moveJointNB(int, double);
			int moveJointByPowerNB(int, int);
			int moveJointForeverNB(int);
			int moveJointTime(int, double);
			int moveJointTimeNB(int, double);
			int moveJointTo(int, double);
			int moveJointToNB(int, double);
			int moveJointToByTrackPos(int, double);
			int moveJointToByTrackPosNB(int, double);
			int moveJointWait(int);
			int moveTime(double);
			int moveTimeNB(double);
			int moveToZero(void);
			int moveToZeroNB(void);
			int moveWait(void);
			int recordAngle(int, double[], double[], int, double, int = 1);
			int recordAngleBegin(int, robotRecordData_t&, robotRecordData_t&, double, int = 1);
			int recordAngleEnd(int, int&);
			int recordAnglesEnd(int&);
			int recordDistanceBegin(int, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
			int recordDistanceEnd(int, int&);
			int recordDistanceOffset(double);
			int recordDistancesEnd(int&);
			int recordWait(void);
			int recordxyBegin(robotRecordData_t&, robotRecordData_t&, double, int = 1);
			int recordxyEnd(int&);
			int relaxJoint(int id);
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
			int setJointSpeed(int, double);
			int setJointSpeedRatio(int, double);
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

		public:
			BodyList& getBodyList(void);
			double getCenter(int);
			const double* getPosition(void);
			const double* getQuaternion(void);
			virtual void getCoM(double&, double&, double&) { return; };

		// utility functions
		protected:
			int addToSim(dWorldID&, dSpaceID&, int, int, rsSim::Sim*);
			double convert(double, int);
			int doze(double);
			int fixBodyToGround(dBodyID);
			dBodyID getBodyID(int);
			double getRotation(int, int);
			double mod_angle(double, double, double);
			int noisy(double*, int, double);
			static void* simPreCollisionThreadEntry(void*);
			static void* simPostCollisionThreadEntry(void*);

		// virual functions for inherited classes
		protected:
			virtual int build(int, const double*, const double*, const double*, int) { return 0; };
			virtual int buildIndividual(const double*, const double*, const double*) { return 0; };
			virtual double getAngle(int) { return 0; };
			virtual void init_params(void) { return; };
			virtual void simPreCollisionThread(void) { return; };
			virtual void simPostCollisionThread(void) { return; };

		// data members
		protected:
			// motor motion directions
			typedef enum motor_state_e {
				NEUTRAL,
				HOLD,
				POSITIVE,
				NEGATIVE,
			} motorState_t;
			// motor motion profiles
			typedef enum motor_mode_e {
				ACCEL_CONSTANT,
				ACCEL_CYCLOIDAL,
				ACCEL_HARMONIC,
				CONTINUOUS,
				SEEK,
			} motorMode_t;

			// recording
			struct Recording {
				Robot *robot;		// robot
				int id;		// joint to record
				int num;			// number of points
				int msecs;			// ms between data points
				double *time;		// array for time
				double **ptime;		// pointer to time array
				double **angle;		// array of angles
				double ***pangle;	// point to array of angles
			};
			// motor accelerations
			struct Accel {
				double init;		// motion initial angle
				double start;		// motion start time
				double period;		// motion period
				double run;			// number of motions
			};
			// motor
			struct Motor {
				bool success;			// trigger for motion completion
				bool record;			// recording in progress
				bool record_active;		// actively recording a new point
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
				dJointID id;			// motor id
				dJointID joint;			// joint of the motor
				Accel accel;			// acceleration variables
				MUTEX_T success_mutex;	// motion successful mutex
				COND_T success_cond;	// motion successful condition
			};

			dSpaceID _space;			// space for this robot
			dWorldID _world;			// world for all robots
			rsSim::Sim *_sim;			// simulation instance
			std::vector<Motor> _motor;	// motors
			BodyList _body;				// body parts
			bool _motion;				// motion in progress
			double _accel[3];			// accelerometer data
			double _center[3];			// offset of body from initial (x,y,z)
			double _distOffset;			// offset for recorded distance
			double _radius;				// radius of wheels
			double _speed;				// linear velocity of the robot
			double _trackwidth;			// trackwidth of robot
			int _connected;				// connected to controller
			int _dof;					// number of DOF
			int _id;					// robot id
			int _pos;					// position in simulation
			int _seed;					// seed for random number generation
			int _shift_data;			// shift recorded data or not
			int _g_shift_data;			// globally shift data for robot
			int _g_shift_data_en;		// globally shift data for robot enable/disable flag
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
			int _leftWheel;		// joint for left wheel
			int _rightWheel;	// joint for right wheel
	};

	// motion threading
	struct RobotMove {
		rsSim::Robot *robot;
		char *expr;
		double x, y, radius, trackwidth;
		double (*func)(double x);
		int i;
	};

} // namespace rsSim

/*
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
		inline int holdJoint(int);
		inline int holdJoints(void);
		inline int holdJointsAtExit(void);
		inline int isMoving(void);
		inline int isNotMoving(void);
		inline int moveForeverNB(void);
		inline int moveJoint(int, double);
		inline int moveJointNB(int, double);
		inline int moveJointByPowerNB(int, int);
		inline int moveJointForeverNB(int);
		inline int moveJointTime(int, double);
		inline int moveJointTimeNB(int, double);
		inline int moveJointTo(int, double);
		inline int moveJointToNB(int, double);
		inline int moveJointToByTrackPos(int, double);
		inline int moveJointToByTrackPosNB(int, double);
		inline int moveJointWait(int);
		inline int moveTime(double);
		inline int moveTimeNB(double);
		inline int moveToZero(void);
		inline int moveToZeroNB(void);
		inline int moveWait(void);
		inline int relaxJoint(int id);
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
		inline int setJointSpeed(int, double);
		inline int setJointSpeedRatio(int, double);
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
class DLLIMPORT RobotGroup : public Group<rsSim::Robot> {};
#include "robotgroup.tpp"
*/

#endif // RSSIM_ROBOT_HPP_

