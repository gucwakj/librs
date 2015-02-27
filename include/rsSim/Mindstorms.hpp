#ifndef RSSIM_MINDSTORMS_HPP_
#define RSSIM_MINDSTORMS_HPP_

#include <rsRobots/Mindstorms>
#include <rsSim/modularRobot.hpp>

namespace rsSim {

	// class
	class Mindstorms : virtual public rsRobots::Mindstorms, virtual public Robot {
			friend class Sim;
		// public api
		public:
			Mindstorms(void);
			virtual ~Mindstorms(void);

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
			virtual int build(const double*, const double*, const double*, int);
			virtual int buildIndividual(const double*, const double*, const double*);
			virtual double getAngle(int);
			virtual void init_params(void);
			virtual void simPreCollisionThread(void);
			virtual void simPostCollisionThread(void);

		// private functions
		private:
			void build_body(dGeomID*, const double*, const double*);
			void build_wheel(int, dGeomID*, const double*, const double*);
	};

} // namespace rsSim

#endif // RSSIM_MINDSTORMS_HPP

