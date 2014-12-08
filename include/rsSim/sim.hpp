#ifndef RSSIM_SIM_HPP_
#define RSSIM_SIM_HPP_

#include <iostream>
#include <vector>

#include <ode/ode.h>

#include <rs/macros.hpp>
#include <rs/enum.hpp>
#include <rsSim/modularRobot.hpp>
#include <rsSim/robot.hpp>

namespace rsSim {

	typedef dBodyID Ground;

} // namespace rsSim

namespace rsSim {

	class Sim {
		// public functions
		public:
			Sim(int);
			virtual ~Sim(void);

			//int addRobot(rsSim::Robot*);
			//int addRobot(rsSim::ModularRobot*);
			//int addRobot2(rsSim::ModularRobot*, rsXML::Robot*);
			//int addRobot2(rsSim::ModularRobot*, rsXML::Robot*, rsSim::ModularRobot*);
			int addRobot3(rsSim::ModularRobot*, int, const double*, const double*, const double*, int, int);
			Ground* addGround(const double*, const double*, const double*, double);
			Ground* addGround(const double*, const double*, const double*, double, int);
			Ground* addGround(const double*, const double*, double);
			virtual int deleteRobot(int);
			void done(void);
			double getClock(void);
			int getCOR(double&, double&);
			int getMu(double&, double&);
			int getPause(void);
			double getStep(void);
			int getUnits(void);
			int runSimulation(void);
			int setCollisions(int);
			int setCOR(double, double);
			int setMu(double, double);
			int setPause(int);

		// private functions
		private:
			int init_ode(void);								// init function for ode variables
			int init_sim(int);								// init function for simulation variables
			static void collision(void*, dGeomID, dGeomID);	// wrapper function for nearCallback to work in class
			static void* simulation_thread(void*);			// simulation thread function

		// private data
		protected:
			struct Robots {
				rsSim::Robot *robot;
				int node;
				THREAD_T thread;
			};

			std::vector<Robots*> _robot;	// robots

			dWorldID _world;				// world in which simulation occurs
			dSpaceID _space;				// space for robots in which to live
			dJointGroupID _group;			// group to store joints
			bool _collision;				// check to perform collisions
			double _clock;					// clock time of simulation
			double _cor[2];					// coefficient of restitution [body/ground, body/body]
			double _mu[2];					// coefficient of friction [body/ground, body/body]
			double _step;					// time of each step of simulation
			int _pause;						// is the simulation paused
			int _preconfig;					// preconfigured robot shape or not
			int _rt;						// whether to run at real time speeds
			int _running;					// is the program running
			int _us;						// us customary units
			COND_T _running_cond;			// condition for actively running program
			MUTEX_T _clock_mutex;			// mutex for getting the clock
			MUTEX_T _pause_mutex;			// mutex for paused simulation
			MUTEX_T _robot_mutex;			// mutex for ground collisions
			MUTEX_T _running_mutex;			// mutex for actively running program
			MUTEX_T _step_mutex;			// mutex for getting the step value
			THREAD_T _simulation;			// simulation thread
	};

} // namespace rsSim

#endif // RSSIM_SIM_HPP_

