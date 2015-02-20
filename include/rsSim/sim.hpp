#ifndef RSSIM_SIM_HPP_
#define RSSIM_SIM_HPP_

#include <vector>

#include <rs/enum.hpp>
#include <rsSim/modularRobot.hpp>
#include <rsSim/robot.hpp>

namespace rsSim {

	// typedefs
	typedef dBodyID Ground;

	// class
	class Sim {
		// public functions
		public:
			Sim(int);
			virtual ~Sim(void);

			//int addRobot(Robot*);
			int addRobot(ModularRobot*, int, const double*, const double*, const double*, int, int);
			int addRobot(ModularRobot*, int, Robot*, const double*, int, int, int, int, int, int);
			Ground* addGround(const double*, const double*, const double*, double);
			Ground* addGround(const double*, const double*, const double*, double, int);
			Ground* addGround(const double*, const double*, double);
			virtual int deleteRobot(int);
			void done(void);
			double getClock(void);
			int getCOR(double&, double&);
			void getCoM(double&, double&, double&);
			int getMu(double&, double&);
			bool getPause(void);
			Robot* getRobot(int);
			double getStep(void);
			int runSimulation(void);
			int setCollisions(int);
			int setCOR(double, double);
			int setMu(double, double);
			int setPause(int);

		// protected data
		protected:
			struct RobotNode {
				rsSim::Robot *robot;
				int node;
				THREAD_T thread;
			};

			bool _collision;					// flag: perform collisions
			bool _running;						// flag: simulation running
			bool _rt;							// flag: run in real time
			bool _pause;						// flag: paused
			double _clock;						// clock time of simulation
			double _friction[2];				// coefficient of friction [body/ground, body/body]
			double _restitution[2];				// coefficient of restitution [body/ground, body/body]
			double _step;						// time step of simulation
			dWorldID _world;					// ode: world
			dSpaceID _space;					// ode: space for robots
			dJointGroupID _group;				// ode: joint group
			std::vector<RobotNode*> _robot;		// list: robots
			COND_T _running_cond;				// condition: actively running simulation
			MUTEX_T _clock_mutex;				// mutex: getting the clock
			MUTEX_T _pause_mutex;				// mutex: paused simulation
			MUTEX_T _robot_mutex;				// mutex: ground collisions
			MUTEX_T _running_mutex;				// mutex: actively running program
			MUTEX_T _step_mutex;				// mutex: getting the step value
			THREAD_T _simulation;				// thread: simulation

		// private functions
		private:
			static void collision(void*, dGeomID, dGeomID);	// wrapper function for nearCallback to work in class
			static void* simulation_thread(void*);			// simulation thread function
	};

} // namespace rsSim

#endif // RSSIM_SIM_HPP_

