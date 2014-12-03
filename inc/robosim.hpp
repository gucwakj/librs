#ifndef ROBOSIM_HPP_
#define ROBOSIM_HPP_

#include <iostream>
//#include <tinyxml2.h>

#include "robot.hpp"
#include "modularrobot.hpp"
#ifdef ENABLE_GRAPHICS
//#include "graphics.hpp"
#endif // ENABLE_GRAPHICS

// ground objects
struct Ground {
	dBodyID body;
	dGeomID geom;
	int type;
	double x, y, z;
	double q[4];
	double l1, l2, l3;
	double r, g, b, alpha;
};

class DLLIMPORT RoboSim {
	// public api
	public:
		RoboSim(char*, int, int = 1);
		virtual ~RoboSim(void);

		int addRobot(Robot*);
		int addRobot(ModularRobot*);
		int addRobot2(ModularRobot*, XMLRobot*);
		int addRobot2(ModularRobot*, XMLRobot*, ModularRobot*);
		int deleteRobot(int);
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
		int setLib(int);
		int setMu(double, double);
		int setPause(int);
		int xmlNewRobot(XMLRobot*);

	// private functions
	private:
		int init_ode(void);								// init function for ode variables
		int init_sim(int);								// init function for simulation variables
		//int init_xml(char*);							// init function to read xml config file
		static void collision(void*, dGeomID, dGeomID);	// wrapper function for nearCallback to work in class
		static void* simulation_thread(void*);			// simulation thread function

	// private data
	private:
		// robots
		struct Robots {
			Robot *robot;
			int node;
			THREAD_T thread;
		};

		dWorldID _world;				// world in which simulation occurs
		dSpaceID _space;				// space for robots in which to live
		dJointGroupID _group;			// group to store joints
#ifdef ENABLE_GRAPHICS
		//Graphics *_graphics;			// graphics object
#endif // ENABLE_GRAPHICS
		std::vector<Ground*> _ground;	// ground (static) objects
		std::vector<Robots*> _robots;	// all robots in simulation
		std::vector<XMLRobot*> _xmlbot;	// robots read from config file
		bool _collision;				// check to perform collisions
		double _clock;					// clock time of simulation
		double _cor[2];					// coefficient of restitution [body/ground, body/body]
		double _mu[2];					// coefficient of friction [body/ground, body/body]
		double _step;					// time of each step of simulation
		int _lib;						// called from lib
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

#endif	// ROBOSIM_HPP_

