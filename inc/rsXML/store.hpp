#ifndef RSXML_STORE_HPP_
#define RSXML_STORE_HPP_

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <tinyxml2.h>

#include <rs/enum.hpp>
#include <rsXML/robot.hpp>

namespace rsXML {

	class Store {
		// public functions
		public:
			Store(char*);
			virtual ~Store(void);

			int addNewRobot(Robot*);
			Robot* getNextRobot(int);
			int getNumGrounds(void);
			int getNumMarkers(void);

		// private functions
		private:
			void load_file(char*, tinyxml2::XMLDocument*);
			void read_config(tinyxml2::XMLDocument*);
			void read_graphics(tinyxml2::XMLDocument*);
			void read_ground(tinyxml2::XMLDocument*);
			void read_sim(tinyxml2::XMLDocument*);

		// private data
		private:
			struct Marker {
				double c[4];	// color
				double p[6];	// position (start and end)
				int size;		// size of object
				int type;		// type
				std::string s;	// label
			};
			struct Ground {
				double c[4];	// color
				double l[3];	// lengths
				double p[3];	// position
				double r[3];	// rotation
				double axis;	// longitudinal axis
				double mass;	// mass
				int type;		// type
			};

			std::vector<Marker*> _marker;		// markers
			std::vector<Ground*> _ground;		// ground obstacles
			std::vector<Robot*> _robot;			// robots

			double _cor[2];						// coefficient of restitution [body/ground, body/body]
			double _mu[2];						// coefficient of friction [body/ground, body/body]
			double _grid[7];					// grid spacing (tics, major, total)
			int _ps;							// is the simulation paused
			int _preconfig;						// preconfigured robot shape or not
			int _rt;							// whether to run at real time speeds
			int _trace;							// tracking of robots
			int _us;							// us customary units
	};

} // namespace rsXML

#endif // RSXML_STORE_HPP_

