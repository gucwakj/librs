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
			std::vector<double> getGrid(void);
			int getNumGrounds(void);
			int getNumMarkers(void);
			bool getPause(void);
			bool getRealTime(void);
			bool getUnits(void);

		// private functions
		private:
			void load_file(char*, tinyxml2::XMLDocument*);
			void read_config(tinyxml2::XMLDocument*);
			void read_graphics(tinyxml2::XMLDocument*);
			void read_ground(tinyxml2::XMLDocument*);
			void read_sim(tinyxml2::XMLDocument*);

		// private data
		private:
			struct Ground {
				double axis;	// longitudinal axis
				double c[4];	// color
				double l[3];	// lengths
				double mass;	// mass
				double p[3];	// position
				double r[3];	// rotation
				int type;		// type
			};
			struct Marker {
				double c[4];	// color
				double p[6];	// position (start and end)
				int size;		// size of object
				int type;		// type
				std::string s;	// label
			};

			bool _pause;					// flag: start in a paused state
			bool _preconfig;				// flag: preconfigured robot shape
			bool _trace;					// flag: trace robot positions
			bool _rt;						// flag: real time motion
			bool _units;					// flag: SI (true) or customary (false)
			double _cor[2];					// coefficient of restitution [body/ground, body/body]
			double _mu[2];					// coefficient of friction [body/ground, body/body]
			std::vector<double> _grid;		// grid
			std::vector<Ground*> _ground;	// ground obstacles
			std::vector<Marker*> _marker;	// markers
			std::vector<Robot*> _robot;		// robots
	};

} // namespace rsXML

#endif // RSXML_STORE_HPP_

