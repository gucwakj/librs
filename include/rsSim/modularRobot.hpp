#ifndef RSSIM_MODULARROBOT_HPP_
#define RSSIM_MODULARROBOT_HPP_

#include <rsSim/robot.hpp>

namespace rsXML {
	class Conn;
}

namespace rsSim {

// connector
struct Connector {
	dBodyID body;
	dGeomID *geom;
	double o[3];
	int face, type;
	int d_side, d_type;
};

class ModularRobot : virtual public rsSim::Robot {
		friend class Sim;
	public:
		ModularRobot(void);
		virtual ~ModularRobot(void);

		int connect(char* = NULL, int = 3);

	public:
		std::vector<Connector*>& getConnectorList(void);
	
	// utility functions for inherited and friend classes
	protected:
		int addNeighbor(ModularRobot*, int, int);
		int addSensor(int, int);
		static void collideSensor(void*, dGeomID, dGeomID);
		dBodyID getConnectorBodyID(int);
		int getNeighborCount(int = -1, int = 0);
		double getNeighborForce(int, int = 0);
		double getNeighborTorque(int, int = 0);

	// virual functions for inherited classes
	protected:
		virtual int addConnector(int, int, double) = 0;
		virtual int build(rsXML::Robot*, dMatrix3, double*, dBodyID, rsXML::Conn*) = 0;
		virtual int fixBodyToConnector(dBodyID, int) = 0;
		virtual int fixConnectorToBody(int, dBodyID, int = -1) = 0;
		virtual int getConnectorParams(int, int, dMatrix3, double*) = 0;
		virtual int getFaceParams(int, dMatrix3, double*) = 0;

	// virtual functions from Robot class
	protected:
		virtual int build(int, const double*, const double*, const double*, int) { return 0; };
		virtual int buildIndividual(double, double, double, dMatrix3, double*) { return 0; };
		virtual double getAngle(int) { return 0; };
		virtual int initParams(int, int) { return 0; };
		virtual int initDims(void) { return 0; };
		virtual void simPreCollisionThread(void) { return; };
		virtual void simPostCollisionThread(void) { return; };

	// data members
	protected:
		// neighbors
		struct Neighbor {
			ModularRobot *robot;
			int face;
		};
		// sensors
		struct Sensor {
			dSpaceID space;
			dBodyID body;
			dGeomID geom;
			int face;
			int type;
		};

		std::vector<Connector*> _conn;		// connectors
		std::vector<Neighbor> _neighbor;	// connected robots
		std::vector<Sensor*> _sensor;		// connected sensors
		std::vector<dJointFeedback*> _fb;	// feedback forces from joints
		//double _bigwheel_radius;			// dimension: big wheel radius
		//double _conn_depth;					// dimension: connector depth
		//double _conn_height;				// dimension: connector height
		//double _conn_radius;				// dimension: connector radius
		//double _smallwheel_radius;			// dimension: small wheel radius
};

} // namepsace rsSim

#endif // RSSIM_MODULARROBOT_HPP_

