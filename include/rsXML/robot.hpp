#ifndef RSXML_ROBOT_HPP_
#define RSXML_ROBOT_HPP_

#include <cmath>
#include <iostream>
#include <vector>

#include <rs/enum.hpp>
#include <rs/macros.hpp>
#include <rsRobots/linkbot.hpp>
#include <rsRobots/Mindstorms.hpp>
#include <rsXML/conn.hpp>

namespace rsXML {

	// typedefs
	typedef std::vector<Conn*> ConnectorList;

	// classes
	class Robot : virtual public rsRobots::Robot {
		// public functions
		public:
			Robot(bool);
			virtual ~Robot(void);

			int addConnector(Conn*);
			Conn* getBaseConnector(void);
			ConnectorList& getConnectorList(void);
			int getConnect(void);
			int getGround(void);
			int getID(void);
			double* getJoints(void);
			double* getLED(void);
			double* getPosition(void);
			double* getQuaternion(void);
			bool getTrace(void);
			virtual void postProcess(void) {};
			void printDebug(void);
			void setConnect(int);
			void setGround(int);
			void setID(int);
			void setJoints(double = 0, double = 0, double = 0, double = 0, double = 0, double = 0);
			void setLED(double, double, double, double);
			void setPosition(double, double, double);
			void setPsi(double);
			void setRotation(double, double, double);

		// data
		protected:
			double _a[6];
			double _p[3];
			double _q[4];
			double _led[4];
			int _base;
			int _connected;
			int _ground;
			int _id;
			int _trace;
			ConnectorList _conn;
	};
	class Linkbot : virtual public rsRobots::Linkbot, virtual public Robot {
		public:
			Linkbot(void) :  rsRobots::Robot(rs::LINKBOTT), rsXML::Robot(1) {};
			void postProcess(void);
	};
	class LinkbotI : public Linkbot {
		public:
			LinkbotI(bool trace) : rsRobots::Robot(rs::LINKBOTI), rsXML::Robot(trace) {};
	};
	class LinkbotL : public Linkbot {
		public:
			LinkbotL(bool trace) : rsRobots::Robot(rs::LINKBOTL), rsXML::Robot(trace) {};
	};
	class LinkbotT : public Linkbot {
		public:
			LinkbotT(bool trace) : rsRobots::Robot(rs::LINKBOTT), rsXML::Robot(trace) {};
	};
	class Mindstorms : virtual public rsRobots::Mindstorms, virtual public Robot {
		public:
			Mindstorms(bool trace) :  rsRobots::Robot(rs::MINDSTORMS), rsXML::Robot(trace) {};
			void postProcess(void);
	};

} // namespace rsXML

#endif // RSXML_ROBOT_HPP_

