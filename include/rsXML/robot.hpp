#ifndef RSXML_ROBOT_HPP_
#define RSXML_ROBOT_HPP_

#include <cmath>
#include <iostream>
#include <vector>

#include <rs/enum.hpp>
#include <rs/macros.hpp>
#include <rsRobots/linkbot.hpp>
#include <rsXML/conn.hpp>

namespace rsXML {

	// classes
	class Robot : virtual public rsRobots::Robot {
		// public functions
		public:
			Robot(bool);
			virtual ~Robot(void);

			int addConnector(Conn*);
			int getConnect(void);
			int getGround(void);
			int getID(void);
			double* getJoints(void);
			double* getPosition(void);
			double* getQuaternion(void);
			bool getTrace(void);
			virtual void postProcess(void) {};
			void printDebug(void);
			void setConnect(int);
			void setGround(int);
			void setID(int);
			void setJoints(double = 0, double = 0, double = 0, double = 0, double = 0, double = 0);
			void setPosition(double, double, double);
			void setPsi(double);
			void setRotation(double, double, double);

		// data
		protected:
			std::vector<Conn*> _conn;
			double _a[6];
			double _p[3];
			double _q[4];
			int _connected;
			int _ground;
			int _id;
			int _trace;
	};
	class LinkbotT : virtual public rsRobots::LinkbotT, public Robot {
		public:
			LinkbotT(bool trace) :  rsRobots::Robot(rs::LINKBOTT), rsXML::Robot(trace) {};
			void postProcess(void);
	};
	class LinkbotI : virtual public rsRobots::LinkbotI, public LinkbotT {
		public:
			LinkbotI(bool trace) : rsRobots::Robot(rs::LINKBOTI), rsXML::LinkbotT(trace) {};
	};
	class LinkbotL : virtual public rsRobots::LinkbotL, public LinkbotT {
		public:
			LinkbotL(bool trace) : rsRobots::Robot(rs::LINKBOTL), rsXML::LinkbotT(trace) {};
	};

} // namespace rsXML

#endif // RSXML_ROBOT_HPP_

