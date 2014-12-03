#ifndef RSXML_ROBOT_HPP_
#define RSXML_ROBOT_HPP_

#include <vector>
#include <cmath>

#include <rs/enum.hpp>
#include <rsXML/conn.hpp>

namespace rsXML {

	class Robot {
		public:
			Robot(void);
			Robot(int, int);
			virtual ~Robot(void);

			int addConnector(Conn*);
			int getConnect(void);
			int getForm(void);
			int getGround(void);
			int getID(void);
			double* getJoints(void);
			double* getPosition(void);
			double* getQuaternion(void);
			int getTrace(void);
			void setConnect(int);
			void setGround(int);
			void setID(int);
			void setJoints(double = 0, double = 0, double = 0, double = 0, double = 0, double = 0);
			void setPosition(double, double, double);
			void setPsi(double);
			void setRotation(double, double, double);

		private:
			std::vector<Conn*> _conn;
			double _p[3];
			double _q[4];
			double _a[6];
			int _connected;
			int _ground;
			int _id;
			int _form;
			int _trace;
	};

} // namespace rsXML

#endif // RSXML_ROBOT_HPP_

