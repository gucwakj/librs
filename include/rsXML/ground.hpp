#ifndef RSXML_GROUND_HPP_
#define RSXML_GROUND_HPP_

#include <cmath>

#include <rs/enum.hpp>

namespace rsXML {

	class Ground {
		// public functions
		public:
			Ground(int);
			virtual ~Ground(void) {};

			double getAxis(void);
			double* getColor(void);
			double* getDimensions(void);
			double getMass(void);
			double* getPosition(void);
			double* getQuaternion(void);
			int getType(void);
			void setAxis(double);
			void setColor(double, double, double, double);
			void setDimensions(double, double, double);
			void setMass(double);
			void setPosition(double, double, double);
			void setRotation(double, double, double);

		// private data
		private:
			double _axis;	// longitudinal axis
			double _c[4];	// color
			double _l[3];	// lengths
			double _mass;	// mass
			double _p[3];	// position
			double _q[4];	// quaternion
			int _type;		// type
	};

} // namespace rsXML

#endif // RSXML_GROUND_HPP_

