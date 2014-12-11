#ifndef RSXML_MARKER_HPP_
#define RSXML_MARKER_HPP_

#include <string>

#include <rs/enum.hpp>

namespace rsXML {

	class Marker {
		// public functions
		public:
			Marker(int);
			virtual ~Marker(void) {};

			double* getColor(void);
			double* getEnd(void);
			double* getStart(void);
			int getSize(void);
			int getType(void);
			std::string getLabel(void);
			void setColor(double, double, double, double);
			void setEnd(double, double, double);
			void setLabel(std::string);
			void setSize(int);
			void setStart(double, double, double);

		// private data
		private:
			double _c[4];	// color
			double _e[3];	// end position
			double _s[3];	// start position
			int _size;		// size of object
			int _type;		// type
			std::string _l;	// label
	};

} // namespace rsXML

#endif // RSXML_MARKER_HPP_
