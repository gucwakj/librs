#ifndef RSXML_CONN_HPP_
#define RSXML_CONN_HPP_

#include <rs/enum.hpp>

namespace rsXML {

	class Conn {
		// public functions
		public:
			Conn(void);
			Conn(double, int, int, int, int, int, int);
			virtual ~Conn(void) {};

		// private data
		private:
			double _size;
			int _conn;
			int _face1;
			int _face2;
			int _robot;
			int _side;
			int _type;
	};

} // namespace rsXML

#endif // RSXML_STORE_HPP_

