#include <cmath>
#include <iostream>
#include <rs/Macros>
#include <rsXML/Reader>
#include <rsXML/Linkbot>
#include <rsXML/Mindstorms>

using namespace rsXML;

Reader::Reader(char *name) {
	// set default values
	_pause = true;
	_preconfig = false;
	_rt = true;
	_trace = false;
	_units = false;
	_version = 0;
	for (int i = 0; i < 2; i++) {
		_friction.push_back(0);
		_restitution.push_back(0);
	}
	for (int i = 0; i < 7; i++) {
		_grid.push_back(0);
	}

	// read XML file
	tinyxml2::XMLDocument doc;
	this->load_file(name, &doc);
	this->read_config(&doc);
	this->read_ground(&doc);
	this->read_graphics(&doc);
	this->read_sim(&doc);
}

Reader::~Reader(void) {
std::cerr << "~Reader start" << std::endl;
	for (int i = 0; i < _robot.size(); i++) {
		delete _robot[i];
	}
	for (int i = 0; i < _ground.size(); i++) {
		delete _ground[i];
	}
	for (int i = 0; i < _marker.size(); i++) {
		delete _marker[i];
	}
std::cerr << "~Reader end" << std::endl;
}

/**********************************************************
	public functions
 **********************************************************/
int Reader::addNewRobot(Robot *bot) {
	_robot.push_back(bot);
	return 0;
}

Ground* Reader::getGround(int id) {
	return _ground[id];
}

Marker* Reader::getMarker(int id) {
	return _marker[id];
}

Robot* Reader::getNextRobot(int form) {
	// find next robot in xml list
	int i = 0;
	for (i = 0; i < _robot.size(); i++) {
		if (!_robot[i]->getConnect() && (form == -1 || _robot[i]->getForm() == form))
			break;
	}

	// no robot found
	if (form != -1 && (i == _robot.size() || _robot[i]->getForm() != form)) {
		switch (form) {
			case rs::LINKBOTI:
				std::cerr << "Error: Could not find a Linkbot-I in the RoboSim GUI robot list." << std::endl;
				break;
			case rs::LINKBOTL:
				std::cerr << "Error: Could not find a Linkbot-L in the RoboSim GUI robot list." << std::endl;
				break;
			case rs::LINKBOTT:
				std::cerr << "Error: Could not find a Linkbot-T in the RoboSim GUI robot list." << std::endl;
				break;
			case rs::EV3: case rs::NXT:
				std::cerr << "Error: Could not find a Mindstorms EV3 or NXT in the RoboSim GUI robot list." << std::endl;
				break;
		}
		if (_preconfig) {
			fprintf(stderr, "       Preconfigured Robot Configuration selected.\n");
			fprintf(stderr, "       Please uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	if (i == _robot.size())
		return NULL;

	// robot now connected
	_robot[i]->setConnect(1);

	// success
	return _robot[i];
}

std::string Reader::getDoc(void) {
	return _path;
}

std::vector<double> Reader::getFriction(void) {
	return _friction;
}

std::vector<double> Reader::getGrid(void) {
	return _grid;
}

std::vector<double> Reader::getRestitution(void) {
	return _restitution;
}

int Reader::getNumGrounds(void) {
	return _ground.size();
}

int Reader::getNumMarkers(void) {
	return _marker.size();
}

int Reader::getNumRobots(void) {
	return _robot.size();
}

bool Reader::getPause(void) {
	return _pause;
}

bool Reader::getRealTime(void) {
	return _rt;
}

bool Reader::getTrace(void) {
	return _trace;
}

bool Reader::getUnits(void) {
	return _units;
}

/**********************************************************
	private functions
 **********************************************************/
void Reader::load_file(char *name, tinyxml2::XMLDocument *doc) {
#ifdef _WIN32
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path))) {
		_path.append("\\robosimrc");
	}
#else
	_path = getenv("HOME");
	if (name) {
		FILE *fp = fopen(name, "r");
		if (fp) {
			_path = name;
			fclose(fp);
		}
		else {
			_path.append("/.robosimrc");
		}
	}
	else {
		_path.append("/.robosimrc");
	}
#endif
	int output = doc->LoadFile(_path.c_str());
	if (output) {
		fprintf(stderr, "Error: Could not find RoboSim config file.\nPlease run RoboSim GUI.\n");
		exit(-1);
	}
}

void Reader::read_config(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;

	// check if should start paused
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("version")) ) {
		node->QueryIntText(&_version);
	}

	// check for custom mu params
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("mu")) ) {
		node->QueryDoubleAttribute("ground", &(_friction[0]));
		node->QueryDoubleAttribute("body", &(_friction[1]));
	}

	// check for custom cor params
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("cor")) ) {
		node->QueryDoubleAttribute("ground", &(_restitution[0]));
		node->QueryDoubleAttribute("body", &(_restitution[1]));
	}

	// check if should start paused
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("pause")) ) {
		node->QueryIntText(reinterpret_cast<int *>(&_pause));
	}

	// check if should run in real time
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("realtime")) ) {
		node->QueryIntText(reinterpret_cast<int *>(&_rt));
	}
}

void Reader::read_graphics(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	double a, b, c, d;

	// check for existence of node
	if ( (node = doc->FirstChildElement("graphics")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if ( !strcmp(node->Value(), "line") ) {
			// create object
			_marker.push_back(new Marker(rs::LINE));
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// end position
			if ( (ele = node->FirstChildElement("end")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_marker.back()->setEnd(a, b, c);
			}
			// start position
			if ( (ele = node->FirstChildElement("start")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			if (!node->QueryDoubleAttribute("width", &a))
				_marker.back()->setSize(a);
		}
		else if ( !strcmp(node->Value(), "point") ) {
			// create object
			_marker.push_back(new Marker(rs::DOT));
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			if (!node->QueryDoubleAttribute("size", &a))
				_marker.back()->setSize(a);
		}
		else if ( !strcmp(node->Value(), "grid") ) {
			node->QueryDoubleAttribute("tics", &_grid[0]);
			node->QueryDoubleAttribute("hash", &_grid[1]);
			node->QueryDoubleAttribute("minx", &_grid[2]);
			node->QueryDoubleAttribute("maxx", &_grid[3]);
			node->QueryDoubleAttribute("miny", &_grid[4]);
			node->QueryDoubleAttribute("maxy", &_grid[5]);
			node->QueryDoubleAttribute("enabled", &_grid[6]);
			for (int i = 0; i < 6; i++) {
				if (_units) _grid[i] /= 100;
				else _grid[i] /= 39.37;
			}
		}
		else if ( !strcmp(node->Value(), "trace") ) {
			node->QueryIntText(reinterpret_cast<int *>(&_trace));
		}
		else if ( !strcmp(node->Value(), "text") ) {
			// create object
			_marker.push_back(new Marker(rs::TEXT));
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// label
			if ( (ele = node->FirstChildElement("label")) ) {
				_marker.back()->setLabel(ele->GetText());
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "units") ) {
			node->QueryIntText(reinterpret_cast<int *>(&_units));
			for (int i = 0; i < 6; i++) {
				if (_units) _grid[i] /= 100;
				else _grid[i] /= 39.37;
			}
		}

		// go to next node
		node = node->NextSiblingElement();
	}
}

void Reader::read_ground(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	double a, b, c, d;

	// check for existence of node
	if ( (node = doc->FirstChildElement("ground")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if ( !strcmp(node->Value(), "box") ) {
			// create object
			_ground.push_back(new Ground(rs::BOX));
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_ground.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_ground.back()->setDimensions(a, b, c);
			}
			// mass
			if (node->QueryDoubleAttribute("mass", &a))
				_ground.back()->setMass(0.1);
			else
				_ground.back()->setMass(a);
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_ground.back()->setPosition(a, b, c);
			}
			// rotation
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_ground.back()->setRotation(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "cylinder") ) {
			// create object
			_ground.push_back(new Ground(rs::CYLINDER));
			// mass
			if (node->QueryDoubleAttribute("mass", &a))
				_ground.back()->setMass(0.1);
			else
				_ground.back()->setMass(a);
			// axis
			if (node->QueryDoubleAttribute("axis", &a))
				_ground.back()->setAxis(1);
			else
				_ground.back()->setAxis(a);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_ground.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &a);
				ele->QueryDoubleAttribute("length", &b);
				_ground.back()->setDimensions(a, b, 0);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_ground.back()->setPosition(a, b, c);
			}
			// rotation
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_ground.back()->setRotation(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "sphere") ) {
			// create object
			_ground.push_back(new Ground(rs::SPHERE));
			// mass
			if (node->QueryDoubleAttribute("mass", &a))
				_ground.back()->setMass(0.1);
			else
				_ground.back()->setMass(a);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_ground.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &a);
				_ground.back()->setDimensions(a, 0, 0);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_ground.back()->setPosition(a, b, c);
			}
		}

		// go to next node
		node = node->NextSiblingElement();
	}
}

void Reader::read_sim(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	tinyxml2::XMLElement *side = NULL;
	int *rtmp, *ftmp, *ntmp, *atmp, ctype = 0, cnum = 0;
	int custom = 0, i = 0, orientation = 0;
	double size = 0, a, b, c, d, e, f;

	// check for existence of node
	if ( (node = doc->FirstChildElement("sim")) ) {
		node = node->FirstChildElement();
	}

	// check if individual vs preconfig
	//node->QueryIntAttribute("type", reinterpret_cast<int *>(&_preconfig));

	// loop over all nodes
	while (node) {
		if (node->ToComment()) {}
		else if ( !strcmp(node->Value(), "linkboti") ) {
			_robot.push_back(new Linkbot(rs::LINKBOTI, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_robot.back()->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot.back()->setName(str);
			}
			if (!node->QueryIntAttribute("orientation", &i)) {
				_robot.back()->setOrientation(i);
				if (i == rs::RIGHT)
					_robot.back()->setPsi(0);
				else if (i == rs::UP)
					_robot.back()->setPsi(M_PI/2);
				else if (i == rs::LEFT)
					_robot.back()->setPsi(M_PI);
				else if (i == rs::DOWN)
					_robot.back()->setPsi(3*M_PI/2);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_robot.back()->setRotation(DEG2RAD(a), DEG2RAD(b), DEG2RAD(c));
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "linkbotl") ) {
			_robot.push_back(new Linkbot(rs::LINKBOTL, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_robot.back()->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot.back()->setName(str);
			}
			if (!node->QueryIntAttribute("orientation", &i)) {
				if (i == rs::RIGHT)
					_robot.back()->setPsi(0);
				else if (i == rs::UP)
					_robot.back()->setPsi(M_PI/2);
				else if (i == rs::LEFT)
					_robot.back()->setPsi(M_PI);
				else if (i == rs::DOWN)
					_robot.back()->setPsi(3*M_PI/2);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_robot.back()->setRotation(DEG2RAD(a), DEG2RAD(b), DEG2RAD(c));
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "linkbott") ) {
			_robot.push_back(new Linkbot(rs::LINKBOTT, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_robot.back()->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot.back()->setName(str);
			}
			if (!node->QueryIntAttribute("orientation", &i)) {
				if (i == rs::RIGHT)
					_robot.back()->setPsi(0);
				else if (i == rs::UP)
					_robot.back()->setPsi(M_PI/2);
				else if (i == rs::LEFT)
					_robot.back()->setPsi(M_PI);
				else if (i == rs::DOWN)
					_robot.back()->setPsi(3*M_PI/2);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_robot.back()->setRotation(DEG2RAD(a), DEG2RAD(b), DEG2RAD(c));
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "ev3") ) {
			_robot.push_back(new Mindstorms(rs::EV3, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("w1", &a);
				ele->QueryDoubleAttribute("w2", &b);
				_robot.back()->setJoints(a, b);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_robot.back()->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot.back()->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_robot.back()->setRotation(DEG2RAD(a), DEG2RAD(b), DEG2RAD(c));
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "nxt") ) {
			_robot.push_back(new Mindstorms(rs::NXT, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("w1", &a);
				ele->QueryDoubleAttribute("w2", &b);
				_robot.back()->setJoints(a, b);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_robot.back()->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot.back()->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_robot.back()->setRotation(DEG2RAD(a), DEG2RAD(b), DEG2RAD(c));
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else {
			if ( !strcmp(node->Value(), "bigwheel") ) {
				ctype = rsLinkbot::BIGWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "bridge") ) {
				ctype = rsLinkbot::BRIDGE;
				cnum = 2;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "caster") ) {
				ctype = rsLinkbot::CASTER;
				cnum = 1;
				node->QueryIntAttribute("custom", &custom);
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "cube") ) {
				ctype = rsLinkbot::CUBE;
				cnum = 5;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "doublebridge") ) {
				ctype = rsLinkbot::DOUBLEBRIDGE;
				cnum = 4;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "faceplate") ) {
				ctype = rsLinkbot::FACEPLATE;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "gripper") ) {
				ctype = rsLinkbot::GRIPPER;
				cnum = 1;
			}
			/*else if ( !strcmp(node->Value(), "l") ) {
				ctype = rsLinkbot::L;
				cnum = 3;
			}*/
			else if ( !strcmp(node->Value(), "omnidrive") ) {
				ctype = rsLinkbot::OMNIPLATE;
				cnum = 4;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "simple") ) {
				ctype = rsLinkbot::SIMPLE;
				cnum = 2;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "smallwheel") ) {
				ctype = rsLinkbot::SMALLWHEEL;
				cnum = 1;
			}
			/*else if ( !strcmp(node->Value(), "square") ) {
				ctype = rsLinkbot::SQUARE;
				cnum = 4;
			}
			else if ( !strcmp(node->Value(), "tank") ) {
				ctype = rsLinkbot::TANK;
				cnum = 3;
			}*/
			else if ( !strcmp(node->Value(), "tinywheel") ) {
				ctype = rsLinkbot::TINYWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "wheel") ) {
				ctype = rsLinkbot::WHEEL;
				cnum = 1;
				node->QueryDoubleAttribute("radius", &size);
			}
			rtmp = new int[cnum];
			ftmp = new int[cnum];
			ntmp = new int[cnum];
			atmp = new int[cnum];

			// store connector to temp variables
			int i = 0;
			if (cnum == 1) {
				i = 1;
				ntmp[0] = -1;
				atmp[0] = -1;
				node->QueryIntAttribute("robot", &rtmp[0]);
				node->QueryIntAttribute("face", &ftmp[0]);
			}
			else {
				side = node->FirstChildElement();
				while (side) {
					side->QueryIntAttribute("id", &ntmp[i]);
					side->QueryIntAttribute("robot", &rtmp[i]);
					if (side->QueryIntAttribute("conn", &atmp[i]) == tinyxml2::XML_NO_ATTRIBUTE) {
						atmp[i] = -1;
						side->QueryIntAttribute("face", &ftmp[i]);
					}
					else {
						ftmp[i] = ntmp[i];
						side->QueryIntAttribute("conn", &atmp[i]);
						if (atmp[i] == rsLinkbot::CASTER)
							side->QueryDoubleAttribute("custom", &size);
						else if (atmp[i] == rsLinkbot::WHEEL)
							side->QueryDoubleAttribute("radius", &size);
					}
					i++;
					side = side->NextSiblingElement();
				}
			}

			// store connectors to each robot
			for (int j = 0; j < i; j++) {
				for (int k = 0; k < _robot.size(); k++) {
					if (_robot[k]->getID() == rtmp[j]) {
						_robot[k]->addConnector(new Conn(size, orientation, atmp[j], ftmp[0], ftmp[j], rtmp[0], ntmp[j], ctype));
						break;
					}
				}
			}

			// delete temporary arrays
			delete [] rtmp;
			delete [] ftmp;
			delete [] ntmp;
			delete [] atmp;

			// reset temporary variables
			custom = 0;
			i = 0;
			orientation = 0;
			size = 0;
		}

		// go to next node
		node = node->NextSiblingElement();
	}

	// debug printing
	/*std::cerr << "XML Reading Debug List" << std::endl;
	for (int i = 0; i < _robot.size(); i++) {
		_robot[i]->printDebug();
	}*/

	// post process each robot data
	for (int i = 0; i < _robot.size(); i++) {
		_robot[i]->postProcess();
	}
}

