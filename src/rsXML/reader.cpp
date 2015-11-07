#include <cmath>
#include <iostream>

#include <config.h>
#include <rs/Enum>
#include <rsXML/Reader>
#ifdef RS_DOF
#include <rsXML/Dof>
#endif
#ifdef RS_LINKBOT
#include <rsXML/Linkbot>
#endif
#ifdef RS_MINDSTORMS
#include <rsXML/Mindstorms>
#endif

using namespace rsXML;

Reader::Reader(const char *name, bool process) {
	// set default values
	_background = NULL;
	_pause = true;
	_preconfig = false;
	_rt = true;
	_trace = false;
	_version = 0;
	for (int i = 0; i < 2; i++) {
		_friction.push_back(0);
		_restitution.push_back(0);
	}

	// default US units and grid
	_units = false;
	_grid.push_back(1);
	_grid.push_back(12);
	_grid.push_back(-48);
	_grid.push_back(48);
	_grid.push_back(-48);
	_grid.push_back(48);
	_grid.push_back(-1);

	// read XML file
	tinyxml2::XMLDocument doc;
	this->load_file(name, &doc);
	this->read_config(&doc);
	this->read_obstacles(&doc);
	this->read_graphics(&doc);
	this->read_sim(&doc, process);
}

Reader::~Reader(void) {
	for (unsigned int i = 0; i < _marker.size(); i++) {
		delete _marker[i];
	}
	for (unsigned int i = 0; i < _obstacle.size(); i++) {
		delete _obstacle[i];
	}
	for (unsigned int i = 0; i < _robot.size(); i++) {
		delete _robot[i];
	}
	delete _background;
}

/**********************************************************
	public functions
 **********************************************************/
std::string Reader::getBackgroundImage(int pos) {
	if (!_background) return std::string();
	return _background->getBackgroundImage(pos);
}

int Reader::getLevel(void) {
	if (!_background) return -1;
	return _background->getLevel();
}

int Reader::addNewRobot(Robot *bot) {
	_robot.push_back(bot);
	return 0;
}

void Reader::addBackgroundObjects(void) {
	for (int i = 0; i < _background->getNumMarkers(); i++)
		_marker.push_back(_background->getMarker(i));
	for (int i = 0; i < _background->getNumObstacles(); i++)
		_obstacle.push_back(_background->getObstacle(i));
}

std::string Reader::getName(void) {
	if (!_background) return std::string();
	return _background->getName();
}

Obstacle* Reader::getObstacle(int id) {
	return _obstacle[id];
}

Marker* Reader::getMarker(int id) {
	return _marker[id];
}

Marker* Reader::getNextMarker(int form) {
	// find next marker in xml list
	unsigned int i = 0;
	for (i = 0; i < _marker.size(); i++) {
		if (!_marker[i]->getConnect() && (form == -1 || _marker[i]->getForm() == form))
			break;
	}

	// haven't found one
	if (i == _marker.size())
		return NULL;

	// robot now connected
	_marker[i]->setConnect(1);

	// success
	return _marker[i];
}

Obstacle* Reader::getNextObstacle(int form) {
	// find next obstacle in xml list
	unsigned int i = 0;
	for (i = 0; i < _obstacle.size(); i++) {
		if (!_obstacle[i]->getConnect() && (form == -1 || _obstacle[i]->getForm() == form))
			break;
	}

	// haven't found one
	if (i == _obstacle.size())
		return NULL;

	// robot now connected
	_obstacle[i]->setConnect(1);

	// success
	return _obstacle[i];
}

Robot* Reader::getNextRobot(int form) {
	// find next robot in xml list
	unsigned int i = 0;
	for (i = 0; i < _robot.size(); i++) {
		if (!_robot[i]->getConnect()) {
			if (_robot[i]->getForm() == rs::EV3 && form == rs::NXT) {
				_robot[i]->setForm(form);
				break;
			}
			else if (_robot[i]->getForm() == rs::NXT && form == rs::EV3) {
				_robot[i]->setForm(form);
				break;
			}
			else if (_robot[i]->getForm() == form)
				break;
			else
				break;
		}
	}

	// no robot found
	if (form != -1 && (i == _robot.size() || _robot[i]->getForm() != form)) {
		switch (form) {
#ifdef RS_DOF
			case rs::Dof:
				std::cerr << "Error: Could not find a Dof in the RoboSim GUI robot list." << std::endl;
				break;
#endif
#ifdef RS_LINKBOT
			case rs::LinkbotI:
				std::cerr << "Error: Could not find a Linkbot-I in the RoboSim GUI robot list." << std::endl;
				break;
			case rs::LinkbotL:
				std::cerr << "Error: Could not find a Linkbot-L in the RoboSim GUI robot list." << std::endl;
				break;
			case rs::LinkbotT:
				std::cerr << "Error: Could not find a Linkbot-T in the RoboSim GUI robot list." << std::endl;
				break;
#endif
#ifdef RS_MINDSTORMS
			case rs::EV3: case rs::NXT:
				std::cerr << "Error: Could not find a Mindstorms EV3 or NXT in the RoboSim GUI robot list." << std::endl;
				break;
#endif
		}
		if (_preconfig) {
			fprintf(stderr, "\tPreconfigured Robot Configuration selected.\n");
			fprintf(stderr, "\tPlease uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	// haven't found one
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
	for (int i = 0; i < 6; i++) {
		_grid[i] = (_units) ? rs::CM2M(_grid[i]) : rs::IN2M(_grid[i]);
	}
	return _grid;
}

std::vector<double> Reader::getRestitution(void) {
	return _restitution;
}

int Reader::getNumObstacles(void) {
	return _obstacle.size();
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
void Reader::load_file(const char *name, tinyxml2::XMLDocument *doc) {
	// get file
	if (name != NULL) {
#ifdef RS_WIN32
		FILE *fp = fopen(name, "r");
		if (fp) {
			_path = name;
			fclose(fp);
		}
		else {
			char base[512];
			if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, base))) {
				_path.append(base);
				_path.append("\\C-STEM Studio\\RoboSim\\");
				_path.append(name);
			}
		}
#else
		_path = getenv("HOME");
		FILE *fp = fopen(name, "r");
		if (fp) {
			_path = name;
			fclose(fp);
		}
		else {
			_path.append("/.robosimrc");
		}
#endif
	}
	else {
		_path = rsXML::getDefaultPath();
	}

	// load file
	int output = doc->LoadFile(_path.c_str());
	if (output) {
		std::cerr << "Error: Could not find RoboSim config file." << std::endl;
		std::cerr << "Please run RoboSim GUI." << std::endl;
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

	// check for the background node
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("background")) ) {
		std::string name(node->GetText());
		_background = new BackgroundReader(name);
	}
	else
		_background = new BackgroundReader("outdoors");
}

void Reader::read_graphics(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	int i;
	double a, b, c, d;

	// check for existence of node
	if ( (node = doc->FirstChildElement("graphics")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if ( !strcmp(node->Value(), "line") ) {
			// create object
			_marker.push_back(new Marker(rs::Line));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// end position
			if ( (ele = node->FirstChildElement("end")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_marker.back()->setEnd(a, b, c);
			}
			// start position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			i = 0;
			if ( !node->QueryIntAttribute("width", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "dot") ) {
			// create object
			_marker.push_back(new Marker(rs::Dot));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			i = 0;
			if ( !node->QueryIntAttribute("radius", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "grid") ) {
			node->QueryDoubleAttribute("tics", &_grid[0]);
			node->QueryDoubleAttribute("hash", &_grid[1]);
			node->QueryDoubleAttribute("minx", &_grid[2]);
			node->QueryDoubleAttribute("maxx", &_grid[3]);
			node->QueryDoubleAttribute("miny", &_grid[4]);
			node->QueryDoubleAttribute("maxy", &_grid[5]);
			node->QueryDoubleAttribute("enabled", &_grid[6]);
		}
		else if ( !strcmp(node->Value(), "trace") ) {
			node->QueryIntText(reinterpret_cast<int *>(&_trace));
		}
		else if ( !strcmp(node->Value(), "text") ) {
			// create object
			_marker.push_back(new Marker(rs::Text));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// label
			if ( (ele = node->FirstChildElement("name")) ) {
				_marker.back()->setLabel(ele->GetText());
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "units") ) {
			node->QueryIntText(reinterpret_cast<int *>(&_units));
		}

		// go to next node
		node = node->NextSiblingElement();
	}
}

void Reader::read_obstacles(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	int i;
	double a, b, c, d;

	// check for existence of node
	if ( (node = doc->FirstChildElement("ground")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if ( !strcmp(node->Value(), "box") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::Box));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_obstacle.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_obstacle.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_obstacle.back()->setDimensions(a, b, c);
			}
			// mass
			a = 0;
			if ( !node->QueryDoubleAttribute("mass", &a) ) {
				_obstacle.back()->setMass(a);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_obstacle.back()->setPosition(a, b, c);
			}
			// rotation
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_obstacle.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
			}
		}
		else if ( !strcmp(node->Value(), "cylinder") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::Cylinder));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_obstacle.back()->setID(i);
			// axis
			i = 0;
			if ( !node->QueryIntAttribute("axis", &i) ) {
				_obstacle.back()->setAxis(i);
			}
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_obstacle.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				a = 0; b = 0;
				ele->QueryDoubleAttribute("radius", &a);
				ele->QueryDoubleAttribute("length", &b);
				_obstacle.back()->setDimensions(a, b, 0);
			}
			// mass
			a = 0;
			if ( !node->QueryDoubleAttribute("mass", &a) ) {
				_obstacle.back()->setMass(a);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_obstacle.back()->setPosition(a, b, c);
			}
			// rotation
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("psi", &a);
				ele->QueryDoubleAttribute("theta", &b);
				ele->QueryDoubleAttribute("phi", &c);
				_obstacle.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
			}
		}
		else if ( !strcmp(node->Value(), "hackysack") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::HackySack));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_obstacle.back()->setID(i);
			// dimensions
			_obstacle.back()->setDimensions(0.0275, 0, 0);	// measured 5.5cm diameter
			// mass
			_obstacle.back()->setMass(0.060);		// 60 grams is average hacky sack weight
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				_obstacle.back()->setPosition(a, b, 0.0275);
			}
		}
		else if ( !strcmp(node->Value(), "sphere") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::Sphere));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_obstacle.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryDoubleAttribute("r", &a);
				ele->QueryDoubleAttribute("g", &b);
				ele->QueryDoubleAttribute("b", &c);
				ele->QueryDoubleAttribute("alpha", &d);
				_obstacle.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ((ele = node->FirstChildElement("size"))) {
				a = 0;
				ele->QueryDoubleAttribute("radius", &a);
				_obstacle.back()->setDimensions(a, 0, 0);
			}
			// mass
			a = 0;
			if ( !node->QueryDoubleAttribute("mass", &a) ) {
				_obstacle.back()->setMass(a);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_obstacle.back()->setPosition(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "woodblock") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::WoodBlock));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_obstacle.back()->setID(i);
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_obstacle.back()->setDimensions(a, b, c);
			}
			// mass
			_obstacle.back()->setMass(0.5*a*b*c);	// mass = density * l * w * h
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_obstacle.back()->setPosition(a, b, c);
			}
		}

		// go to next node
		node = node->NextSiblingElement();
	}
}

void Reader::read_sim(tinyxml2::XMLDocument *doc, bool process) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	tinyxml2::XMLElement *side = NULL;
	int *rtmp, *ftmp, *ntmp, *atmp, ctype = 0, cnum = 0;
	int custom = 0, i = 0, j = 0, orientation = 0;
	double size = 0, a, b, c, d;

	// check for existence of node
	if ( (node = doc->FirstChildElement("sim")) ) {
		node = node->FirstChildElement();
	}

	// check if individual vs preconfig
	//node->QueryIntAttribute("type", reinterpret_cast<int *>(&_preconfig));

	// loop over all nodes
	while (node) {
		if (node->ToComment()) {}
#ifdef RS_DOF
		else if ( !strcmp(node->Value(), "dof") ) {
			_robot.push_back(new Dof());
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("j", &a);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
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
				if (i == rs::Right)
					_robot.back()->setPsi(0);
				else if (i == rs::Up)
					_robot.back()->setPsi(rs::Pi/2);
				else if (i == rs::Left)
					_robot.back()->setPsi(rs::Pi);
				else if (i == rs::Down)
					_robot.back()->setPsi(3*rs::Pi/2);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryDoubleAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("theta", &b);
					ele->QueryDoubleAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryDoubleAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("x", &a);
					ele->QueryDoubleAttribute("y", &b);
					ele->QueryDoubleAttribute("z", &c);
					ele->QueryDoubleAttribute("w", &d);
					_robot.back()->setRotation(a, b, c, d);
				}
				else {
					_robot.back()->setRotation(0, 0, 0, 1);
				}
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
#endif
#ifdef RS_LINKBOT
		else if ( !strcmp(node->Value(), "linkboti") ) {
			_robot.push_back(new Linkbot(rs::LinkbotI, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
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
				if (i == rs::Right)
					_robot.back()->setPsi(0);
				else if (i == rs::Up)
					_robot.back()->setPsi(rs::Pi/2);
				else if (i == rs::Left)
					_robot.back()->setPsi(rs::Pi);
				else if (i == rs::Down)
					_robot.back()->setPsi(3*rs::Pi/2);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryDoubleAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("theta", &b);
					ele->QueryDoubleAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryDoubleAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("x", &a);
					ele->QueryDoubleAttribute("y", &b);
					ele->QueryDoubleAttribute("z", &c);
					ele->QueryDoubleAttribute("w", &d);
					_robot.back()->setRotation(a, b, c, d);
				}
				else {
					_robot.back()->setRotation(0, 0, 0, 1);
				}
			}
			if ( (ele = node->FirstChildElement("wheels")) ) {
				i = 0; j = 0;
				ele->QueryIntAttribute("left", &i);
				ele->QueryIntAttribute("right", &j);
				if (i == rsLinkbot::Connectors::Wheel || j == rsLinkbot::Connectors::Wheel) {
					std::cerr << "RoboSim currently only supports the default wheel sizes." << std::endl;
					std::cerr << "Defaulting to small Linkbot wheels." << std::endl;
					i = rsLinkbot::Connectors::SmallWheel;
					j = rsLinkbot::Connectors::SmallWheel;
				}
				if (i) {
					_robot.back()->addConnector(new Conn(0, 0, -1, 1, 1, _robot.back()->getID(), 1, rsLinkbot::Connectors::Simple));
					_robot.back()->addConnector(new Conn(0, 0, i, 1, 1, _robot.back()->getID(), 2, rsLinkbot::Connectors::Simple));
				}
				if (i || j) {
					_robot.back()->addConnector(new Conn(0, 0, -1, 2, 2, _robot.back()->getID(), 1, rsLinkbot::Connectors::Simple));
					_robot.back()->addConnector(new Conn(0, 0, rsLinkbot::Connectors::Caster, 2, 2, _robot.back()->getID(), 2, rsLinkbot::Connectors::Simple));
				}
				if (j) {
					_robot.back()->addConnector(new Conn(0, 0, -1, 3, 3, _robot.back()->getID(), 1, rsLinkbot::Connectors::Simple));
					_robot.back()->addConnector(new Conn(0, 0, j, 3, 3, _robot.back()->getID(), 2, rsLinkbot::Connectors::Simple));
				}
				_robot.back()->setWheels(i, j);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "linkbotl") ) {
			_robot.push_back(new Linkbot(rs::LinkbotL, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
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
				if (i == rs::Right)
					_robot.back()->setPsi(0);
				else if (i == rs::Up)
					_robot.back()->setPsi(rs::Pi/2);
				else if (i == rs::Left)
					_robot.back()->setPsi(rs::Pi);
				else if (i == rs::Down)
					_robot.back()->setPsi(3*rs::Pi/2);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryDoubleAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("theta", &b);
					ele->QueryDoubleAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryDoubleAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("x", &a);
					ele->QueryDoubleAttribute("y", &b);
					ele->QueryDoubleAttribute("z", &c);
					ele->QueryDoubleAttribute("w", &d);
					_robot.back()->setRotation(a, b, c, d);
				}
				else {
					_robot.back()->setRotation(0, 0, 0, 1);
				}
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "linkbott") ) {
			_robot.push_back(new Linkbot(rs::LinkbotT, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
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
				if (i == rs::Right)
					_robot.back()->setPsi(0);
				else if (i == rs::Up)
					_robot.back()->setPsi(rs::Pi/2);
				else if (i == rs::Left)
					_robot.back()->setPsi(rs::Pi);
				else if (i == rs::Down)
					_robot.back()->setPsi(3*rs::Pi/2);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryDoubleAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("theta", &b);
					ele->QueryDoubleAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryDoubleAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("x", &a);
					ele->QueryDoubleAttribute("y", &b);
					ele->QueryDoubleAttribute("z", &c);
					ele->QueryDoubleAttribute("w", &d);
					_robot.back()->setRotation(a, b, c, d);
				}
				else {
					_robot.back()->setRotation(0, 0, 0, 1);
				}
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
#endif
#ifdef RS_MINDSTORMS
		else if ( !strcmp(node->Value(), "ev3") ) {
			_robot.push_back(new Mindstorms(rs::EV3, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0;
				ele->QueryDoubleAttribute("w1", &a);
				ele->QueryDoubleAttribute("w2", &b);
				_robot.back()->setJoints(a, b);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
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
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryDoubleAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("theta", &b);
					ele->QueryDoubleAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryDoubleAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("x", &a);
					ele->QueryDoubleAttribute("y", &b);
					ele->QueryDoubleAttribute("z", &c);
					ele->QueryDoubleAttribute("w", &d);
					_robot.back()->setRotation(a, b, c, d);
				}
				else {
					_robot.back()->setRotation(0, 0, 0, 1);
				}
			}
			if ( (ele = node->FirstChildElement("wheels")) ) {
				i = 0; j = 0;
				ele->QueryIntAttribute("left", &i);
				ele->QueryIntAttribute("right", &j);
				_robot.back()->setWheels(i, j);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "nxt") ) {
			_robot.push_back(new Mindstorms(rs::NXT, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0;
				ele->QueryDoubleAttribute("w1", &a);
				ele->QueryDoubleAttribute("w2", &b);
				_robot.back()->setJoints(a, b);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
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
				a = 0; b = 0; c = 0;
				ele->QueryDoubleAttribute("x", &a);
				ele->QueryDoubleAttribute("y", &b);
				ele->QueryDoubleAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryDoubleAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("theta", &b);
					ele->QueryDoubleAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryDoubleAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryDoubleAttribute("x", &a);
					ele->QueryDoubleAttribute("y", &b);
					ele->QueryDoubleAttribute("z", &c);
					ele->QueryDoubleAttribute("w", &d);
					_robot.back()->setRotation(a, b, c, d);
				}
				else {
					_robot.back()->setRotation(0, 0, 0, 1);
				}
			}
			if ( (ele = node->FirstChildElement("wheels")) ) {
				i = 0; j = 0;
				ele->QueryIntAttribute("left", &i);
				ele->QueryIntAttribute("right", &j);
				_robot.back()->setWheels(i, j);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
#endif
		else {
#ifdef RS_DOF
			if ( !strcmp(node->Value(), "el") ) {
				ctype = rsDof::Connectors::El;
				cnum = 2;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "foot") ) {
				ctype = rsDof::Connectors::Foot;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "plank") ) {
				ctype = rsDof::Connectors::Plank;
				cnum = 2;
				node->QueryIntAttribute("orientation", &orientation);
			}
#endif
#ifdef RS_LINKBOT
			if ( !strcmp(node->Value(), "bigwheel") ) {
				ctype = rsLinkbot::Connectors::BigWheel;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "bridge") ) {
				ctype = rsLinkbot::Connectors::Bridge;
				cnum = 2;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "caster") ) {
				ctype = rsLinkbot::Connectors::Caster;
				cnum = 1;
				node->QueryIntAttribute("custom", &custom);
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "cube") ) {
				ctype = rsLinkbot::Connectors::Cube;
				cnum = 5;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "doublebridge") ) {
				ctype = rsLinkbot::Connectors::DoubleBridge;
				cnum = 4;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "faceplate") ) {
				ctype = rsLinkbot::Connectors::Faceplate;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "gripper") ) {
				ctype = rsLinkbot::Connectors::Gripper;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "omnidrive") ) {
				ctype = rsLinkbot::Connectors::Omniplate;
				cnum = 4;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "simple") ) {
				ctype = rsLinkbot::Connectors::Simple;
				cnum = 2;
				node->QueryIntAttribute("orientation", &orientation);
			}
			else if ( !strcmp(node->Value(), "smallwheel") ) {
				ctype = rsLinkbot::Connectors::SmallWheel;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "tinywheel") ) {
				ctype = rsLinkbot::Connectors::TinyWheel;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "wheel") ) {
				ctype = rsLinkbot::Connectors::Wheel;
				cnum = 1;
				node->QueryDoubleAttribute("radius", &size);
			}
#endif
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
#ifdef RS_LINKBOT
						if (atmp[i] == rsLinkbot::Connectors::Caster)
							side->QueryDoubleAttribute("custom", &size);
						else if (atmp[i] == rsLinkbot::Connectors::Wheel)
							side->QueryDoubleAttribute("radius", &size);
#endif
					}
					i++;
					side = side->NextSiblingElement();
				}
			}

			// store connectors to each robot
			for (int j = 0; j < i; j++) {
				for (unsigned int k = 0; k < _robot.size(); k++) {
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
	for (unsigned int i = 0; i < _robot.size(); i++) {
		_robot[i]->printDebug();
	}*/

	// post process each robot data
	if (process) {
		for (unsigned int i = 0; i < _robot.size(); i++) {
			_robot[i]->postProcess();
		}
	}
}

std::string rsXML::getDefaultPath(void) {
	std::string path;

#ifdef RS_WIN32
	char base[512];
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, base))) {
		path = base;
		path.append("\\C-STEM Studio\\RoboSim\\robosim.xml");
	}
#else
	path = getenv("HOME");
	path.append("/.robosimrc");
#endif

	return path;
}

