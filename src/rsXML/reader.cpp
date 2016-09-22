#include <cmath>
#include <iostream>
#include <vector>

#include <tinyxml2.h>

#include <config.h>
#include <rs/Enum>
#include <rsXML/Config>
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
	_trace = true;
	_version = RSXML_VER_NONE;
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
std::string Reader::getBackgroundImage(short pos) {
	if (!_background) return std::string();
	return _background->getBackgroundImage(pos);
}

short Reader::getLevel(void) {
	if (!_background) return -1;
	return _background->getLevel();
}

void Reader::addNewRobot(Robot *bot) {
	_robot.push_back(bot);
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

Obstacle* Reader::getObstacle(short id) {
	return _obstacle[id];
}

Marker* Reader::getMarker(short id) {
	return _marker[id];
}

Marker* Reader::getNextMarker(short form) {
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

Obstacle* Reader::getNextObstacle(short form) {
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

Robot* Reader::getNextRobot(short form) {
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

std::vector<float> Reader::getFriction(void) {
	return _friction;
}

std::vector<float> Reader::getGrid(void) {
	for (int i = 0; i < 6; i++) {
		_grid[i] = (_units) ? rs::CM2M(_grid[i]) : rs::IN2M(_grid[i]);
	}
	return _grid;
}

std::vector<float> Reader::getRestitution(void) {
	return _restitution;
}

short Reader::getNumObstacles(void) {
	return _obstacle.size();
}

short Reader::getNumMarkers(void) {
	return _marker.size();
}

short Reader::getNumRobots(void) {
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
			if (PathFileExistsA(_path.c_str()) != 1) {
				std::string dir(base);
				dir.append("\\C-STEM Studio");
				CreateDirectory(dir.c_str(), NULL);
				dir.append("\\RoboSim");
				CreateDirectory(dir.c_str(), NULL);
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

	// check version number
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("version")) ) {
		node->QueryIntText(&_version);
	}
	if (_version != RSXML_VER_CURRENT) {
		_version = RSXML_VER_BAD;
		tinyxml2::XMLElement *element = doc->FirstChildElement("config");
		if (element) element->DeleteChildren();
		return;
	}

	// check for custom mu params
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("mu")) ) {
		node->QueryFloatAttribute("ground", &(_friction[0]));
		node->QueryFloatAttribute("body", &(_friction[1]));
	}

	// check for custom cor params
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("cor")) ) {
		node->QueryFloatAttribute("ground", &(_restitution[0]));
		node->QueryFloatAttribute("body", &(_restitution[1]));
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
	// check for compatible version
	if (_version == RSXML_VER_BAD) {
		tinyxml2::XMLElement *element = doc->FirstChildElement("graphics");
		if (element) element->DeleteChildren();
		return;
	}

	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	int i;
	float a, b, c, d;

	// check for existence of node
	if ( (node = doc->FirstChildElement("graphics")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if ( !strcmp(node->Value(), "circle") ) {
			// create object
			_marker.push_back(new Marker(rs::Circle));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// start position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			i = 0;
			if ( !node->QueryIntAttribute("width", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "ellipse") ) {
			// create object
			_marker.push_back(new Marker(rs::Ellipse));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// start position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// end position
			if ( (ele = node->FirstChildElement("end")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setEnd(a, b, c);
			}
			// size
			i = 0;
			if ( !node->QueryIntAttribute("width", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "line") ) {
			// create object
			_marker.push_back(new Marker(rs::Line));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// end position
			if ( (ele = node->FirstChildElement("end")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setEnd(a, b, c);
			}
			// start position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
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
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			i = 0;
			if ( !node->QueryIntAttribute("radius", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "grid") ) {
			node->QueryFloatAttribute("tics", &_grid[0]);
			node->QueryFloatAttribute("hash", &_grid[1]);
			node->QueryFloatAttribute("minx", &_grid[2]);
			node->QueryFloatAttribute("maxx", &_grid[3]);
			node->QueryFloatAttribute("miny", &_grid[4]);
			node->QueryFloatAttribute("maxy", &_grid[5]);
			node->QueryFloatAttribute("enabled", &_grid[6]);
		}
		else if ( !strcmp(node->Value(), "polygon") ) {
			// create object
			_marker.push_back(new Marker(rs::Polygon));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// end position
			if ( (ele = node->FirstChildElement("end")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setEnd(a, b, c);
			}
			// start position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			i = 0;
			if ( !node->QueryIntAttribute("width", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "rectangle") ) {
			// create object
			_marker.push_back(new Marker(rs::Rectangle));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// end position
			if ( (ele = node->FirstChildElement("end")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setEnd(a, b, c);
			}
			// start position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			i = 0;
			if ( !node->QueryIntAttribute("width", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "text") ) {
			// create object
			_marker.push_back(new Marker(rs::Text));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// size
			i = 0;
			if ( !node->QueryIntAttribute("size", &i) ) {
				_marker.back()->setSize(i);
			}
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// label
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_marker.back()->setLabel(str);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "trace") ) {
			node->QueryIntText(reinterpret_cast<int *>(&_trace));
		}
		else if ( !strcmp(node->Value(), "triangle") ) {
			// create object
			_marker.push_back(new Marker(rs::Triangle));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_marker.back()->setID(i);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_marker.back()->setColor(a, b, c, d);
			}
			// end position
			if ( (ele = node->FirstChildElement("end")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setEnd(a, b, c);
			}
			// start position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_marker.back()->setStart(a, b, c);
			}
			// size
			i = 0;
			if ( !node->QueryIntAttribute("width", &i) ) {
				_marker.back()->setSize(i);
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
	// check for compatible version
	if (_version == RSXML_VER_BAD) {
		tinyxml2::XMLElement *element = doc->FirstChildElement("obstacles");
		if (element) element->DeleteChildren();
		return;
	}

	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	int i;
	float a, b, c, d;

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
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_obstacle.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_obstacle.back()->setDimensions(a, b, c);
			}
			// mass
			a = 0;
			if ( !node->QueryFloatAttribute("mass", &a) ) {
				_obstacle.back()->setMass(a);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_obstacle.back()->setPosition(a, b, c);
			}
			// rotation
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("psi", &a);
				ele->QueryFloatAttribute("theta", &b);
				ele->QueryFloatAttribute("phi", &c);
				_obstacle.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
			}
		}
		else if ( !strcmp(node->Value(), "competitionborder") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::CompetitionBorder));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_obstacle.back()->setID(i);
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("xlength", &a);
				ele->QueryFloatAttribute("ylength", &b);
				ele->QueryFloatAttribute("radius", &c);
				_obstacle.back()->setDimensions(a, b, c);
			}
			// mass
			_obstacle.back()->setMass(10000);
			// color
			if ( (ele = node->FirstChildElement("color")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_obstacle.back()->setColor(a, b, c, d);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				_obstacle.back()->setPosition(a, b, 0);
			}
			// rotation
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_obstacle.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_obstacle.back()->setRotation(a, b, c, d);
				}
				else {
					_obstacle.back()->setRotation(0, 0, 0, 1);
				}
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
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_obstacle.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ( (ele = node->FirstChildElement("size")) ) {
				a = 0; b = 0;
				ele->QueryFloatAttribute("radius", &a);
				ele->QueryFloatAttribute("length", &b);
				_obstacle.back()->setDimensions(a, b, 0);
			}
			// mass
			a = 0;
			if ( !node->QueryFloatAttribute("mass", &a) ) {
				_obstacle.back()->setMass(a);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_obstacle.back()->setPosition(a, b, c);
			}
			// rotation
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("psi", &a);
				ele->QueryFloatAttribute("theta", &b);
				ele->QueryFloatAttribute("phi", &c);
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
			_obstacle.back()->setDimensions(0.055, 0.055, 0.055);	// measured 5.5cm diameter
			// mass
			_obstacle.back()->setMass(0.060);		// 60 grams is average hacky sack weight
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0.0275;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_obstacle.back()->setPosition(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "pullupbar") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::PullupBar));
			// id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_obstacle.back()->setID(i);
			// dimensions
			_obstacle.back()->setDimensions(0, 0, 0);
			// mass
			_obstacle.back()->setMass(10000);
			// color
			_obstacle.back()->setColor(1, 1, 1, 1);
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				_obstacle.back()->setPosition(a, b, 0);
			}
			// rotation
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_obstacle.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_obstacle.back()->setRotation(a, b, c, d);
				}
				else {
					_obstacle.back()->setRotation(0, 0, 0, 1);
				}
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
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_obstacle.back()->setColor(a, b, c, d);
			}
			// dimensions
			if ((ele = node->FirstChildElement("size"))) {
				a = 0;
				ele->QueryFloatAttribute("radius", &a);
				_obstacle.back()->setDimensions(a, 0, 0);
			}
			// mass
			a = 0;
			if ( !node->QueryFloatAttribute("mass", &a) ) {
				_obstacle.back()->setMass(a);
			}
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
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
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_obstacle.back()->setDimensions(a, b, c);
			}
			// mass
			_obstacle.back()->setMass(0.5*a*b*c);	// mass = density * l * w * h
			// position
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_obstacle.back()->setPosition(a, b, c);
			}
		}

		// go to next node
		node = node->NextSiblingElement();
	}
}

void Reader::read_sim(tinyxml2::XMLDocument *doc, bool process) {
	// check for compatible version
	if (_version == RSXML_VER_BAD) {
		tinyxml2::XMLElement *element = doc->FirstChildElement("sim");
		if (element) element->DeleteChildren();
		return;
	}

	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	tinyxml2::XMLElement *side = NULL;
	std::vector<int> rtmp, ftmp, ntmp, atmp;
	rtmp.resize(5);
	ftmp.resize(5);
	ntmp.resize(5);
	atmp.resize(5);
	int ctype = 0, cnum = 0;
	int custom = 0, i = 0, j = 0, k = 0, orientation = 0;
	float size = 0, a, b, c, d;

	// check for existence of node
	if ( (node = doc->FirstChildElement("sim")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if (node->ToComment()) {}
#ifdef RS_DOF
		else if ( !strcmp(node->Value(), "dof") ) {
			// query scale
			a = 1;
			node->QueryFloatAttribute("scale", &a);
			// create robot
			_robot.push_back(new Dof(a, _trace));
			// query id
			i = 0;
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			// query enabled joint
			if ( (ele = node->FirstChildElement("enabled")) ) {
				i = 1;
				ele->QueryIntText(&i);
				_robot.back()->setEnabled(i);
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0;
				ele->QueryFloatText(&a);
				_robot.back()->setJoints(a, 0);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
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
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
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
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Individual, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("f1", &a);
				ele->QueryFloatAttribute("f2", &b);
				ele->QueryFloatAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
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
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot.back()->setRotation(a, b, c, d);
				}
				else {
					_robot.back()->setRotation(0, 0, 0, 1);
				}
			}
			if ( (ele = node->FirstChildElement("wheels")) ) {
				i = 0; j = 0, k = 0;
				ele->QueryIntAttribute("left", &i);
				ele->QueryIntAttribute("right", &j);
				ele->QueryIntAttribute("caster", &k);
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
					_robot.back()->addConnector(new Conn(0, 0, k, 2, 2, _robot.back()->getID(), 2, rsLinkbot::Connectors::Simple));
				}
				if (j) {
					_robot.back()->addConnector(new Conn(0, 0, -1, 3, 3, _robot.back()->getID(), 1, rsLinkbot::Connectors::Simple));
					_robot.back()->addConnector(new Conn(0, 0, j, 3, 3, _robot.back()->getID(), 2, rsLinkbot::Connectors::Simple));
				}
				_robot.back()->setWheels(i, j);
				_robot.back()->setCaster(k);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "linkbotl") ) {
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Individual, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("f1", &a);
				ele->QueryFloatAttribute("f2", &b);
				ele->QueryFloatAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
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
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
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
			_robot.push_back(new Linkbot(rs::LinkbotT, rsLinkbot::Preconfigs::Individual, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("f1", &a);
				ele->QueryFloatAttribute("f2", &b);
				ele->QueryFloatAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
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
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot.back()->setRotation(a, b, c, d);
				}
				else {
					_robot.back()->setRotation(0, 0, 0, 1);
				}
			}
			i = (node->QueryIntAttribute("ground", &i)) ? -1 : i;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "bow") ) {
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Bow, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Bow, _trace));
			int second = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[first]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[second]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[first]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			_robot[second]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[second]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "bugclock") ) {
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::BugClock, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::BugClock, _trace));
			int second = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::BugClock, _trace));
			int third = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[third]->setID(i + 2);
			_robot[first]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, -1, 3, 3, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 3, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 3, 3, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, 0, -1, 3, 1, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
				_robot[third]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
				_robot[third]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "explorer") ) {
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Explorer, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Explorer, _trace));
			int second = _robot.size() - 1;
			_robot[second]->setOrientation(rs::Down);
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Explorer, _trace));
			int third = _robot.size() - 1;
			_robot[third]->setOrientation(rs::Left);
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Explorer, _trace));
			int fourth = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Explorer, _trace));
			int fifth = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[third]->setID(i + 2);
			_robot[fourth]->setID(i + 3);
			_robot[fifth]->setID(i + 4);
			_robot[first]->addConnector(new Conn(0, rs::Up, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Cube));
			_robot[first]->addConnector(new Conn(0, rs::Up, rsLinkbot::Connectors::Caster, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Cube));
			_robot[first]->addConnector(new Conn(0, rs::Up, rsLinkbot::Connectors::Caster, 1, 1, _robot[first]->getID(), 4, rsLinkbot::Connectors::Cube));
			_robot[second]->addConnector(new Conn(0, rs::Up, -1, 1, 3, _robot[first]->getID(), 3, rsLinkbot::Connectors::Cube));
			_robot[third]->addConnector(new Conn(0, rs::Up, -1, 1, 2, _robot[first]->getID(), 5, rsLinkbot::Connectors::Cube));
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::BigWheel, 3, 3, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::BigWheel, 1, 1, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[third]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[third]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[third]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[fourth]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[third]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[fourth]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[fourth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[fourth]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, -1, 1, 1, _robot[fifth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Left, rsLinkbot::Connectors::Gripper, 1, 1, _robot[fifth]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[fifth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, rsLinkbot::Connectors::Gripper, 3, 3, _robot[fifth]->getID(), 2, rsLinkbot::Connectors::Simple));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
				_robot[third]->setLED(a, b, c, d);
				_robot[fourth]->setLED(a, b, c, d);
				_robot[fifth]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
				_robot[third]->setName(str);
				_robot[fourth]->setName(str);
				_robot[fifth]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "fourbotdrive") ) {
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::FourBotDrive, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::FourBotDrive, _trace));
			int second = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::FourBotDrive, _trace));
			int third = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::FourBotDrive, _trace));
			int fourth = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[third]->setID(i + 2);
			_robot[fourth]->setID(i + 3);
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Omniplate));
			_robot[second]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 2, rsLinkbot::Connectors::Omniplate));
			_robot[third]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 3, rsLinkbot::Connectors::Omniplate));
			_robot[fourth]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 4, rsLinkbot::Connectors::Omniplate));
			_robot[first]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, 0, -1, 3, 3, _robot[third]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 3, 3, _robot[third]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[fourth]->addConnector(new Conn(0, 0, -1, 3, 3, _robot[fourth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fourth]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 3, 3, _robot[fourth]->getID(), 2, rsLinkbot::Connectors::Simple));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
				_robot[third]->setLED(a, b, c, d);
				_robot[fourth]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
				_robot[third]->setName(str);
				_robot[fourth]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "fourwheeldrive") ) {
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::FourBotDrive, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::FourBotDrive, _trace));
			int second = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Cube));
			_robot[second]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 3, rsLinkbot::Connectors::Cube));
			_robot[first]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, -1, 3, 3, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 3, 3, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 3, 3, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 3, 3, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "fourwheelexplorer") ) {
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Explorer, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Explorer, _trace));
			int second = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Explorer, _trace));
			int third = _robot.size() - 1;
			_robot[third]->setOrientation(rs::Left);
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Explorer, _trace));
			int fourth = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Explorer, _trace));
			int fifth = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[third]->setID(i + 2);
			_robot[fourth]->setID(i + 3);
			_robot[fifth]->setID(i + 4);
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Cube));
			_robot[first]->addConnector(new Conn(0, rs::Right, rsLinkbot::Connectors::Caster, 2, 2, _robot[first]->getID(), 4, rsLinkbot::Connectors::Cube));
			_robot[second]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 3, rsLinkbot::Connectors::Cube));
			_robot[third]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 5, rsLinkbot::Connectors::Cube));
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::BigWheel, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::BigWheel, 3, 3, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::BigWheel, 1, 1, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 3, 3, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::BigWheel, 3, 3, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[third]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[third]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[third]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[fourth]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[third]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[fourth]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[fourth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[fourth]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Up, -1, 1, 1, _robot[fifth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Left, rsLinkbot::Connectors::Gripper, 1, 1, _robot[fifth]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[fifth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, rsLinkbot::Connectors::Gripper, 3, 3, _robot[fifth]->getID(), 2, rsLinkbot::Connectors::Simple));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
				_robot[third]->setLED(a, b, c, d);
				_robot[fourth]->setLED(a, b, c, d);
				_robot[fifth]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
				_robot[third]->setName(str);
				_robot[fourth]->setName(str);
				_robot[fifth]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "groupbow") ) {
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::GroupBow, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::GroupBow, _trace));
			int second = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::GroupBow, _trace));
			int third = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::GroupBow, _trace));
			int fourth = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[third]->setID(i + 2);
			_robot[fourth]->setID(i + 3);
			_robot[first]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[second]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[first]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			_robot[second]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[second]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			_robot[third]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[third]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[fourth]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[third]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[third]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[third]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			_robot[fourth]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[fourth]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
				_robot[third]->setLED(a, b, c, d);
				_robot[fourth]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
				_robot[third]->setName(str);
				_robot[fourth]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
				_robot[third]->setPosition(a + 0.1524, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
					_robot[third]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
					_robot[third]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
					_robot[third]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "inchworm") ) {
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Inchworm, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Inchworm, _trace));
			int second = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[first]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[second]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[first]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			_robot[second]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[second]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "lift") ) {
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Lift, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Lift, _trace));
			int second = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Lift, _trace));
			int third = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Lift, _trace));
			int fourth = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[third]->setID(i + 2);
			_robot[fourth]->setID(i + 3);
			_robot[first]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[first]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[second]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[second]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[first]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[second]->addConnector(new Conn(0, rs::Left, -1, 2, 2, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, rs::Left, -1, 2, 2, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[third]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[third]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[third]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[fourth]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[third]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[fourth]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[third]->getID(), 2, rsLinkbot::Connectors::Bridge));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
				_robot[third]->setLED(a, b, c, d);
				_robot[fourth]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
				_robot[third]->setName(str);
				_robot[fourth]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "omnidrive") ) {
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Omnidrive, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Omnidrive, _trace));
			int second = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Omnidrive, _trace));
			int third = _robot.size() - 1;
			_robot[third]->setOrientation(rs::Left);
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Omnidrive, _trace));
			int fourth = _robot.size() - 1;
			_robot[fourth]->setOrientation(rs::Left);
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[third]->setID(i + 2);
			_robot[fourth]->setID(i + 3);
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Omniplate));
			_robot[second]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 2, rsLinkbot::Connectors::Omniplate));
			_robot[third]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 3, rsLinkbot::Connectors::Omniplate));
			_robot[fourth]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[first]->getID(), 4, rsLinkbot::Connectors::Omniplate));
			_robot[first]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[second]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[second]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[third]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[third]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[third]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[fourth]->addConnector(new Conn(0, 0, -1, 1, 1, _robot[fourth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fourth]->addConnector(new Conn(0, 0, rsLinkbot::Connectors::SmallWheel, 1, 1, _robot[fourth]->getID(), 2, rsLinkbot::Connectors::Simple));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
				_robot[third]->setLED(a, b, c, d);
				_robot[fourth]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
				_robot[third]->setName(str);
				_robot[fourth]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "snake") ) {
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Snake, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Snake, _trace));
			int second = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Snake, _trace));
			int third = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Snake, _trace));
			int fourth = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotI, rsLinkbot::Preconfigs::Snake, _trace));
			int fifth = _robot.size() - 1;
			_robot[fifth]->setJoints(-10, 0, 10);
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[third]->setID(i + 2);
			_robot[fourth]->setID(i + 3);
			_robot[fifth]->setID(i + 4);
			_robot[first]->addConnector(new Conn(0, rs::Up, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, rs::Left, rsLinkbot::Connectors::Gripper, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, rs::Right, rsLinkbot::Connectors::Gripper, 3, 3, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[first]->addConnector(new Conn(0, rs::Left, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, rs::Left, -1, 2, 2, _robot[first]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[second]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[second]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[second]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[second]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[third]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[second]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[third]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[second]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[third]->addConnector(new Conn(0, rs::Left, -1, 2, 2, _robot[third]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fourth]->addConnector(new Conn(0, rs::Left, -1, 2, 2, _robot[third]->getID(), 2, rsLinkbot::Connectors::Simple));
			_robot[fourth]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[fourth]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[fourth]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[fourth]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[fifth]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[fourth]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, -1, 3, 3, _robot[fourth]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, -1, 2, 2, _robot[fifth]->getID(), 1, rsLinkbot::Connectors::Simple));
			_robot[fifth]->addConnector(new Conn(0, rs::Right, rsLinkbot::Connectors::Caster, 2, 2, _robot[fifth]->getID(), 2, rsLinkbot::Connectors::Simple));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
				_robot[third]->setLED(a, b, c, d);
				_robot[fourth]->setLED(a, b, c, d);
				_robot[fifth]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
				_robot[third]->setName(str);
				_robot[fourth]->setName(str);
				_robot[fifth]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
		else if ( !strcmp(node->Value(), "stand") ) {
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Stand, _trace));
			int first = _robot.size() - 1;
			_robot.push_back(new Linkbot(rs::LinkbotL, rsLinkbot::Preconfigs::Stand, _trace));
			int second = _robot.size() - 1;
			node->QueryIntAttribute("id", &i);
			_robot[first]->setID(i);
			_robot[second]->setID(i + 1);
			_robot[first]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 1, rsLinkbot::Connectors::Bridge));
			_robot[second]->addConnector(new Conn(0, rs::Left, -1, 1, 1, _robot[first]->getID(), 2, rsLinkbot::Connectors::Bridge));
			_robot[first]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[first]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			_robot[second]->addConnector(new Conn(0, 0, -1, 2, 2, _robot[second]->getID(), 1, rsLinkbot::Connectors::Faceplate));
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot[first]->setLED(a, b, c, d);
				_robot[second]->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot[first]->setName(str);
				_robot[second]->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot[first]->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot[first]->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
					_robot[first]->setRotation(a, b, c, d);
				}
				else {
					_robot[first]->setRotation(0, 0, 0, 1);
				}
			}
		}
#endif
#ifdef RS_MINDSTORMS
		else if ( !strcmp(node->Value(), "ev3") ) {
			_robot.push_back(new Mindstorms(rs::EV3, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
			if ( (ele = node->FirstChildElement("joint")) ) {
				a = 0; b = 0;
				ele->QueryFloatAttribute("w1", &a);
				ele->QueryFloatAttribute("w2", &b);
				_robot.back()->setJoints(a, b);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot.back()->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot.back()->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
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
				ele->QueryFloatAttribute("w1", &a);
				ele->QueryFloatAttribute("w2", &b);
				_robot.back()->setJoints(a, b);
			}
			if ( (ele = node->FirstChildElement("led")) ) {
				a = 0; b = 0; c = 0; d = 0;
				ele->QueryFloatAttribute("r", &a);
				ele->QueryFloatAttribute("g", &b);
				ele->QueryFloatAttribute("b", &c);
				ele->QueryFloatAttribute("alpha", &d);
				_robot.back()->setLED(a, b, c, d);
			}
			if ( (ele = node->FirstChildElement("name")) ) {
				const char *n = ele->GetText();
				std::string str(n ? n : "");
				_robot.back()->setName(str);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				a = 0; b = 0; c = 0;
				ele->QueryFloatAttribute("x", &a);
				ele->QueryFloatAttribute("y", &b);
				ele->QueryFloatAttribute("z", &c);
				_robot.back()->setPosition(a, b, c);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				a = 0; b = 0; c = 0, d = 0;
				if (ele->QueryFloatAttribute("psi", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("theta", &b);
					ele->QueryFloatAttribute("phi", &c);
					_robot.back()->setRotation(rs::D2R(a), rs::D2R(b), rs::D2R(c));
				}
				else if (ele->QueryFloatAttribute("x", &a) != tinyxml2::XML_NO_ATTRIBUTE) {
					ele->QueryFloatAttribute("x", &a);
					ele->QueryFloatAttribute("y", &b);
					ele->QueryFloatAttribute("z", &c);
					ele->QueryFloatAttribute("w", &d);
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
			else if ( !strcmp(node->Value(), "snap") ) {
				ctype = rsDof::Connectors::Snap;
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
				node->QueryFloatAttribute("radius", &size);
			}
#endif
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
							side->QueryFloatAttribute("custom", &size);
						else if (atmp[i] == rsLinkbot::Connectors::Wheel)
							side->QueryFloatAttribute("radius", &size);
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
	if (PathFileExistsA(path.c_str()) != 1) {
		std::string dir(base);
		dir.append("\\C-STEM Studio");
		CreateDirectory(dir.c_str(), NULL);
		dir.append("\\RoboSim");
		CreateDirectory(dir.c_str(), NULL);
	}
#else
	path = getenv("HOME");
	path.append("/.robosimrc");
#endif

	return path;
}

