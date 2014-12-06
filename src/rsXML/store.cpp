#include "store.hpp"

using namespace rsXML;

Store::Store(char *name) {
	// set default values
	_cor[0] = 0;
	_cor[1] = 0;
	_mu[0] = 0;
	_mu[1] = 0;
	_ps = 3;
	_preconfig = 0;
	_rt = 1;
	_trace = false;
	_us = 1;

	// read XML file
	tinyxml2::XMLDocument doc;
	this->load_file(name, &doc);
	this->read_config(&doc);
	this->read_ground(&doc);
	this->read_graphics(&doc);
	this->read_sim(&doc);
}

Store::~Store(void) {
std::cerr << "deleting Store" << std::endl;
	for (int i = 0; i < _robot.size(); i++) {
		delete _robot[i];
	}
}

/**********************************************************
	public functions
 **********************************************************/
int Store::addNewRobot(Robot *bot) {
	_robot.push_back(bot);
	return 0;
}

Robot* Store::getNextRobot(int form) {
	// find next robot in xml list
	int i = 0;
	for (i = 0; i < _robot.size(); i++) {
		if (_robot[i]->getForm() == form && !_robot[i]->getConnect())
			break;
	}

	// no robot found
	if (i == _robot.size() || _robot[i]->getForm() != form) {
		switch (form) {
			case rs::CUBUS:
				fprintf(stderr, "Error: Could not find CUBUS in RoboSim GUI.\n");
				break;
			case rs::LINKBOTI:
				fprintf(stderr, "Error: Could not find LinkbotI in RoboSim GUI.\n");
				break;
			case rs::LINKBOTL:
				fprintf(stderr, "Error: Could not find LinkbotL in RoboSim GUI.\n");
				break;
			case rs::LINKBOTT:
				fprintf(stderr, "Error: Could not find LinkbotT in RoboSim GUI.\n");
				break;
			case rs::MOBOT:
				fprintf(stderr, "Error: Could not find Mobot in RoboSim GUI.\n");
				break;
			case rs::NXT:
				fprintf(stderr, "Error: Could not find NXT in RoboSim GUI.\n");
				break;
		}
		if (_preconfig) {
			fprintf(stderr, "       Preconfigured Robot Configuration selected.\n");
			fprintf(stderr, "       Please uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	// robot now connected
	_robot[i]->setConnect(1);

	// success
	return _robot[i];
}

const double* Store::getGrid(void) {
	return _grid;
}

int Store::getNumGrounds(void) {
	return _ground.size();
}

int Store::getNumMarkers(void) {
	return _marker.size();
}

bool Store::getUnits(void) {
	return _us;
}

/**********************************************************
	private functions
 **********************************************************/
void Store::load_file(char *name, tinyxml2::XMLDocument *doc) {
	char path[512];
#ifdef _WIN32
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path))) {
		strcat(path, "\\robosimrc");
	}
#else
	strcpy(path, getenv("HOME"));
	if (name) {
		FILE *fp = fopen(name, "r");
		if (fp) {
			strcpy(path, name);
			fclose(fp);
		}
		else {
			strcat(path, "/.robosimrc");
		}
	}
	else {
		strcat(path, "/.robosimrc");
	}
#endif
	int output = doc->LoadFile(path);
	if (output) {
		fprintf(stderr, "Error: Could not find RoboSim config file.\nPlease run RoboSim GUI.\n");
		exit(-1);
	}
}

void Store::read_config(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;

	// check for custom mu params
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("mu")) ) {
		node->QueryDoubleAttribute("ground", &(_mu[0]));
		node->QueryDoubleAttribute("body", &(_mu[1]));
	}

	// check for custom cor params
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("cor")) ) {
		node->QueryDoubleAttribute("ground", &(_cor[0]));
		node->QueryDoubleAttribute("body", &(_cor[1]));
	}

	// check if should start paused
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("pause")) ) {
		node->QueryIntAttribute("val", &_ps);
	}

	// check if should run in real time
	if ( (node = doc->FirstChildElement("config")->FirstChildElement("realtime")) ) {
		node->QueryIntAttribute("val", &_rt);
	}
}

void Store::read_graphics(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;

	// check for existence of node
	if ( (node = doc->FirstChildElement("graphics")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if ( !strcmp(node->Value(), "line") ) {
			// create object
			_marker.push_back(new Marker());
			_marker.back()->type = rs::LINE;

			// get user defined values from xml
			if (node->QueryIntAttribute("width", &_marker.back()->size)) {
				_marker.back()->size = 1;
			}
			if ( (ele = node->FirstChildElement("start")) ) {
				ele->QueryDoubleAttribute("x", &_marker.back()->p[0]);
				ele->QueryDoubleAttribute("y", &_marker.back()->p[1]);
				ele->QueryDoubleAttribute("z", &_marker.back()->p[2]);
			}
			if ( (ele = node->FirstChildElement("end")) ) {
				ele->QueryDoubleAttribute("x", &_marker.back()->p[3]);
				ele->QueryDoubleAttribute("y", &_marker.back()->p[4]);
				ele->QueryDoubleAttribute("z", &_marker.back()->p[5]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_marker.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_marker.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_marker.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_marker.back()->c[3]);
			}
		}
		else if ( !strcmp(node->Value(), "point") ) {
			// create object
			_marker.push_back(new Marker());
			_marker.back()->type = rs::DOT;

			// get user defined values from xml
			if (node->QueryIntAttribute("size", &_marker.back()->size)) {
				_marker.back()->size = 1;
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &_marker.back()->p[0]);
				ele->QueryDoubleAttribute("y", &_marker.back()->p[1]);
				ele->QueryDoubleAttribute("z", &_marker.back()->p[2]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_marker.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_marker.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_marker.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_marker.back()->c[3]);
			}
		}
		else if ( !strcmp(node->Value(), "grid") ) {
			node->QueryIntAttribute("units", &_us);
			node->QueryDoubleAttribute("tics", &_grid[0]);
			node->QueryDoubleAttribute("major", &_grid[1]);
			node->QueryDoubleAttribute("minx", &_grid[2]);
			node->QueryDoubleAttribute("maxx", &_grid[3]);
			node->QueryDoubleAttribute("miny", &_grid[4]);
			node->QueryDoubleAttribute("maxy", &_grid[5]);
			node->QueryDoubleAttribute("enabled", &_grid[6]);
			for (int i = 0; i < 6; i++) {
				if (_us)_grid[i] /= 39.37;
				else _grid[i] /= 100;
			}
		}
		else if ( !strcmp(node->Value(), "tracking") ) {
			node->QueryIntAttribute("val", (int*)(&_trace));
		}
		else {
			// create object
			_marker.push_back(new Marker());
			_marker.back()->type = rs::TEXT;

			// get user defined values from xml
			_marker.back()->s = node->Value();
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &_marker.back()->p[0]);
				ele->QueryDoubleAttribute("y", &_marker.back()->p[1]);
				ele->QueryDoubleAttribute("z", &_marker.back()->p[2]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_marker.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_marker.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_marker.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_marker.back()->c[3]);
			}
		}
		// go to next node
		node = node->NextSiblingElement();
	}
}

void Store::read_ground(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;

	// check for existence of node
	if ( (node = doc->FirstChildElement("ground")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if ( !strcmp(node->Value(), "box") ) {
			// store default variables
			_ground.push_back(new Ground());
			_ground.back()->type = rs::BOX;
			_ground.back()->c[0] = 0;
			_ground.back()->c[1] = 0;
			_ground.back()->c[2] = 0;
			_ground.back()->c[3] = 1;

			// get user defined values from xml
			if (node->QueryDoubleAttribute("mass", &_ground.back()->mass)) {
				_ground.back()->mass = 0.1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_ground.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_ground.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_ground.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_ground.back()->c[3]);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_ground.back()->p[0]));
				ele->QueryDoubleAttribute("y", &(_ground.back()->p[1]));
				ele->QueryDoubleAttribute("z", &(_ground.back()->p[2]));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(_ground.back()->r[0]));
				ele->QueryDoubleAttribute("theta", &(_ground.back()->r[1]));
				ele->QueryDoubleAttribute("phi", &(_ground.back()->r[2]));
			}
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("x", &(_ground.back()->l[0]));
				ele->QueryDoubleAttribute("y", &(_ground.back()->l[1]));
				ele->QueryDoubleAttribute("z", &(_ground.back()->l[2]));
			}
		}
		else if ( !strcmp(node->Value(), "cylinder") ) {
			// store default variables
			_ground.push_back(new Ground());
			_ground.back()->c[0] = 0;
			_ground.back()->c[1] = 0;
			_ground.back()->c[2] = 0;
			_ground.back()->c[3] = 1;

			// get user defined values from xml
			if (node->QueryDoubleAttribute("mass", &_ground.back()->mass)) {
				_ground.back()->mass = 0.1;
			}
			if (node->QueryDoubleAttribute("axis", &_ground.back()->axis)) {
				_ground.back()->axis = 1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_ground.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_ground.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_ground.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_ground.back()->c[3]);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_ground.back()->p[0]));
				ele->QueryDoubleAttribute("y", &(_ground.back()->p[1]));
				ele->QueryDoubleAttribute("z", &(_ground.back()->p[2]));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(_ground.back()->r[0]));
				ele->QueryDoubleAttribute("theta", &(_ground.back()->r[1]));
				ele->QueryDoubleAttribute("phi", &(_ground.back()->r[2]));
			}
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &(_ground.back()->l[0]));
				ele->QueryDoubleAttribute("length", &(_ground.back()->l[1]));
			}
		}
		else if ( !strcmp(node->Value(), "sphere") ) {
			// store default variables
			_ground.push_back(new Ground());
			_ground.back()->c[0] = 0;
			_ground.back()->c[1] = 0;
			_ground.back()->c[2] = 0;
			_ground.back()->c[3] = 1;

			// get user defined values from xml
			if (node->QueryDoubleAttribute("mass", &_ground.back()->mass)) {
				_ground.back()->mass = 0.1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_ground.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_ground.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_ground.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_ground.back()->c[3]);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_ground.back()->p[0]));
				ele->QueryDoubleAttribute("y", &(_ground.back()->p[1]));
				ele->QueryDoubleAttribute("z", &(_ground.back()->p[2]));
			}
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &(_ground.back()->l[0]));
			}
		}

		// go to next node
		node = node->NextSiblingElement();
	}
}

void Store::read_sim(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	tinyxml2::XMLElement *side = NULL;
	int *rtmp, *ftmp, *ntmp, *atmp, ctype = 0, cnum = 0;
	int custom = 0, i = 0;
	double size = 0, a, b, c, d, e, f;

	// check for existence of node
	if ( (node = doc->FirstChildElement("sim")) ) {
		node = node->FirstChildElement();
	}

	// check if individual vs preconfig
	node->QueryIntAttribute("type", &_preconfig);

	// loop over all nodes
	while (node) {
		if (node->ToComment()) {}
		else if ( !strcmp(node->Value(), "cubus") ) {
			_robot.push_back(new Robot(rs::CUBUS, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
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
				_robot.back()->setRotation(a, b, c);
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("a1", &a);
				ele->QueryDoubleAttribute("a2", &b);
				ele->QueryDoubleAttribute("a3", &c);
				ele->QueryDoubleAttribute("a4", &d);
				ele->QueryDoubleAttribute("a5", &e);
				ele->QueryDoubleAttribute("a6", &f);
				_robot.back()->setJoints(a, b, c, d, e, f);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? i : -1;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "linkboti") ) {
			_robot.push_back(new Robot(rs::LINKBOTI, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
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
				_robot.back()->setRotation(a, b, c);
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if (!node->QueryIntAttribute("orientation", &i)) {
				if (i == 1)
					_robot.back()->setPsi(0);
				else if (i == 2)
					_robot.back()->setPsi(M_PI/2);
				else if (i == 3)
					_robot.back()->setPsi(M_PI);
				else if (i == 4)
					_robot.back()->setPsi(3*M_PI/2);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? i : -1;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "linkbotl") ) {
			_robot.push_back(new Robot(rs::LINKBOTL, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
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
				_robot.back()->setRotation(a, b, c);
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if (!node->QueryIntAttribute("orientation", &i)) {
				if (i == 1)
					_robot.back()->setPsi(0);
				else if (i == 2)
					_robot.back()->setPsi(M_PI/2);
				else if (i == 3)
					_robot.back()->setPsi(M_PI);
				else if (i == 4)
					_robot.back()->setPsi(3*M_PI/2);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? i : -1;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "linkbott") ) {
			_robot.push_back(new Robot(rs::LINKBOTT, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
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
				_robot.back()->setRotation(a, b, c);
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &a);
				ele->QueryDoubleAttribute("f2", &b);
				ele->QueryDoubleAttribute("f3", &c);
				_robot.back()->setJoints(a, b, c);
			}
			if (!node->QueryIntAttribute("orientation", &i)) {
				if (i == 1)
					_robot.back()->setPsi(0);
				else if (i == 2)
					_robot.back()->setPsi(M_PI/2);
				else if (i == 3)
					_robot.back()->setPsi(M_PI);
				else if (i == 4)
					_robot.back()->setPsi(3*M_PI/2);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? i : -1;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "mobot") ) {
			_robot.push_back(new Robot(rs::MOBOT, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
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
				_robot.back()->setRotation(a, b, c);
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("a1", &a);
				ele->QueryDoubleAttribute("a2", &b);
				ele->QueryDoubleAttribute("a3", &c);
				ele->QueryDoubleAttribute("a4", &d);
				_robot.back()->setJoints(a, b, c, d);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? i : -1;
			_robot.back()->setGround(i);
		}
		else if ( !strcmp(node->Value(), "nxt") ) {
			_robot.push_back(new Robot(rs::NXT, _trace));
			node->QueryIntAttribute("id", &i);
			_robot.back()->setID(i);
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
				_robot.back()->setRotation(a, b, c);
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("w1", &a);
				ele->QueryDoubleAttribute("w2", &b);
				_robot.back()->setJoints(a, b);
			}
			i = (node->QueryIntAttribute("ground", &i)) ? i : -1;
			_robot.back()->setGround(i);
		}
		else {
			if ( !strcmp(node->Value(), "bigwheel") ) {
				ctype = rs::BIGWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "bridge") ) {
				ctype = rs::BRIDGE;
				cnum = 2;
			}
			else if ( !strcmp(node->Value(), "caster") ) {
				ctype = rs::CASTER;
				cnum = 1;
				node->QueryIntAttribute("custom", &custom);
			}
			else if ( !strcmp(node->Value(), "cube") ) {
				ctype = rs::CUBE;
				cnum = 5;
			}
			else if ( !strcmp(node->Value(), "faceplate") ) {
				ctype = rs::FACEPLATE;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "gripper") ) {
				ctype = rs::GRIPPER;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "l") ) {
				ctype = rs::L;
				cnum = 3;
			}
			else if ( !strcmp(node->Value(), "omnidrive") ) {
				ctype = rs::OMNIDRIVE;
				cnum = 4;
			}
			else if ( !strcmp(node->Value(), "simple") ) {
				ctype = rs::SIMPLE;
				cnum = 2;
			}
			else if ( !strcmp(node->Value(), "smallwheel") ) {
				ctype = rs::SMALLWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "square") ) {
				ctype = rs::SQUARE;
				cnum = 4;
			}
			else if ( !strcmp(node->Value(), "tank") ) {
				ctype = rs::TANK;
				cnum = 3;
			}
			else if ( !strcmp(node->Value(), "tinywheel") ) {
				ctype = rs::TINYWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "wheel") ) {
				ctype = rs::WHEEL;
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
						if (atmp[i] == rs::CASTER)
							side->QueryDoubleAttribute("custom", &size);
						else if (atmp[i] == rs::WHEEL)
							side->QueryDoubleAttribute("radius", &size);
					}
					i++;
					side = side->NextSiblingElement();
				}
			}

			// store connectors to each robot
			for (int j = 0; j < i; j++) {
				for (int k = 0; k < _robot.size(); k++) {
					if (_robot[k]->getID() == rtmp[j])
						_robot[k]->addConnector(new Conn(size, atmp[j], ftmp[0], ftmp[j], rtmp[0], ntmp[j], ctype));
				}
			}

			// delete temporary arrays
			delete [] rtmp;
			delete [] ftmp;
			delete [] ntmp;
			delete [] atmp;
		}

		// go to next node
		node = node->NextSiblingElement();
	}
}

