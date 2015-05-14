#include <iostream>

#include <rs/Enum>
#include <rsXML/Marker>
#include <rsXML/Obstacle>
#include <rsXML/BackgroundReader>

using namespace rsXML;

BackgroundReader::BackgroundReader(std::string dir) {
	// read XML file
	tinyxml2::XMLDocument doc;
	std::string path = this->load_file(dir, &doc);

	// declare local variables
	tinyxml2::XMLElement *node = NULL;

	// get name
	if ( (node = doc.FirstChildElement("name")) )
		_name = node->GetText();

	// get level
	if ( (node = doc.FirstChildElement("level")) ) {
		if ( !strcmp(node->GetText(), "board") )
			_level = rs::Level::Board;
		else if ( !strcmp(node->GetText(), "outdoors") )
			_level = rs::Level::Outdoors;
	}

	// get screenshot
	if ( (node = doc.FirstChildElement("screenshot")) )
		_screenshot.append(path).append(node->GetText());

	this->read_background(&doc, path);
	this->read_obstacles(&doc);
	this->read_graphics(&doc);
}

BackgroundReader::~BackgroundReader(void) {
}

/**********************************************************
	public functions
 **********************************************************/
std::string BackgroundReader::getBackgroundImage(unsigned int pos) {
	if (_path.size() > pos) return _path[pos];
	return std::string();
}

int BackgroundReader::getLevel(void) {
	return _level;
}

Marker* BackgroundReader::getMarker(int id) {
	return _marker[id];
}

std::string BackgroundReader::getName(void) {
	return _name;
}

int BackgroundReader::getNumObstacles(void) {
	return _obstacle.size();
}

int BackgroundReader::getNumMarkers(void) {
	return _marker.size();
}

Obstacle* BackgroundReader::getObstacle(int id) {
	return _obstacle[id];
}

std::string BackgroundReader::getScreenshot(void) {
	return _screenshot;
}

/**********************************************************
	private functions
 **********************************************************/
std::string BackgroundReader::load_file(std::string dir, tinyxml2::XMLDocument *doc) {
	std::string directory(dir);

	// get file name to load
	std::string file;
	file.append(directory).append("/background.xml");

	// if full path, open and return
	FILE *fp = fopen(file.c_str(), "r");
	if (fp) {
		int output = doc->LoadFile(file.c_str());
		if (output) std::cerr << "Warning: Could not find RoboSim background file.  Using default settings." << std::endl;
		else return directory.append("/");
	}

	// get default file location
	std::string path = rsXML::getDefaultBackgroundPath();
	std::string filepath(path); filepath.append(directory).append("/background.xml");

	// load file
	doc->LoadFile(filepath.c_str());

	// return path of file
	return path.append(directory).append("/");
}

void BackgroundReader::read_background(tinyxml2::XMLDocument *doc, std::string path) {
	// check for existence of node
	tinyxml2::XMLElement *node = doc->FirstChildElement("background");
	if (!node) return;

	// read background images
	_path.resize(7);
	tinyxml2::XMLElement *ele = NULL;
	ele = node->FirstChildElement("ground");
	if (ele) _path[rs::GROUND].append(path).append(ele->GetText());
	ele = node->FirstChildElement("front");
	if (ele) _path[rs::FRONT].append(path).append(ele->GetText());
	ele = node->FirstChildElement("left");
	if (ele) _path[rs::LEFTSIDE].append(path).append(ele->GetText());
	ele = node->FirstChildElement("back");
	if (ele) _path[rs::BACK].append(path).append(ele->GetText());
	ele = node->FirstChildElement("right");
	if (ele) _path[rs::RIGHTSIDE].append(path).append(ele->GetText());
	ele = node->FirstChildElement("top");
	if (ele) _path[rs::TOP].append(path).append(ele->GetText());
	ele = node->FirstChildElement("bottom");
	if (ele) _path[rs::BOTTOM].append(path).append(ele->GetText());
}

void BackgroundReader::read_graphics(tinyxml2::XMLDocument *doc) {
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
			_marker.push_back(new Marker(rs::LINE));
			// id
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
			if ( !node->QueryDoubleAttribute("width", &a) ) {
				a = 0;
				_marker.back()->setSize(a);
			}
		}
		else if ( !strcmp(node->Value(), "dot") ) {
			// create object
			_marker.push_back(new Marker(rs::DOT));
			// id
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
			if ( !node->QueryDoubleAttribute("radius", &a) ) {
				a = 0;
				_marker.back()->setSize(a);
			}
		}
		else if ( !strcmp(node->Value(), "text") ) {
			// create object
			_marker.push_back(new Marker(rs::TEXT));
			// id
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

		// go to next node
		node = node->NextSiblingElement();
	}
}

void BackgroundReader::read_obstacles(tinyxml2::XMLDocument *doc) {
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
			_obstacle.push_back(new Obstacle(rs::BOX));
			// id
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
			if ( !node->QueryDoubleAttribute("mass", &a) ) {
				a = 0;
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
				_obstacle.back()->setRotation(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "cylinder") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::CYLINDER));
			// id
			node->QueryIntAttribute("id", &i);
			_obstacle.back()->setID(i);
			// axis
			if ( !node->QueryDoubleAttribute("axis", &a) ) {
				a = 0;
				_obstacle.back()->setAxis(a);
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
			if ( !node->QueryDoubleAttribute("mass", &a) ) {
				a = 0;
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
				_obstacle.back()->setRotation(a, b, c);
			}
		}
		else if ( !strcmp(node->Value(), "sphere") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::SPHERE));
			// id
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
				a = 0;
				ele->QueryDoubleAttribute("radius", &a);
				_obstacle.back()->setDimensions(a, 0, 0);
			}
			// mass
			if ( !node->QueryDoubleAttribute("mass", &a) ) {
				a = 0;
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

		// go to next node
		node = node->NextSiblingElement();
	}
}

std::string rsXML::getDefaultBackgroundPath(void) {
    std::string path;
#ifdef _WIN32
    DWORD size = 128;
    HKEY key;
#if defined(_WIN64)
    RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\Wow6432Node\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#else
    RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#endif
    char path[128];
    RegQueryValueEx(key, TEXT("CHHOME"), NULL, NULL, (LPBYTE)path, &size);
    path[size] = '\0';
    if (path[0] == '\0')
        path = "C:/Ch";
    else
        path = path;
    path += "/package/chrobosim/data/";
#else
	path = "/home/kgucwa/projects/librs/resources/background/";
#endif
	return path;
}

