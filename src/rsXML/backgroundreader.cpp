#include <iostream>

#include <tinyxml2.h>

#include <rs/Enum>
#include <rs/Macros>
#include <rsXML/Marker>
#include <rsXML/Obstacle>
#include <rsXML/BackgroundReader>

using namespace rsXML;

BackgroundReader::BackgroundReader(std::string path, std::string folder) {
	// read XML file
	tinyxml2::XMLDocument doc;
	_path = this->load_file(path, folder, &doc);

	// declare local variables
	tinyxml2::XMLElement *node = NULL;

	// get name
	if ( (node = doc.FirstChildElement("name")) )
		_name = node->GetText();

	// get level
	if ((node = doc.FirstChildElement("level"))) {
		if (!strcmp(node->GetText(), "none"))
			_level = rs::Level::None;
		else if ( !strcmp(node->GetText(), "activitymat") )
			_level = rs::Level::ActivityMat;
		else if ( !strcmp(node->GetText(), "board") )
			_level = rs::Level::Board;
		else if ( !strcmp(node->GetText(), "outdoors") )
			_level = rs::Level::Outdoors;
		else if ( !strcmp(node->GetText(), "RPC2014") )
			_level = rs::Level::RPC2014;
		else if ( !strcmp(node->GetText(), "RPC2015") )
			_level = rs::Level::RPC2015;
		else if ( !strcmp(node->GetText(), "RPC2016") )
			_level = rs::Level::RPC2016;
		else if ( !strcmp(node->GetText(), "RPC2017") )
			_level = rs::Level::RPC2017;
	}

	// get screenshot
	if ( (node = doc.FirstChildElement("screenshot")) )
		_screenshot.append(_path).append(folder).append("/").append(node->GetText());

	this->read_background(&doc, _path, folder);
	this->read_obstacles(&doc);
	this->read_graphics(&doc);
}

BackgroundReader::~BackgroundReader(void) { }

/**********************************************************
	public functions
 **********************************************************/
std::string BackgroundReader::getBackgroundImage(unsigned short pos) {
	if (_imgs.size() > pos) return _imgs[pos];
	return std::string();
}

short BackgroundReader::getLevel(void) {
	return _level;
}

Marker* BackgroundReader::getMarker(short id) {
	return _marker[id];
}

std::string BackgroundReader::getName(void) {
	return _name;
}

short BackgroundReader::getNumObstacles(void) {
	return _obstacle.size();
}

short BackgroundReader::getNumMarkers(void) {
	return _marker.size();
}

Obstacle* BackgroundReader::getObstacle(short id) {
	return _obstacle[id];
}

std::string BackgroundReader::getPath(void) {
	return _path;
}

std::string BackgroundReader::getScreenshot(void) {
	return _screenshot;
}

/**********************************************************
	private functions
 **********************************************************/
std::string BackgroundReader::load_file(std::string path, std::string folder, tinyxml2::XMLDocument *doc) {
	// get file name to load
	std::string file;
	file.append(path).append(folder).append("/background.xml");

	// if full path, open and return
	FILE *fp = fopen(file.c_str(), "r");
	if (fp) {
		int output = doc->LoadFile(file.c_str());
		if (output) std::cerr << "Warning: Could not find RoboSim background file.  Using default settings." << std::endl;
		else return path;
	}

	// get default file location
	path = rsXML::getDefaultBackgroundPath();
	std::string filepath(path);
	filepath.append(folder).append("/background.xml");

	// load file
	doc->LoadFile(filepath.c_str());

	// return path of file
	return path;
}

void BackgroundReader::read_background(tinyxml2::XMLDocument *doc, std::string path, std::string folder) {
	// check for existence of node
	tinyxml2::XMLElement *node = doc->FirstChildElement("background");
	if (!node) return;

	// read background images
	_imgs.resize(7);
	tinyxml2::XMLElement *ele = NULL;
	ele = node->FirstChildElement("ground");
	if (ele) _imgs[rs::Ground].append(path).append(folder).append("/").append(ele->GetText());
	ele = node->FirstChildElement("front");
	if (ele) _imgs[rs::Front].append(path).append(folder).append("/").append(ele->GetText());
	ele = node->FirstChildElement("left");
	if (ele) _imgs[rs::LeftSide].append(path).append(folder).append("/").append(ele->GetText());
	ele = node->FirstChildElement("back");
	if (ele) _imgs[rs::Back].append(path).append(folder).append("/").append(ele->GetText());
	ele = node->FirstChildElement("right");
	if (ele) _imgs[rs::RightSide].append(path).append(folder).append("/").append(ele->GetText());
	ele = node->FirstChildElement("top");
	if (ele) _imgs[rs::Top].append(path).append(folder).append("/").append(ele->GetText());
	ele = node->FirstChildElement("bottom");
	if (ele) _imgs[rs::Bottom].append(path).append(folder).append("/").append(ele->GetText());
}

void BackgroundReader::read_graphics(tinyxml2::XMLDocument *doc) {
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
		if ( !strcmp(node->Value(), "line") ) {
			// create object
			_marker.push_back(new Marker(rs::Line));
			// id
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
			a = 0;
			if ( !node->QueryIntAttribute("width", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "dot") ) {
			// create object
			_marker.push_back(new Marker(rs::Dot));
			// id
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
			a = 0;
			if ( !node->QueryIntAttribute("radius", &i) ) {
				_marker.back()->setSize(i);
			}
		}
		else if ( !strcmp(node->Value(), "text") ) {
			// create object
			_marker.push_back(new Marker(rs::Text));
			// id
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

		// go to next node
		node = node->NextSiblingElement();
	}
}

void BackgroundReader::read_obstacles(tinyxml2::XMLDocument *doc) {
	// declare local variables
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	int i;
	float a, b, c, d;

	// check for existence of node
	if ( (node = doc->FirstChildElement("obstacles")) ) {
		node = node->FirstChildElement();
	}

	// loop over all nodes
	while (node) {
		if ( !strcmp(node->Value(), "box") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::Box));
			// id
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
		else if ( !strcmp(node->Value(), "cylinder") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::Cylinder));
			// id
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
		else if ( !strcmp(node->Value(), "sphere") ) {
			// create object
			_obstacle.push_back(new Obstacle(rs::Sphere));
			// id
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

		// go to next node
		node = node->NextSiblingElement();
	}
}

std::string rsXML::getDefaultBackgroundPath(void) {
	std::string path;
#ifdef RS_WIN32
	DWORD size = 128;
	HKEY key;
#if defined(_WIN64)
	RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\Wow6432Node\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#else
	RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#endif
	char base[128];
	RegQueryValueEx(key, TEXT("CHHOME"), NULL, NULL, (LPBYTE)base, &size);
	base[size] = '\0';
	if (base[0] == '\0')
		path = "C:/Ch";
	else
		path = base;
	path += "/package/chrobosim/data/background/";
#else
	path = "/usr/local/ch/package/chrobosim/data/background/";
#endif
	return path;
}

std::string rsXML::getDefaultChallengePath(void) {
	std::string path;
#ifdef RS_WIN32
	DWORD size = 128;
	HKEY key;
#if defined(_WIN64)
	RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\Wow6432Node\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#else
	RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#endif
	char base[128];
	RegQueryValueEx(key, TEXT("CHHOME"), NULL, NULL, (LPBYTE)base, &size);
	base[size] = '\0';
	if (base[0] == '\0')
		path = "C:/Ch";
	else
		path = base;
	path += "/package/chrobosim/data/challenges/";
#else
	path = "/usr/local/ch/package/chrobosim/data/challenges/";
#endif
	return path;
}

