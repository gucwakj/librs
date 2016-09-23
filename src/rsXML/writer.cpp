#include <iostream>

#include <config.h>
#include <rsXML/Config>
#include <rsXML/Writer>
#ifdef RS_LINKBOT
#include <rsXML/Linkbot>
#endif

using namespace rsXML;

Writer::Writer(std::string name, const std::string &orig) {
	// save filename
	_path = name;

	// load file
	if (_doc.LoadFile(orig.c_str())) {
		// warn that the old file couldn't be read
		std::cerr << "Warning: Could not open " << orig << " for reading." << std::endl;
		std::cerr << "\tStarting a new XML document at " << name << std::endl;
	}

	// create new nodes
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;

	// create config
	if (!(node = _doc.FirstChildElement("config"))) {
		ele = _doc.NewElement("config");
		_doc.InsertFirstChild(ele);
	}

	// create graphics
	if (!(node = _doc.FirstChildElement("graphics"))) {
		_doc.InsertAfterChild(ele, _doc.NewElement("graphics"));
		ele = _doc.FirstChildElement("graphics");
	}

	// create ground
	if (!(node = _doc.FirstChildElement("ground"))) {
		_doc.InsertAfterChild(ele, _doc.NewElement("ground"));
		ele = _doc.FirstChildElement("ground");
	}

	// create simulation
	if (!(node = _doc.FirstChildElement("sim"))) {
		_doc.InsertAfterChild(ele, _doc.NewElement("sim"));
		ele = _doc.FirstChildElement("sim");
	}

	// create declaration
	tinyxml2::XMLDeclaration *dec = NULL;
	if (!(dec = _doc.FirstChild()->ToDeclaration())) {
		dec = _doc.NewDeclaration("RoboSim XML Configuration");
		_doc.InsertFirstChild(dec);
	}

	// set version number
	this->getOrCreateElement("config", "version")->DeleteChildren();
	this->getOrCreateElement("config", "version")->InsertFirstChild(this->toText(RSXML_VER_CURRENT));

	// write to disk
	this->save();
}

Writer::~Writer(void) {
	this->save();
}

/**********************************************************
	public functions
 **********************************************************/
void Writer::reidRobot(tinyxml2::XMLElement *robot) {
	int i = -1;
	robot->QueryIntAttribute("id", &i);
	robot->SetAttribute("id", i - 1);
}

void Writer::setMarker(tinyxml2::XMLElement *marker, std::string name, const rs::Pos &p1, const rs::Pos &p2, const rs::Vec &c, int size, const rs::Pos &pt) {
	// set start position
	tinyxml2::XMLElement *pos = getOrCreateChild(marker, "position");
	pos->SetAttribute("x", p1[0]);
	pos->SetAttribute("y", p1[1]);
	pos->SetAttribute("z", p1[2]);

	// set end position
	tinyxml2::XMLElement *end = getOrCreateChild(marker, "end");
	end->SetAttribute("x", p2[0]);
	end->SetAttribute("y", p2[1]);
	end->SetAttribute("z", p2[2]);

	// set individual pieces
	switch (marker->IntAttribute("form")) {
		case rs::Arc:
			marker->SetAttribute("width", size);
			break;
		case rs::ArcSector:
			marker->SetAttribute("width", size);
			break;
		case rs::ArcSegment:
			marker->SetAttribute("width", size);
			break;
		case rs::Arrow:
			marker->SetAttribute("width", size);
			break;
		case rs::Circle:
			marker->SetAttribute("width", size);
			break;
		case rs::Dot:
			marker->SetAttribute("radius", size);
			break;
		case rs::Ellipse:
			marker->SetAttribute("width", size);
			break;
		case rs::Line:
			marker->SetAttribute("width", size);
			break;
		case rs::Polygon:
			marker->SetAttribute("width", size);
			break;
		case rs::Quad: {
			marker->SetAttribute("width", size);
			// set point position
			tinyxml2::XMLElement *point = getOrCreateChild(marker, "pt");
			point->SetAttribute("x", pt[0]);
			point->SetAttribute("y", pt[1]);
			break;
		}
		case rs::Rectangle:
			marker->SetAttribute("width", size);
			break;
		case rs::Text:
			this->getOrCreateChild(marker, "name")->DeleteChildren();
			this->getOrCreateChild(marker, "name")->InsertFirstChild(this->toText(name));
			marker->SetAttribute("size", size);
			break;
		case rs::Triangle:
			marker->SetAttribute("width", size);
			break;
	}

	// set led
	tinyxml2::XMLElement *led = getOrCreateChild(marker, "color");
	led->SetAttribute("r", c[0]);
	led->SetAttribute("g", c[1]);
	led->SetAttribute("b", c[2]);
	led->SetAttribute("alpha", c[3]);

	// write to disk
	this->save();
}

void Writer::setObstacle(tinyxml2::XMLElement *obstacle, std::string name, const rs::Pos &p, const rs::Quat &q, const rs::Vec &l, const rs::Vec &c, double mass) {
	// set attributes
	obstacle->SetAttribute("mass", mass);

	// set name
	tinyxml2::XMLElement *n = getOrCreateChild(obstacle, "position");
	n->DeleteChildren();
	n->InsertFirstChild(this->toText(name));

	// set position
	tinyxml2::XMLElement *pos = getOrCreateChild(obstacle, "position");
	pos->SetAttribute("x", p[0]);
	pos->SetAttribute("y", p[1]);
	pos->SetAttribute("z", p[2]);

	// set rotation
	tinyxml2::XMLElement *rot = getOrCreateChild(obstacle, "rotation");
	rot->SetAttribute("x", q[0]);
	rot->SetAttribute("y", q[1]);
	rot->SetAttribute("z", q[2]);
	rot->SetAttribute("w", q[3]);

	// set size
	tinyxml2::XMLElement *size = getOrCreateChild(obstacle, "size");
	switch (obstacle->IntAttribute("form")) {
		case rs::Box:
		case rs::WoodBlock:
			size->SetAttribute("x", l[0]);
			size->SetAttribute("y", l[1]);
			size->SetAttribute("z", l[2]);
			break;
		case rs::CompetitionBorder:
			size->SetAttribute("xlength", l[0]);
			size->SetAttribute("ylength", l[1]);
			size->SetAttribute("radius", l[2]);
			break;
		case rs::Cylinder:
			size->SetAttribute("radius", l[0]);
			size->SetAttribute("length", l[1]);
			obstacle->SetAttribute("axis", l[2]);
			break;
		case rs::Sphere:
			size->SetAttribute("radius", l[0]);
			break;
	}

	// set led
	tinyxml2::XMLElement *led = getOrCreateChild(obstacle, "color");
	led->SetAttribute("r", c[0]);
	led->SetAttribute("g", c[1]);
	led->SetAttribute("b", c[2]);
	led->SetAttribute("alpha", c[3]);

	// write to disk
	this->save();
}

void Writer::setPreconfig(tinyxml2::XMLElement *robot, std::string name, const rs::Pos &p, const rs::Quat &q, const rs::Vec &c) {
	// set position
	tinyxml2::XMLElement *pos = getOrCreateChild(robot, "position");
	pos->SetAttribute("x", p[0]);
	pos->SetAttribute("y", p[1]);
	pos->SetAttribute("z", p[2]);

	// set rotation
	tinyxml2::XMLElement *rot = getOrCreateChild(robot, "rotation");
	rot->SetAttribute("x", q[0]);
	rot->SetAttribute("y", q[1]);
	rot->SetAttribute("z", q[2]);
	rot->SetAttribute("w", q[3]);

	// set led
	tinyxml2::XMLElement *led = getOrCreateChild(robot, "led");
	led->SetAttribute("r", c[0]);
	led->SetAttribute("g", c[1]);
	led->SetAttribute("b", c[2]);
	led->SetAttribute("alpha", c[3]);

	// set name
	this->getOrCreateChild(robot, "name")->DeleteChildren();
	this->getOrCreateChild(robot, "name")->InsertFirstChild(this->toText(name));

	// write to disk
	this->save();
}

void Writer::setRobot(tinyxml2::XMLElement *robot, std::string name, const rs::Pos &p, const rs::Quat &q, const rs::Vec &r, const rs::Vec &c) {
	// set position
	tinyxml2::XMLElement *pos = getOrCreateChild(robot, "position");
	pos->SetAttribute("x", p[0]);
	pos->SetAttribute("y", p[1]);
	pos->SetAttribute("z", p[2]);

	// set rotation
	tinyxml2::XMLElement *rot = getOrCreateChild(robot, "rotation");
	rot->SetAttribute("x", q[0]);
	rot->SetAttribute("y", q[1]);
	rot->SetAttribute("z", q[2]);
	rot->SetAttribute("w", q[3]);

	// set joints
	tinyxml2::XMLElement *joint = getOrCreateChild(robot, "joint");
	switch (robot->IntAttribute("form")) {
		case rs::LinkbotI:
		case rs::LinkbotL:
		case rs::LinkbotT:
			joint->DeleteAttribute("w1");
			joint->DeleteAttribute("w2");
			joint->SetAttribute("f1", r[0]);
			joint->SetAttribute("f2", r[1]);
			joint->SetAttribute("f3", r[2]);
			break;
		case rs::EV3:
		case rs::NXT:
			joint->DeleteAttribute("f1");
			joint->DeleteAttribute("f2");
			joint->DeleteAttribute("f3");
			joint->SetAttribute("w1", r[0]);
			joint->SetAttribute("w2", r[1]);
			break;
	}

	// set led
	tinyxml2::XMLElement *led = getOrCreateChild(robot, "led");
	led->SetAttribute("r", c[0]);
	led->SetAttribute("g", c[1]);
	led->SetAttribute("b", c[2]);
	led->SetAttribute("alpha", c[3]);

	// set name
	this->getOrCreateChild(robot, "name")->DeleteChildren();
	this->getOrCreateChild(robot, "name")->InsertFirstChild(this->toText(name));

	// write to disk
	this->save();
}

void Writer::setRobotWheels(tinyxml2::XMLElement *robot, int wheel1, double size1, int wheel2, double size2, int caster) {
	// set wheel type
	tinyxml2::XMLElement *wheel = getOrCreateChild(robot, "wheels");
	wheel->SetAttribute("left", wheel1);
	wheel->SetAttribute("right", wheel2);
	wheel->SetAttribute("caster", caster);

	// save custom size if applicable
	if (size1) wheel->SetAttribute("radiusLeft", size1);
	if (size2) wheel->SetAttribute("radiusRight", size2);

	// write to disk
	this->save();
}

void Writer::setConnectorSide(tinyxml2::XMLElement *conn, int id, int robotid, int face, int item1, double item2) {
	tinyxml2::XMLElement *side = getOrCreateSide(conn, "side", id);
	side->SetAttribute("id", id);
	side->SetAttribute("robot", robotid);
	if (face)
		side->SetAttribute("face", item1);
	else
		side->SetAttribute("conn", item1);

	if (item2)
		side->SetAttribute("radius", item2);

	// write to disk
	this->save();
}

bool Writer::deleteMarker(int id) {
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("graphics");
	tinyxml2::XMLElement *node = sim->FirstChildElement();
	int j;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id) {
			// delete child
			sim->DeleteChild(node);
			// write to disk
			this->save();
			return true;
		}
		node = node->NextSiblingElement();
	}
	return false;
}

bool Writer::deleteObstacle(int id) {
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("ground");
	tinyxml2::XMLElement *node = sim->FirstChildElement();
	int j;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id) {
			// delete child
			sim->DeleteChild(node);
			// write to disk
			this->save();
			return true;
		}
		node = node->NextSiblingElement();
	}
	return false;
}

bool Writer::deleteRobot(int id) {
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("sim");
	tinyxml2::XMLElement *node = sim->FirstChildElement();
	int j;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id) {
			// delete child
			sim->DeleteChild(node);
			// write to disk
			this->save();
			return true;
		}
		node = node->NextSiblingElement();
	}
	return false;
}

void Writer::setBackground(std::string name) {
	this->getOrCreateElement("config", "background")->DeleteChildren();
	this->getOrCreateElement("config", "background")->InsertFirstChild(this->toText(name));
	this->save();
}

void Writer::setGrid(std::vector<float> grid) {
	// clear old grid settings
	tinyxml2::XMLElement *g = this->getOrCreateElement("graphics", "grid");
	g->DeleteChildren();

	// set new grid
	g->SetAttribute("tics", grid[0]);
	g->SetAttribute("hash", grid[1]);
	g->SetAttribute("minx", grid[2]);
	g->SetAttribute("maxx", grid[3]);
	g->SetAttribute("miny", grid[4]);
	g->SetAttribute("maxy", grid[5]);
	g->SetAttribute("enabled", grid[6]);

	// write to disk
	this->save();
}

void Writer::setTrace(bool trace) {
	this->getOrCreateElement("graphics", "trace")->DeleteChildren();
	this->getOrCreateElement("graphics", "trace")->InsertFirstChild(this->toText(trace));
	this->save();
}

void Writer::setUnits(bool units) {
	this->getOrCreateElement("graphics", "units")->DeleteChildren();
	this->getOrCreateElement("graphics", "units")->InsertFirstChild(this->toText(units));
	this->save();
}

int Writer::save(void) {
	return _doc.SaveFile(_path.c_str());
}

int Writer::saveFile(std::string name) {
	return _doc.SaveFile(name.c_str());
}

tinyxml2::XMLElement* Writer::getOrCreateChild(tinyxml2::XMLElement *p, const char *child) {
	tinyxml2::XMLElement *e = p->FirstChildElement(child);
	if (!e) {
		e = _doc.NewElement(child);
		p->InsertFirstChild(e);
	}
	return e;
}

tinyxml2::XMLElement* Writer::getOrCreateConnector(int form, int orientation) {
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("sim");
	tinyxml2::XMLElement *node = sim->FirstChildElement();
	switch (form) {
#ifdef RS_LINKBOT
		case rsLinkbot::Connectors::Simple:
			node = _doc.NewElement("simple");
			break;
#endif
	}
	node->SetAttribute("orientation", orientation);
	sim->InsertEndChild(node);
	return node;
}

tinyxml2::XMLElement* Writer::getOrCreateElement(const char *parent, const char *child) {
	tinyxml2::XMLElement *e = _doc.FirstChildElement(parent)->FirstChildElement(child);
	if (!e) {
		e = _doc.NewElement(child);
		_doc.FirstChildElement(parent)->InsertFirstChild(e);
	}
	return e;
}

tinyxml2::XMLElement* Writer::getOrCreateMarker(int form, int id) {
	tinyxml2::XMLElement *graphics = _doc.FirstChildElement("graphics");
	tinyxml2::XMLElement *node = graphics->FirstChildElement();
	int j = -1;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id) {
			node->SetAttribute("form", form);
			return node;
		}
		node = node->NextSiblingElement();
	}
	switch (form) {
		case rs::Arc:
			node = _doc.NewElement("arc");
			break;
		case rs::ArcSector:
			node = _doc.NewElement("arcsector");
			break;
		case rs::ArcSegment:
			node = _doc.NewElement("arcsegment");
			break;
		case rs::Arrow:
			node = _doc.NewElement("arrow");
			break;
		case rs::Circle:
			node = _doc.NewElement("circle");
			break;
		case rs::Dot:
			node = _doc.NewElement("dot");
			break;
		case rs::Ellipse:
			node = _doc.NewElement("ellipse");
			break;
		case rs::Line:
			node = _doc.NewElement("line");
			break;
		case rs::Polygon:
			node = _doc.NewElement("polygon");
			break;
		case rs::Quad:
			node = _doc.NewElement("quad");
			break;
		case rs::Rectangle:
			node = _doc.NewElement("rectangle");
			break;
		case rs::Text:
			node = _doc.NewElement("text");
			break;
		case rs::Triangle:
			node = _doc.NewElement("triangle");
			break;
	}
	node->SetAttribute("id", id);
	node->SetAttribute("form", form);
	graphics->InsertFirstChild(node);
	return node;
}

tinyxml2::XMLElement* Writer::getOrCreateObstacle(int form, int id) {
	tinyxml2::XMLElement *ground = _doc.FirstChildElement("ground");
	tinyxml2::XMLElement *node = ground->FirstChildElement();
	int j = -1;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id) {
			node->SetAttribute("form", form);
			return node;
		}
		node = node->NextSiblingElement();
	}
	switch (form) {
		case rs::Box:
			node = _doc.NewElement("box");
			break;
		case rs::CompetitionBorder:
			node = _doc.NewElement("competitionborder");
			break;
		case rs::Cylinder:
			node = _doc.NewElement("cylinder");
			break;
		case rs::HackySack:
			node = _doc.NewElement("hackysack");
			break;
		case rs::PullupBar:
			node = _doc.NewElement("pullupbar");
			break;
		case rs::Sphere:
			node = _doc.NewElement("sphere");
			break;
		case rs::WoodBlock:
			node = _doc.NewElement("woodblock");
			break;
	}
	node->SetAttribute("id", id);
	node->SetAttribute("form", form);
	ground->InsertFirstChild(node);
	return node;
}

tinyxml2::XMLElement* Writer::getOrCreatePreconfig(int form, int shape, int id) {
	// get robot elements
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("sim");
	tinyxml2::XMLElement *node = sim->FirstChildElement();

	// find old node with 'id'
	int i = -1;
	while (node) {
		node->QueryIntAttribute("id", &i);
		if (i == id) return node;
		node = node->NextSiblingElement();
	}

	// create new node if one matching 'id' is not found
	switch (shape) {
#ifdef RS_LINKBOT
		case rsLinkbot::Preconfigs::Bow:
			node = _doc.NewElement("bow");
			break;
		case rsLinkbot::Preconfigs::BugClock:
			node = _doc.NewElement("bugclock");
			break;
		case rsLinkbot::Preconfigs::Explorer:
			node = _doc.NewElement("explorer");
			break;
		case rsLinkbot::Preconfigs::FourBotDrive:
			node = _doc.NewElement("fourbotdrive");
			break;
		case rsLinkbot::Preconfigs::FourWheelDrive:
			node = _doc.NewElement("fourwheeldrive");
			break;
		case rsLinkbot::Preconfigs::FourWheelExplorer:
			node = _doc.NewElement("fourwheelexplorer");
			break;
		case rsLinkbot::Preconfigs::GroupBow:
			node = _doc.NewElement("groupbow");
			break;
		case rsLinkbot::Preconfigs::Inchworm:
			node = _doc.NewElement("inchworm");
			break;
		case rsLinkbot::Preconfigs::Lift:
			node = _doc.NewElement("lift");
			break;
		case rsLinkbot::Preconfigs::Omnidrive:
			node = _doc.NewElement("omnidrive");
			break;
		case rsLinkbot::Preconfigs::Snake:
			node = _doc.NewElement("snake");
			break;
		case rsLinkbot::Preconfigs::Stand:
			node = _doc.NewElement("stand");
			break;
#endif
	}
	node->SetAttribute("id", id);
	node->SetAttribute("form", form);

	// insert after node with id = id-1
	tinyxml2::XMLElement *prev_node = sim->FirstChildElement();
	while (prev_node) {
		int i = -1;
		prev_node->QueryIntAttribute("id", &i);
		if (i == id - 1) {
			sim->InsertAfterChild(prev_node, node);
			return node;
		}
		prev_node = prev_node->NextSiblingElement();
	}

	// insert at beginning
	sim->InsertFirstChild(node);
	return node;
}

tinyxml2::XMLElement* Writer::getOrCreateRobot(int form, int id) {
	// get robot elements
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("sim");
	tinyxml2::XMLElement *node = sim->FirstChildElement();

	// reform old node with 'id' to new robot form
	int i = -1;
	while (node) {
		node->QueryIntAttribute("id", &i);
		if (i == id) {
			node->SetAttribute("form", form);
			switch (form) {
				case rs::LinkbotI:
					node->SetValue("linkboti");
					break;
				case rs::LinkbotL:
					node->SetValue("linkbotl");
					break;
				case rs::EV3:
					node->SetValue("ev3");
					break;
				case rs::NXT:
					node->SetValue("nxt");
					break;
			}
			return node;
		}
		node = node->NextSiblingElement();
	}

	// create new node if one matching 'id' is not found
	switch (form) {
		case rs::LinkbotI:
			node = _doc.NewElement("linkboti");
			node->SetAttribute("form", rs::LinkbotI);
			break;
		case rs::LinkbotL:
			node = _doc.NewElement("linkbotl");
			node->SetAttribute("form", rs::LinkbotL);
			break;
		case rs::LinkbotT:
			node = _doc.NewElement("linkbott");
			node->SetAttribute("form", rs::LinkbotT);
			break;
		case rs::EV3:
			node = _doc.NewElement("ev3");
			node->SetAttribute("form", rs::EV3);
			break;
		case rs::NXT:
			node = _doc.NewElement("nxt");
			node->SetAttribute("form", rs::NXT);
			break;
	}
	node->SetAttribute("id", id);
	node->SetAttribute("form", form);

	// insert after node with id = id-1
	tinyxml2::XMLElement *prev_node = sim->FirstChildElement();
	while (prev_node) {
		int i = -1;
		prev_node->QueryIntAttribute("id", &i);
		if (i == id - 1) {
			sim->InsertAfterChild(prev_node, node);
			return node;
		}
		prev_node = prev_node->NextSiblingElement();
	}

	// insert at beginning
	sim->InsertFirstChild(node);
	return node;
}

tinyxml2::XMLElement* Writer::getOrCreateSide(tinyxml2::XMLElement *p, const char *child, int id) {
	tinyxml2::XMLElement *e = p->FirstChildElement(child);
	int i = -1;
	if (!e) {
		e = _doc.NewElement(child);
		p->InsertEndChild(e);
	}
	else {
		while (e) {
			e->QueryIntAttribute("id", &i);
			if (i == id)
				return e;
			e = e->NextSiblingElement();
		}
		if (!e) {
			e = _doc.NewElement(child);
			p->InsertEndChild(e);
		}
	}
	return e;
}

tinyxml2::XMLText* Writer::toText(bool b) {
	return _doc.NewText(std::to_string(b).c_str());
}

tinyxml2::XMLText* Writer::toText(int i) {
	return _doc.NewText(std::to_string(i).c_str());
}

tinyxml2::XMLText* Writer::toText(std::string s) {
	return _doc.NewText(s.c_str());
}

