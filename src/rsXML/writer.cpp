#include <iostream>
#include <rsXML/Writer>

using namespace rsXML;

Writer::Writer(std::string name, const std::string &orig) {
	// save filename
	_path = name;

	// load file
	if (_doc.LoadFile(orig.c_str())) {
		// warn that the old file couldn't be read
		std::cerr << "Warning: Could not open " << orig << " for reading." << std::endl;
		std::cerr << "         Starting a new XML document at " << name << std::endl;
	}

	// create simulation
	tinyxml2::XMLElement *node;
	if ( !(node = _doc.FirstChildElement("sim")) ) {
		tinyxml2::XMLElement *sim = _doc.NewElement("sim");
		_doc.InsertFirstChild(sim);
	}

	// create graphics
	if ( !(node = _doc.FirstChildElement("graphics")) ) {
		tinyxml2::XMLElement *graphics = _doc.NewElement("graphics");
		_doc.InsertFirstChild(graphics);
	}

	// create ground
	if ( !(node = _doc.FirstChildElement("ground")) ) {
		tinyxml2::XMLElement *ground = _doc.NewElement("ground");
		_doc.InsertFirstChild(ground);
	}

	// create config
	if ( !(node = _doc.FirstChildElement("config")) ) {
		tinyxml2::XMLElement *config = _doc.NewElement("config");
		_doc.InsertFirstChild(config);
	}

	// create declaration
	/*if ( !(node = _doc.FirstChildElement("RoboSim XML Configuration")) ) {
std::cerr << "Here" << std::endl;
		tinyxml2::XMLDeclaration *dec = _doc.NewDeclaration("RoboSim XML Configuration");
		_doc.InsertFirstChild(dec);
	}*/

	// write to disk
	this->save();
}

Writer::~Writer(void) {
	this->save();
}

/**********************************************************
	public functions
 **********************************************************/
void Writer::setMarker(tinyxml2::XMLElement *marker, std::string name, const rs::Pos &p1, const rs::Pos &p2, const rs::Vec &c, int size) {
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
		case rs::DOT:
			marker->SetAttribute("radius", size);
			break;
		case rs::LINE:
			marker->SetAttribute("width", size);
			break;
		case rs::TEXT:
			this->getOrCreateChild(marker, "name")->DeleteChildren();
			this->getOrCreateChild(marker, "name")->InsertFirstChild(this->toText(name));
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

void Writer::setObstacle(tinyxml2::XMLElement *obstacle, std::string name, const rs::Pos &p, const rs::Quat &q, const rs::Vec &l, const rs::Vec &c, int mass) {
	// set attributes
	obstacle->SetAttribute("mass", mass);

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
		case rs::BOX:
			size->SetAttribute("x", l[0]);
			size->SetAttribute("y", l[1]);
			size->SetAttribute("z", l[2]);
			break;
		case rs::CYLINDER:
			size->SetAttribute("length", l[0]);
			size->SetAttribute("radius", l[1]);
			obstacle->SetAttribute("axis", l[2]);
			break;
		case rs::SPHERE:
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
		case rs::LINKBOTI:
		case rs::LINKBOTL:
		case rs::LINKBOTT:
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

void Writer::setRobotWheels(tinyxml2::XMLElement *robot, int wheel1, double size1, int wheel2, double size2) {
	// set wheel type
	tinyxml2::XMLElement *wheel = getOrCreateChild(robot, "wheels");
	wheel->SetAttribute("left", wheel1);
	wheel->SetAttribute("right", wheel2);

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

void Writer::setGrid(std::vector<double> grid) {
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
		case rsLinkbot::SIMPLE:
			node = _doc.NewElement("simple");
			break;
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
	int j;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id) {
			node->SetAttribute("form", form);
			return node;
		}
		node = node->NextSiblingElement();
	}
	switch (form) {
		case rs::DOT:
			node = _doc.NewElement("dot");
			break;
		case rs::LINE:
			node = _doc.NewElement("line");
			break;
		case rs::TEXT:
			node = _doc.NewElement("text");
			break;
	}
	node->SetAttribute("id", id);
	node->SetAttribute("form", form);
	if (id == 0)
		graphics->InsertFirstChild(node);
	else
		graphics->InsertAfterChild(getOrCreateMarker(form, id-1), node);
	return node;
}

tinyxml2::XMLElement* Writer::getOrCreateObstacle(int form, int id) {
	tinyxml2::XMLElement *ground = _doc.FirstChildElement("ground");
	tinyxml2::XMLElement *node = ground->FirstChildElement();
	int j;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id) {
			node->SetAttribute("form", form);
			return node;
		}
		node = node->NextSiblingElement();
	}
	switch (form) {
		case rs::BOX:
			node = _doc.NewElement("box");
			break;
		case rs::CYLINDER:
			node = _doc.NewElement("cylinder");
			break;
		case rs::SPHERE:
			node = _doc.NewElement("sphere");
			break;
	}
	node->SetAttribute("id", id);
	node->SetAttribute("form", form);
	if (id == 0)
		ground->InsertFirstChild(node);
	else
		ground->InsertAfterChild(getOrCreateObstacle(form, id-1), node);
	return node;
}

tinyxml2::XMLElement* Writer::getOrCreateRobot(int form, int id) {
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("sim");
	tinyxml2::XMLElement *node = sim->FirstChildElement();
	int j;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id) {
			node->SetAttribute("form", form);
			switch (form) {
				case rs::LINKBOTI:
					node->SetValue("linkboti");
					break;
				case rs::LINKBOTL:
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
	switch (form) {
		case rs::LINKBOTI:
			node = _doc.NewElement("linkboti");
			node->SetAttribute("form", rs::LINKBOTI);
			break;
		case rs::LINKBOTL:
			node = _doc.NewElement("linkbotl");
			node->SetAttribute("form", rs::LINKBOTL);
			break;
		case rs::LINKBOTT:
			node = _doc.NewElement("linkbott");
			node->SetAttribute("form", rs::LINKBOTT);
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
	if (id == 0)
		sim->InsertFirstChild(node);
	else
		sim->InsertAfterChild(getOrCreateRobot(form, id-1), node);
	return node;
}

tinyxml2::XMLElement* Writer::getOrCreateSide(tinyxml2::XMLElement *p, const char *child, int id) {
	tinyxml2::XMLElement *e = p->FirstChildElement(child);
	int i;
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

tinyxml2::XMLText* Writer::toText(std::string s) {
	return _doc.NewText(s.c_str());
}

