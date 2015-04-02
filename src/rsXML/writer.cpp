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
void Writer::setRobot(tinyxml2::XMLElement *robot, std::string name, double *p, double *q, double *r, double *c) {
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
			joint->SetAttribute("f1", r[0]);
			joint->SetAttribute("f2", r[1]);
			joint->SetAttribute("f3", r[2]);
			break;
		case rs::EV3:
		case rs::NXT:
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

/*
void Writer::addConnector(int id) {
	// add wheels
	tinyxml2::XMLElement *simple1 = _doc.NewElement("simple");
	tinyxml2::XMLElement *s1side1 = _doc.NewElement("side");
	s1side1->SetAttribute("id", 1);
	s1side1->SetAttribute("robot", 0);
	s1side1->SetAttribute("face", 1);
	simple1->InsertFirstChild(s1side1);
	tinyxml2::XMLElement *s1side2 = _doc.NewElement("side");
	s1side2->SetAttribute("id", 2);
	s1side2->SetAttribute("robot", 0);
	s1side2->SetAttribute("conn", SMALLWHEEL);
	simple1->InsertAfterChild(s1side1, s1side2);

	tinyxml2::XMLElement *simple2 = _doc.NewElement("simple");
	tinyxml2::XMLElement *s2side1 = _doc.NewElement("side");
	s2side1->SetAttribute("id", 1);
	s2side1->SetAttribute("robot", 0);
	s2side1->SetAttribute("face", 3);
	simple2->InsertFirstChild(s2side1);
	tinyxml2::XMLElement *s2side2 = _doc.NewElement("side");
	s2side2->SetAttribute("id", 2);
	s2side2->SetAttribute("robot", 0);
	s2side2->SetAttribute("conn", SMALLWHEEL);
	simple2->InsertAfterChild(s2side1, s2side2);

	// add caster
	tinyxml2::XMLElement *simple3 = _doc.NewElement("simple");
	sim->InsertAfterChild(simple2, simple3);
	tinyxml2::XMLElement *s3side1 = _doc.NewElement("side");
	s3side1->SetAttribute("id", 1);
	s3side1->SetAttribute("robot", 0);
	s3side1->SetAttribute("face", 2);
	simple3->InsertFirstChild(s3side1);
	tinyxml2::XMLElement *s3side2 = _doc.NewElement("side");
	s3side2->SetAttribute("id", 2);
	s3side2->SetAttribute("robot", 0);
	s3side2->SetAttribute("conn", CASTER);
	simple3->InsertAfterChild(s3side1, s3side2);

	// add accessories to robot
	sim->InsertAfterChild(robot, simple1);
	sim->InsertAfterChild(simple1, simple2);
	sim->InsertAfterChild(simple2, simple3);
}
*/

tinyxml2::XMLElement* Writer::getRobot(int id) {
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("sim");
	tinyxml2::XMLElement *node = sim->FirstChildElement();
	int j;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id)
			return node;
		node = node->NextSiblingElement();
	}
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

void Writer::save(void) {
	_doc.SaveFile(_path.c_str());
}

tinyxml2::XMLElement* Writer::getOrCreateChild(tinyxml2::XMLElement *p, const char *child) {
	tinyxml2::XMLElement *e = p->FirstChildElement(child);
	if (!e) {
		e = _doc.NewElement(child);
		p->InsertFirstChild(e);
	}
	return e;
}

tinyxml2::XMLElement* Writer::getOrCreateElement(const char *parent, const char *child) {
	tinyxml2::XMLElement *e = _doc.FirstChildElement(parent)->FirstChildElement(child);
	if (!e) {
		e = _doc.NewElement(child);
		_doc.FirstChildElement(parent)->InsertFirstChild(e);
	}
	return e;
}

tinyxml2::XMLElement* Writer::getOrCreateRobot(int form, int id) {
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("sim");
	tinyxml2::XMLElement *node = sim->FirstChildElement();
	int j;
	while (node) {
		node->QueryIntAttribute("id", &j);
		if (j == id)
			return node;
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
	node->SetAttribute("form", form);
	if (id == 0)
		sim->InsertFirstChild(node);
	else
		sim->InsertAfterChild(getOrCreateRobot(form, id-1), node);
	return node;
}

tinyxml2::XMLText* Writer::toText(bool b) {
	return _doc.NewText(std::to_string(b).c_str());
}

tinyxml2::XMLText* Writer::toText(std::string s) {
	return _doc.NewText(s.c_str());
}

