#include <iostream>
#include <rsXML/Writer>

using namespace rsXML;

Writer::Writer(std::string name, std::string orig) {
	// save filename
	_path = name;

	// load file
	if (_doc.LoadFile(orig.c_str())) {
		std::cerr << "Warning: Could open " << orig << " for reading." << std::endl;
		std::cerr << "         Starting a new XML document at " << name << std::endl;
	}

	// set declaration
	tinyxml2::XMLDeclaration *dec = _doc.NewDeclaration();
	_doc.InsertFirstChild(dec);

	// set configuration options
	tinyxml2::XMLElement *config = _doc.NewElement("config");
	_doc.InsertAfterChild(dec, config);

	// set new version
	//tinyxml2::XMLElement *version = _doc.NewElement("version");
	//version->SetAttribute("val", XML_VERSION);
	//config->InsertFirstChild(version);

	// set graphics options
	tinyxml2::XMLElement *graphics = _doc.NewElement("graphics");
	_doc.InsertAfterChild(config, graphics);

	// set grid values
	tinyxml2::XMLElement *grid = _doc.NewElement("grid");
	grid->SetAttribute("units", 1);
	grid->SetAttribute("enabled", 1);
	grid->SetAttribute("major", 12);
	grid->SetAttribute("tics", 1);
	grid->SetAttribute("minx", -48);
	grid->SetAttribute("maxx", 48);
	grid->SetAttribute("miny", -48);
	grid->SetAttribute("maxy", 48);
	graphics->InsertFirstChild(grid);

	// set tracking of robots
	tinyxml2::XMLElement *tracking = _doc.NewElement("trace");
	tracking->SetAttribute("enabled", 1);
	graphics->InsertAfterChild(grid, tracking);

	// create simulation
	tinyxml2::XMLElement *sim = _doc.NewElement("sim");
	_doc.InsertAfterChild(config, sim);
	sim->SetAttribute("type", 0);

	// write to disk
	this->save();
}

Writer::~Writer(void) {
}

/**********************************************************
	public functions
 **********************************************************/
void Writer::addRobot(int id, int form, double *p, double *q, double *r, double *c) {
	// get simulation element
	tinyxml2::XMLElement *sim = _doc.FirstChildElement("sim");

	// set robot type
	tinyxml2::XMLElement *robot;
	switch (form) {
		case rs::LINKBOTI:
			robot = _doc.NewElement("linkboti");
			break;
		case rs::LINKBOTL:
			robot = _doc.NewElement("linkbotl");
			break;
		case rs::LINKBOTT:
			robot = _doc.NewElement("linkbott");
			break;
		case rs::MINDSTORMS:
			robot = _doc.NewElement("mindstorms");
			break;
		case rs::MOBOT:
			robot = _doc.NewElement("mobot");
			break;
	}
	sim->InsertFirstChild(robot);

	// set id
	robot->SetAttribute("id", id);

	// set position
	tinyxml2::XMLElement *pos = _doc.NewElement("position");
	pos->SetAttribute("x", p[0]);
	pos->SetAttribute("y", p[1]);
	pos->SetAttribute("z", p[2]);
	robot->InsertFirstChild(pos);

	// set rotation
	tinyxml2::XMLElement *rot = _doc.NewElement("rotation");
	rot->SetAttribute("x", q[0]);
	rot->SetAttribute("y", q[1]);
	rot->SetAttribute("z", q[2]);
	rot->SetAttribute("w", q[3]);
	robot->InsertAfterChild(pos, rot);

	// set joints
	tinyxml2::XMLElement *joint = _doc.NewElement("joint");
	switch (form) {
		case rs::LINKBOTI:
		case rs::LINKBOTL:
		case rs::LINKBOTT:
			joint->SetAttribute("f1", r[0]);
			joint->SetAttribute("f2", r[1]);
			joint->SetAttribute("f3", r[2]);
			break;
		case rs::MINDSTORMS:
			joint->SetAttribute("w1", r[0]);
			joint->SetAttribute("w2", r[1]);
			break;
		case rs::MOBOT:
			joint->SetAttribute("a1", r[0]);
			joint->SetAttribute("a2", r[1]);
			joint->SetAttribute("a3", r[2]);
			joint->SetAttribute("a4", r[2]);
			break;
	}
	robot->InsertAfterChild(rot, joint);

	// set led
	tinyxml2::XMLElement *led = _doc.NewElement("led");
	led->SetAttribute("r", c[0]);
	led->SetAttribute("g", c[1]);
	led->SetAttribute("b", c[2]);
	led->SetAttribute("alpha", c[3]);
	robot->InsertAfterChild(joint, led);

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

void Writer::setGrid(bool units, std::vector<double> grid) {
	// clear old grid settings
	tinyxml2::XMLElement *graphics = _doc.FirstChildElement("graphics");
	graphics->DeleteChild(graphics->FirstChildElement("grid"));

	// set new grid
	tinyxml2::XMLElement *g = _doc.NewElement("grid");
	g->SetAttribute("units", units);
	g->SetAttribute("enabled", grid[6]);
	g->SetAttribute("major", grid[0]);
	g->SetAttribute("tics", grid[1]);
	g->SetAttribute("minx", grid[2]);
	g->SetAttribute("maxx", grid[3]);
	g->SetAttribute("miny", grid[4]);
	g->SetAttribute("maxy", grid[5]);

	// add new grid
	graphics->InsertFirstChild(g);

	// write to disk
	this->save();
}

void Writer::save(void) {
	_doc.SaveFile(_path.c_str());
}
