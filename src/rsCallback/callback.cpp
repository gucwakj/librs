#include <rsCallback/Callback>
#include <rsCallback/Obstacle>
#ifdef RS_DOF
#include <rsCallback/Dof>
#endif
#ifdef RS_LINKBOT
#include <rsCallback/Linkbot>
#endif
#ifdef RS_MINDSTORMS
#include <rsCallback/Mindstorms>
#endif

using namespace rsCallback;

Callback::Callback(void) {
	_units = false;
}

void Callback::attachCallback(rsScene::Obstacle *scene, rsSim::Obstacle *sim) {
	scene->setUpdateCallback(new Obstacle(sim));
}

void Callback::attachCallback(rsScene::Group *scene, rsSim::Robot *sim, rsSim::BodyList &bodies) {
	switch (sim->getForm()) {
#ifdef RS_MINDSTORMS
		case rs::EV3: case rs::NXT:
			scene->setUpdateCallback(new Mindstorms(dynamic_cast<rsSim::Mindstorms *>(sim), bodies, _units));
			break;
#endif
	}
}

void Callback::attachCallback(rsScene::Group *scene, rsSim::Robot *sim, rsSim::BodyList &bodies, rsSim::ConnectorList &conn) {
	switch (sim->getForm()) {
#ifdef RS_DOF
		case rs::DOF:
			scene->setUpdateCallback(new Dof(dynamic_cast<rsSim::Dof *>(sim), bodies, conn, _units));
			break;
#endif
#ifdef RS_LINKBOT
		case rs::LINKBOTI: case rs::LINKBOTL: case rs::LINKBOTT:
			scene->setUpdateCallback(new Linkbot(dynamic_cast<rsSim::Linkbot *>(sim), bodies, conn, _units));
			break;
#endif
	}
}

void Callback::setUnits(bool units) {
	_units = units;
}

