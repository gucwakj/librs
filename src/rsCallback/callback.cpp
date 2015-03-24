#include <rsCallback/Callback>
#include <rsCallback/GroundCallback>
#include <rsCallback/LinkbotCallback>
#include <rsCallback/MindstormsCallback>

using namespace rsCallback;

Callback::Callback(void) {
	_units = false;
}

void Callback::attachCallback(rsScene::Ground *scene, rsSim::Ground *sim) {
	scene->setUpdateCallback(new GroundCallback(sim));
}

void Callback::attachCallback(rsScene::Robot *scene, rsSim::Robot *sim, rsSim::BodyList &bodies) {
	switch (sim->getForm()) {
#ifdef ENABLE_MINDSTORMS
		case rs::EV3: case rs::NXT:
			scene->setUpdateCallback(new MindstormsCallback(dynamic_cast<rsSim::Mindstorms *>(sim), bodies, _units));
			break;
#endif
	}
}

void Callback::attachCallback(rsScene::Robot *scene, rsSim::Robot *sim, rsSim::BodyList &bodies, rsSim::ConnectorList &conn) {
	switch (sim->getForm()) {
#ifdef ENABLE_LINKBOT
		case rs::LINKBOTI: case rs::LINKBOTL: case rs::LINKBOTT:
			scene->setUpdateCallback(new LinkbotCallback(dynamic_cast<rsSim::Linkbot *>(sim), bodies, conn, _units));
			break;
#endif
	}
}

void Callback::setUnits(bool units) {
	_units = units;
}

