#include <rsCallback/Callback>
#include <rsCallback/ObstacleCallback>
#include <rsCallback/LinkbotCallback>
#include <rsCallback/MindstormsCallback>

using namespace rsCallback;

Callback::Callback(void) {
	_units = false;
}

void Callback::attachCallback(rsScene::Obstacle *scene, rsSim::Obstacle *sim) {
	scene->setUpdateCallback(new ObstacleCallback(sim));
}

void Callback::attachCallback(rsScene::Robot *scene, rsSim::Robot *sim, rsSim::BodyList &bodies) {
	switch (sim->getForm()) {
		case rs::EV3: case rs::NXT:
			scene->setUpdateCallback(new MindstormsCallback(dynamic_cast<rsSim::Mindstorms *>(sim), bodies, _units));
			break;
	}
}

void Callback::attachCallback(rsScene::Robot *scene, rsSim::Robot *sim, rsSim::BodyList &bodies, rsSim::ConnectorList &conn) {
	switch (sim->getForm()) {
		case rs::LINKBOTI: case rs::LINKBOTL: case rs::LINKBOTT:
			scene->setUpdateCallback(new LinkbotCallback(dynamic_cast<rsSim::Linkbot *>(sim), bodies, conn, _units));
			break;
	}
}

void Callback::setUnits(bool units) {
	_units = units;
}

