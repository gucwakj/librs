#include <rsSim/linkbot.hpp>
#include <rsSim/Mindstorms.hpp>

#include <rsCallback/callback.hpp>
#include <rsCallback/groundCallback.hpp>
#include <rsCallback/linkbotCallback.hpp>
#include <rsCallback/mindstormsCallback.hpp>

using namespace rsCallback;

Callback::Callback(void) {
	_units = false;
}

void Callback::attachCallback(rsScene::Ground *scene, rsSim::Ground *sim) {
	scene->setUpdateCallback(new groundCallback(sim));
}

void Callback::attachCallback(rsScene::Robot *scene, rsSim::Robot *sim, rsSim::BodyList &bodies) {
	switch (sim->getForm()) {
		case rs::MINDSTORMS:
			scene->setUpdateCallback(new mindstormsCallback(dynamic_cast<rsSim::Mindstorms *>(sim), bodies, _units));
			break;
	}
}

void Callback::attachCallback(rsScene::Robot *scene, rsSim::Robot *sim, rsSim::BodyList &bodies, rsSim::ConnectorList &conn) {
	switch (sim->getForm()) {
		case rs::LINKBOTI:
		case rs::LINKBOTL:
		case rs::LINKBOTT:
			scene->setUpdateCallback(new linkbotCallback(dynamic_cast<rsSim::Linkbot *>(sim), bodies, conn, _units));
			break;
		case rs::MINDSTORMS:
			scene->setUpdateCallback(new mindstormsCallback(dynamic_cast<rsSim::Mindstorms *>(sim), bodies, _units));
			break;
	}
}

void Callback::setUnits(bool units) {
	_units = units;
}

