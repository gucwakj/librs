#include <rsCallback/Callback>
#include <rsCallback/Obstacle>

using namespace rsCallback;

void Callback::attachCallback(rsScene::Obstacle *scene, rsSim::Obstacle *sim) {
	scene->setUpdateCallback(new Obstacle(sim));
}

