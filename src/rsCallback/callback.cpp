#include <rsCallback/Callback>
#include <rsCallback/Obstacle>

using namespace rsCallback;

/**********************************************************
	public functions
 **********************************************************/
void Callback::attachCallback(rsScene::Obstacle *scene, rsSim::Obstacle *sim) {
	scene->setUpdateCallback(new Obstacle(sim));
}

