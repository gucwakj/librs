#include <rsCallback/callback.hpp>
#include <rsCallback/groundCallback.hpp>

using namespace rsCallback;

void Callback::attachCallback(rsScene::Ground *scene, rsSim::Ground2 *sim) {
	scene->setUpdateCallback(new groundCallback(sim));
}

