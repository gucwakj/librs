#include <rsCallback/callback.hpp>
#include <rsCallback/groundCallback.hpp>

using namespace rsCallback;

void Callback::attachCallback(rsScene::Ground *scene, rsSim::Ground *sim) {
	scene->setUpdateCallback(new groundCallback(sim));
}

