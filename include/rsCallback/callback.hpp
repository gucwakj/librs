#ifndef RSCALLBACK_CALLBACK_HPP_
#define RSCALLBACK_CALLBACK_HPP_

#include <osg/Node>
#include <osg/NodeVisitor>

#include <rsScene/scene.hpp>
#include <rsSim/sim.hpp>

namespace rsCallback {

	class Callback {
		// public functions
		public:
			Callback(void) {};
			virtual ~Callback(void) {};

			void attachCallback(rsScene::Ground*, rsSim::Ground*);
	};

} // namespace rsCallback

#endif // RSCALLBACK_CALLBACK_HPP_

