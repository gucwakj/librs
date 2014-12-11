#ifndef RSCALLBACK_CALLBACK_HPP_
#define RSCALLBACK_CALLBACK_HPP_

#include <osg/Node>
#include <osg/NodeVisitor>
#include <osgText/Text>

#include <rsScene/scene.hpp>
#include <rsSim/sim.hpp>
#include <rsSim/linkbot.hpp>

namespace rsCallback {

	class Callback {
		// public functions
		public:
			Callback(void);
			virtual ~Callback(void) {};

			void attachCallback(rsScene::Ground*, rsSim::Ground*);
			void attachCallback(rsScene::Robot*, rsSim::Robot*, rsSim::BodyList&, rsSim::ConnectorList&);

			void setUnits(bool);

		// private data
		private:
			bool _units;
	};

} // namespace rsCallback

#endif // RSCALLBACK_CALLBACK_HPP_
