#ifndef RSCALLBACK_LINKBOTCALLBACK_HPP_
#define RSCALLBACK_LINKBOTCALLBACK_HPP_

#include <osg/Node>
#include <osg/NodeVisitor>

#include <rsSim/linkbot.hpp>

namespace rsCallback {

	class linkbotCallback : public osg::NodeCallback {
		// public functions
		public:
			linkbotCallback(rsSim::LinkbotT*, rsSim::BodyList&, rsSim::ConnectorList&, bool);
			virtual ~linkbotCallback(void) {};

			virtual void operator()(osg::Node*, osg::NodeVisitor*);

		// private data
		private:
			bool _units;
			int _count;
			rsSim::LinkbotT *_robot;
			rsSim::BodyList _bodies;
			rsSim::ConnectorList _conn;
	};

} // namespace rsCallback

#endif // RSCALLBACK_LINKBOTCALLBACK_HPP_

