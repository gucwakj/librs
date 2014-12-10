#ifndef RSCALLBACK_LINKBOTCALLBACK_HPP_
#define RSCALLBACK_LINKBOTCALLBACK_HPP_

#include <osg/Geode>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osgText/Text>

#include <rsSim/linkbot.hpp>

namespace rsCallback {

	class linkbotCallback : public osg::NodeCallback {
		// public functions
		public:
			linkbotCallback(rsSim::CLinkbotT*, dBodyID*, rsSim::ConnectorList&, bool);
			virtual ~linkbotCallback(void) {};

			virtual void operator()(osg::Node*, osg::NodeVisitor*);

		// private data
		private:
			bool _units;
			int _count;
			rsSim::CLinkbotT *_robot;
			rsSim::ConnectorList _conn;
			dBodyID *_bodies;
	};

} // namespace rsCallback

#endif // RSCALLBACK_LINKBOTCALLBACK_HPP_

