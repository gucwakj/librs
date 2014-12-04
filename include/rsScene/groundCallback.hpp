#ifndef RSSCENE_GROUNDCALLBACK_HPP_
#define RSSCENE_GROUNDCALLBACK_HPP_

#include <osg/Node>
#include <osg/NodeVisitor>

namespace rsScene {

struct Ground;

class groundNodeCallback : public osg::NodeCallback {
	public:
		groundCallback(Ground*);
		virtual void operator()(osg::Node*, osg::NodeVisitor*);
	private:
		Ground *_ground;
};

} // namespace rsScene

#endif // RSSCENE_GROUNDCALLBACK_HPP_

