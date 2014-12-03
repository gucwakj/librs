#ifndef RSSCENE_TEXTURECALLBACK_HPP_
#define RSSCENE_TEXTURECALLBACK_HPP_

#include <osg/Node>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/TexMat>
#include <osgUtil/CullVisitor>

namespace rsScene {

class textureCallback : public osg::NodeCallback {
	public:
		textureCallback(osg::TexMat&);
		virtual void operator()(osg::Node*, osg::NodeVisitor*);
	private:
		osg::TexMat& _texMat;
};

} // namespace rsScene

#endif // RSSCENE_TEXTURECALLBACK_HPP_

