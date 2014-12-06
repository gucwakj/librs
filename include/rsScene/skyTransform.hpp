#ifndef RSSCENE_SKYTRANSFORM_HPP_
#define RSSCENE_SKYTRANSFORM_HPP_

#include <osg/Matrix>
#include <osg/NodeVisitor>
#include <osg/Transform>
#include <osgUtil/CullVisitor>

namespace rsScene {

	class skyTransform : public osg::Transform {
		// public functions
		public:
			virtual bool computeLocalToWorldMatrix(osg::Matrix&, osg::NodeVisitor*) const;
			virtual bool computeWorldToLocalMatrix(osg::Matrix&, osg::NodeVisitor*) const;
	};

} // namespace rsScene

#endif // RSSCENE_SKYTRANSFORM_HPP_

