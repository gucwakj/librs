#include "skyTransform.hpp"

using namespace rsScene;

bool skyTransform::computeLocalToWorldMatrix(osg::Matrix &matrix, osg::NodeVisitor *nv) const {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor *>(nv);
	if (cv) {
		osg::Vec3 eyePointLocal = cv->getEyeLocal();
		matrix.preMultTranslate(eyePointLocal);
	}
	return true;
}

bool skyTransform::computeWorldToLocalMatrix(osg::Matrix &matrix, osg::NodeVisitor *nv) const {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor *>(nv);
	if (cv) {
		osg::Vec3 eyePointLocal = cv->getEyeLocal();
		matrix.postMultTranslate(-eyePointLocal);
	}
	return true;
}

