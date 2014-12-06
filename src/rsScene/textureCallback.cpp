#include <rsScene/textureCallback.hpp>

using namespace rsScene;

void textureCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor *>(nv);
	if (cv) {
		const osg::Matrix &MV = *(cv->getModelViewMatrix());
		const osg::Matrix R = osg::Matrix::rotate(osg::DegreesToRadians(112.0), 0.0, 0.0, 1.0) *
								osg::Matrix::rotate(osg::DegreesToRadians(90.0), 1.0, 0.0, 0.0);

		osg::Quat q = MV.getRotate();
		const osg::Matrix C = osg::Matrix::rotate(q.inverse());

		_texMat.setMatrix(C*R);
	}
	traverse(node, nv);
}

