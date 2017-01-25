#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include <rsScene/OpenCVOperation>

using namespace rsScene;

void OpenCVOperation::operator()(const osg::Image &image, const unsigned int /*context_id*/) {
	// get rid of const qualifier
	osg::ref_ptr<osg::Image> img = new osg::Image(image);

	// get pixel format of image
	if (img->getPixelFormat() != GL_RGB) return;

	// check image size
	int width = img->s();
	int height = img->t();
	int depth = img->r();
	if (width > 0 && height > 0 && depth > 0) {
		// convert image from osg -> opencv
		IplImage *ipl = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		cvSetData(ipl, img->data(), 3*width);
		cv::Mat mat = cv::cvarrToMat(ipl);

		// flip image
		cv::Mat mat2;
		cv::flip(mat, mat2, 0);

		// show image for testing
		//imshow("Mat", mat2);
		//cvWaitKey(0);

		// save to disk for testing
		//cv::imwrite("output.png", mat2);
	}
}

