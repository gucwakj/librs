#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include <rsScene/OpenCVOperation>
#include <rsScene/Scene>

using namespace rsScene;

OpenCVOperation::OpenCVOperation(Scene *scene) {
	_scene = scene;
}

void OpenCVOperation::operator()(const osg::Image &image, const unsigned int /*context_id*/) {
	// get rid of const qualifier
	osg::ref_ptr<osg::Image> img = new osg::Image(image);

	// get pixel format of image
	if (img->getPixelFormat() != GL_RGB) return;

	// check image size
	int width = img->s();
	int height = img->t();
	if (width > 0 && height > 0 && img->r() > 0) {
		// convert image from osg -> opencv
		IplImage *ipl = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		cvSetData(ipl, img->data(), 3*width);
		cv::Mat mat = cv::cvarrToMat(ipl);

		// flip image
		cv::flip(mat, mat, 0);

		// convert the image from BGR to HSV
		cvtColor(mat, mat, cv::COLOR_BGR2HSV);

		// threshold the HSV image
		inRange(mat, cv::Scalar(120, 0, 0), cv::Scalar(179, 255, 255), mat);

		// use probabilistic hough line detector
		cv::vector<cv::Vec4i> lines;
		cv::HoughLinesP(mat, lines, 1, CV_PI/180, 200, 200, 20);
		double pt[6] = {0};
		for (unsigned int i = 0; i < lines.size(); i++) {
			for (unsigned int j = 0; j < 4; j++) {
				pt[j] += lines[i][j];
			}
			pt[4] += (lines[i][2] - lines[i][0])/2 + lines[i][0];
			pt[5] += (lines[i][3] - lines[i][1])/2 + lines[i][1];
		}
		if (lines.size()) {
			for (unsigned int i = 0; i < 6; i++) {
				pt[i] /= lines.size();
			}
		}

		// calculate angle
		double theta = CV_PI/2;
		if (lines.size()) {
			cv::Point c(mat.cols/2, mat.rows/2);
			cv::Point pc(pt[4], pt[5]);
			theta = asin((c.y - pc.y) / sqrt(pow(pc.x - c.x, 2) + pow(pc.y - c.y, 2)));
			theta = CV_PI/2 - theta;
			if (c.x > pc.x) { theta *= -1; }
		}

		// send back to scene 
		_scene->setTheta(theta);

		// show image for testing
		bool output = false;
		if (output) {
			// create output drawing
			cv::Mat drawing = cv::Mat::zeros(mat.size(), CV_8UC3);
			// draw line and center
			cv::line(drawing, cv::Point(pt[0], pt[1]), cv::Point(pt[2], pt[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
			cv::circle(drawing, cv::Point(pt[4], pt[5]), 5.0, cv::Scalar(0, 255, 0), -1, 8);
			// show output
			imshow("drawing", drawing);
			cvWaitKey(40);
		}

		// save to disk for testing
		//cv::imwrite("output.png", mat);
	}
}

