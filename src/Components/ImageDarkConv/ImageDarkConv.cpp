/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ImageDarkConv.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
//#include <opencv2/highgui.hpp>

using namespace cv;

namespace Processors {
namespace ImageDarkConv {

ImageDarkConv::ImageDarkConv(const std::string & name) :
		Base::Component(name)  {
}

ImageDarkConv::~ImageDarkConv() {
}

void ImageDarkConv::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&ImageDarkConv::onNewImage, this));
	addDependency("onNewImage", &in_img);

}

bool ImageDarkConv::onInit() {
	return true;
}

bool ImageDarkConv::onFinish() {
	return true;
}

bool ImageDarkConv::onStop() {
	return true;
}

bool ImageDarkConv::onStart() {
	return true;
}

void ImageDarkConv::onNewImage() {
	Mat image = in_img.read().clone();
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            uchar gray_value = image.at<Vec3b>(y, x)[0] * 0.114 + image.at<Vec3b>(y, x)[1] * 0.587 +
                               image.at<Vec3b>(y, x)[2] * 0.299;
            for (int c = 0; c < 3; c++) {
                image.at<Vec3b>(y, x)[c] =
                        saturate_cast<uchar>(gray_value - 50);
            }
        }
    }
	out_img.write(image);
}



} //: namespace ImageDarkConv
} //: namespace Processors
