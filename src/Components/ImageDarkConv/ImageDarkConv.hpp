/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef IMAGEDARKCONV_HPP_
#define IMAGEDARKCONV_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace ImageDarkConv {

/*!
 * \class ImageDarkConv
 * \brief ImageDarkConv processor class.
 *
 * description TODO
 */
class ImageDarkConv: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ImageDarkConv(const std::string & name = "ImageDarkConv");

	/*!
	 * Destructor
	 */
	virtual ~ImageDarkConv();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties

	
	// Handlers
	void onNewImage();

};

} //: namespace ImageDarkConv
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ImageDarkConv", Processors::ImageDarkConv::ImageDarkConv)

#endif /* IMAGEDARKCONV_HPP_ */
