/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef MATCHESVISUALIZATION_HPP_
#define MATCHESVISUALIZATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/Objects3D/Object3D.hpp"
#include "Types/Features.hpp"
#include "Types/FeaturesWithColor.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace MatchesVisualization {

/*!
 * \class MatchesVisualization
 * \brief MatchesVisualization processor class.
 *
 * description TODO
 */
class MatchesVisualization: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	MatchesVisualization(const std::string & name = "MatchesVisualization");

	/*!
	 * Destructor
	 */
	virtual ~MatchesVisualization();

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
	Base::DataStreamIn<Types::Features> in_scene_features_;
	Base::DataStreamIn<Types::Objects3D::Object3D> in_matches_;
	Base::DataStreamIn<std::vector<int>> in_inliers_;

	// Output data streams
	Base::DataStreamOut<Types::FeaturesWithColor> out_features_;

	// Handlers

	// Properties

	
	// Handlers
	void onNewData();

};

} //: namespace MatchesVisualization
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("MatchesVisualization", Processors::MatchesVisualization::MatchesVisualization)

#endif /* MATCHESVISUALIZATION_HPP_ */
