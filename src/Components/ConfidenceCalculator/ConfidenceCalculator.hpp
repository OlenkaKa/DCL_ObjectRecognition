/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef CONFIDENCECALCULATOR_HPP_
#define CONFIDENCECALCULATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/Objects3D/Object3D.hpp"

namespace Processors {
namespace ConfidenceCalculator {

/*!
 * \class ConfidenceCalculator
 * \brief ConfidenceCalculator processor class.
 *
 * TODO description
 */
class ConfidenceCalculator : public Base::Component {
public:
    /*!
     * Constructor.
     */
    ConfidenceCalculator(const std::string &name = "ConfidenceCalculator");

    /*!
     * Destructor
     */
    virtual ~ConfidenceCalculator();

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
    Base::DataStreamIn <std::vector<int> > in_inliers;
    Base::DataStreamIn <Types::Objects3D::Object3D> in_object_points;

    // Output data streams
    Base::DataStreamOut<std::vector<double> > out_confidences;

    // Handlers

    // Properties
    Base::Property<int> min_inliers;


    // Handlers
    void onNewObjectData();

};

} //: namespace ConfidenceCalculator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ConfidenceCalculator", Processors::ConfidenceCalculator::ConfidenceCalculator)

#endif /* CONFIDENCECALCULATOR_HPP_ */
