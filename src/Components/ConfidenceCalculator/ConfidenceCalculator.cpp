/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ConfidenceCalculator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ConfidenceCalculator {

ConfidenceCalculator::ConfidenceCalculator(const std::string &name) :
        Base::Component(name),
        min_inliers_num("min_inliers_num", 10, "min_inliers") {
    registerProperty(min_inliers_num);
}

ConfidenceCalculator::~ConfidenceCalculator() {
}

void ConfidenceCalculator::prepareInterface() {
    // Register data streams, events and event handlers HERE!
    registerStream("in_inliers", &in_inliers);
    registerStream("in_object_points", &in_object_points);
    registerStream("out_confidence", &out_confidence);
    // Register handlers
    registerHandler("onNewObjectData", boost::bind(&ConfidenceCalculator::onNewObjectData, this));
    addDependency("onNewObjectData", &in_inliers);
    addDependency("onNewObjectData", &in_object_points);
}

bool ConfidenceCalculator::onInit() {
    return true;
}

bool ConfidenceCalculator::onFinish() {
    return true;
}

bool ConfidenceCalculator::onStop() {
    return true;
}

bool ConfidenceCalculator::onStart() {
    return true;
}

void ConfidenceCalculator::onNewObjectData() {
}

} //: namespace ConfidenceCalculator
} //: namespace Processors
