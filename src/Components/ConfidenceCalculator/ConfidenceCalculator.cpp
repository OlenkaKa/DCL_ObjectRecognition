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

using namespace std;

namespace Processors {
namespace ConfidenceCalculator {

ConfidenceCalculator::ConfidenceCalculator(const std::string &name) :
        Base::Component(name),
        min_inliers("min_inliers", 10) {
    registerProperty(min_inliers);
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
    vector<int> inliers = in_inliers.read();
    Types::Objects3D::Object3D object_points = in_object_points.read();

    double confidence;

    size_t model_points_num = object_points.getModelPoints().size();
    size_t inliers_num = inliers.size();

    if (model_points_num == 0) {
        confidence = 0.0;
    } else {
        confidence = (double) inliers_num / (double) model_points_num;
        if (inliers_num < min_inliers) {
            confidence /= 2;
        }
    }
//    CLOG(LERROR) << "----------------------------";
//    CLOG(LERROR) << "model_points_num: " << model_points_num;
//    CLOG(LERROR) << "inliers_num: " << inliers_num;
//    CLOG(LERROR) << "CONFIDENCE: " << confidence;
//    CLOG(LERROR) << "----------------------------";
    out_confidence.write(confidence);
}

} //: namespace ConfidenceCalculator
} //: namespace Processors
