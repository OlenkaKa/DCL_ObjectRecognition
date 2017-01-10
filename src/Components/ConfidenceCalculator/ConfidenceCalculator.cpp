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
    registerStream("out_confidences", &out_confidences);
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

    vector<double> confidences(inliers.size());

    double model_points_num = (double) object_points.getModelPoints().size() / (double) inliers.size();

    if (model_points_num == 0) {
        fill(confidences.begin(), confidences.end(), 0.0);
    } else {
        for (int i = 0; i < inliers.size(); ++i) {
            double confidence = (double) inliers[i] / (double) model_points_num;
            if (inliers[i] < min_inliers) {
                confidence /= 2;
            }
            confidences[i] = confidence;

            CLOG(LERROR) << "----------------------------";
            CLOG(LERROR) << "hypothese: " << i;
            CLOG(LERROR) << "model_points_num: " << model_points_num;
            CLOG(LERROR) << "inliers_num: " << inliers[i];
            CLOG(LERROR) << "CONFIDENCE: " << confidence;
            CLOG(LERROR) << "----------------------------";
        }
    }
    out_confidences.write(confidences);
}

} //: namespace ConfidenceCalculator
} //: namespace Processors
