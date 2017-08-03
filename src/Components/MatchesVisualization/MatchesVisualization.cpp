/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <algorithm>
#include <memory>
#include <string>

#include "MatchesVisualization.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace MatchesVisualization {

MatchesVisualization::MatchesVisualization(const std::string & name) :
		Base::Component(name)  {
}

MatchesVisualization::~MatchesVisualization() {
}

void MatchesVisualization::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_scene_features", &in_scene_features_);
	registerStream("in_matches", &in_matches_);
    registerStream("in_inliers", &in_inliers_);
	registerStream("out_features", &out_features_);
	// Register handlers
	registerHandler("onNewData", boost::bind(&MatchesVisualization::onNewData, this));
	addDependency("onNewData", &in_scene_features_);
	addDependency("onNewData", &in_matches_);
    addDependency("onNewData", &in_inliers_);
}

bool MatchesVisualization::onInit() {
	return true;
}

bool MatchesVisualization::onFinish() {
	return true;
}

bool MatchesVisualization::onStop() {
	return true;
}

bool MatchesVisualization::onStart() {
	return true;
}

void MatchesVisualization::onNewData() {
	Types::Features scene_features = in_scene_features_.read();
    CLOG(LWARNING) << "in_scene_features_: " << scene_features.features.size();

    const std::vector <cv::Point2f> matches_object3d = in_matches_.read().getImagePoints();
    CLOG(LWARNING) << "in_matches_: " << matches_object3d.size();

    vector<int> pose_inliers = in_inliers_.read();
    CLOG(LWARNING) << "in_inliers_: " << pose_inliers.size();

    std::vector<cv::KeyPoint> incorrect;
    std::vector<cv::KeyPoint> matches;
    std::vector<cv::KeyPoint> inliers;

    for (const cv::KeyPoint & keypoint : scene_features.features) {
        for (int idx = 0; idx < matches_object3d.size(); ++idx) {
            if (matches_object3d[idx] == keypoint.pt) {
                if (std::find(pose_inliers.begin(), pose_inliers.end(), idx) != pose_inliers.end()) {
                    inliers.push_back(keypoint);
                } else {
                    matches.push_back(keypoint);
                }
                break;
            }
        }
        if ((matches.size() == 0 || matches.back().pt != keypoint.pt) &&
            (inliers.size() == 0 || inliers.back().pt != keypoint.pt)) {

            incorrect.push_back(keypoint);
        }
    }

    std::vector<Types::FeaturesWithColor::ColorPair> features(3);
    features.push_back(Types::FeaturesWithColor::ColorPair(cv::Scalar(255, 255, 255), incorrect));
    features.push_back(Types::FeaturesWithColor::ColorPair(cv::Scalar(0, 255, 0), matches));
    features.push_back(Types::FeaturesWithColor::ColorPair(cv::Scalar(0, 0, 255), inliers));

    out_features_.write(Types::FeaturesWithColor(features));
}



} //: namespace MatchesVisualization
} //: namespace Processors
