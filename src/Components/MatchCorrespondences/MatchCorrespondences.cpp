/*!
 * \file MatchCorrespondences.cpp
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include <boost/bind.hpp>
#include <boost/thread/locks.hpp>

#include "MatchCorrespondences.hpp"
#include "Common/Logger.hpp"

using namespace boost;
using namespace cv;
using namespace std;

namespace Processors {
namespace MatchCorrespondences {

const unsigned int MatchCorrespondences::DESCRIPTOR_SIZE = 128;

MatchCorrespondences::MatchCorrespondences(const std::string &name) :
        Base::Component(name),
        ratio_("ratio", boost::bind(&MatchCorrespondences::onRatioChanged, this, _1, _2), 0.8),
        matcher_type_("matcher", boost::bind(&MatchCorrespondences::onMatcherTypeChanged, this, _1, _2), "flann") {
    registerProperty(ratio_);
    registerProperty(matcher_type_);
}

MatchCorrespondences::~MatchCorrespondences() {
}

void MatchCorrespondences::prepareInterface() {
    // Register data streams, events and event handlers
    registerStream("in_scene_features", &in_scene_features_);
    registerStream("in_scene_descriptors", &in_scene_descriptors_);
    registerStream("in_model_labels", &in_model_labels_);
    registerStream("in_model_clouds_xyzrgb", &in_model_clouds_xyzrgb_);
    registerStream("in_model_clouds_xyzsift", &in_model_clouds_xyzsift_);
    registerStream("in_model_vertices_xyz", &in_model_vertices_xyz_);
    registerStream("in_model_triangles", &in_model_triangles_);
    registerStream("in_model_bounding_boxes", &in_model_bounding_boxes_);
    registerStream("out_object_name", &out_object_name_);
    registerStream("out_object", &out_object_points_);

    // Register handlers
    registerHandler("onNewScene", bind(&MatchCorrespondences::onNewScene, this));
    addDependency("onNewScene", &in_scene_features_);
    addDependency("onNewScene", &in_scene_descriptors_);

    registerHandler("onNewModel", bind(&MatchCorrespondences::onNewModel, this));
    addDependency("onNewModel", &in_model_labels_);
    addDependency("onNewModel", &in_model_bounding_boxes_);
    addDependency("onNewModel", &in_model_triangles_);
    addDependency("onNewModel", &in_model_vertices_xyz_);
    addDependency("onNewModel", &in_model_clouds_xyzsift_);
    addDependency("onNewModel", &in_model_clouds_xyzrgb_);
}

bool MatchCorrespondences::onInit() {
    CLOG(LTRACE) << "MatchCorrespondences::onInit";
    initMatcher();
    return true;
}

bool MatchCorrespondences::onFinish() {
    CLOG(LTRACE) << "MatchCorrespondences::onFinish";
    return true;
}

bool MatchCorrespondences::onStop() {
    CLOG(LTRACE) << "MatchCorrespondences::onStop";
    return true;
}

bool MatchCorrespondences::onStart() {
    CLOG(LTRACE) << "MatchCorrespondences::onStart";
    return true;
}

void MatchCorrespondences::onNewScene() {
    CLOG(LTRACE) << "MatchCorrespondences::onNewScene";
    shared_lock<shared_mutex> read_lock(model_lock_);

    vector<KeyPoint> scene_keypoints = in_scene_features_.read().features;
    Mat scene_descriptors = in_scene_descriptors_.read();

    vector<vector<DMatch> > matches1, matches2;
    matcher_->knnMatch(scene_descriptors, model_descriptors_, matches1, 7);
    matcher_->knnMatch(model_descriptors_, scene_descriptors, matches2, 7);

    ratioTest(matches1);
    ratioTest(matches2);

    vector<DMatch> good_matches;
    symmetryTest(matches1, matches2, good_matches);


    vector<Point3f> object_points3d;
    vector<Point2f> object_points2d;

    for (size_t i = 0, matches_size = good_matches.size(); i < matches_size; ++i) {
        object_points3d.push_back(model_points_[good_matches[i].trainIdx]);
        object_points2d.push_back(scene_keypoints[good_matches[i].queryIdx].pt);
    }

//    CLOG(LNOTICE) << "3d: " << object_points3d.size();
//    CLOG(LNOTICE) << "2d: " << object_points2d.size();
//    CLOG(LNOTICE) << "-------------------------------";
//
//    for(int i = 0; i < object_points3d.size(); ++i) {
//        Point3f p = object_points3d[i];
//        Point2f ip = object_points2d[i];
//        CLOG(LNOTICE) << p.x << "\t" << p.y << "\t" << p.z << "\t | \t" << ip.x << "\t" << ip.y;
//    }

    // write result
    Types::Objects3D::Object3D object_3d;
    object_3d.setModelPoints(object_points3d);
    object_3d.setImagePoints(object_points2d);
    out_object_name_.write(model_name_);
    out_object_points_.write(object_3d);
}

void MatchCorrespondences::onNewModel() {
    CLOG(LTRACE) << "MatchCorrespondences::onNewModel";
    unique_lock<shared_mutex> write_lock(model_lock_);

    model_labels_ = in_model_labels_.read();
    model_clouds_xyzrgb_ = in_model_clouds_xyzrgb_.read();
    model_clouds_xyzsift_ = in_model_clouds_xyzsift_.read();
    model_vertices_xyz_ = in_model_vertices_xyz_.read();
    model_triangles_ = in_model_triangles_.read();
    model_bounding_boxes_ = in_model_bounding_boxes_.read();

    model_name_ = model_labels_[0];
    pcl::PointCloud<PointXYZSIFT>::Ptr model_cloud = model_clouds_xyzsift_[0];
    model_points_.clear();
    model_descriptors_.release();
    Mat_<float> temp_descriptors((int) model_cloud->size(), DESCRIPTOR_SIZE, CV_32F);

    int row_idx = 0;
    for (pcl::PointCloud<PointXYZSIFT>::iterator it = model_cloud->begin(), end_it = model_cloud->end();
         it != end_it; ++it, ++row_idx) {
        model_points_.push_back(Point3f(it->x, it->y, it->z));
        vector<float> row(DESCRIPTOR_SIZE);
        row.assign(it->descriptor, it->descriptor + DESCRIPTOR_SIZE);
        temp_descriptors.row(row_idx) = Mat(row).t();
    }
    model_descriptors_ = temp_descriptors.clone();
}

void MatchCorrespondences::onRatioChanged(float old_value, float new_value) {
    if (new_value > 1.0 || new_value < 0.0) {
        CLOG(LWARNING) << "Cannot set radio to " << new_value << ". Old value will be used: ratio = " << old_value << ".";
        ratio_ = old_value;
    } else {
        ratio_ = new_value;
        CLOG(LTRACE) << "MatchCorrespondences::onRatioChanged - from " << old_value << " to " << ratio_;
    }
}

void MatchCorrespondences::onMatcherTypeChanged(const string& old_value, const string& new_value) {
    CLOG(LERROR) << "MatchCorrespondences::onMatcherTypeChanged";
    if (old_value.compare(new_value) == 0) {
        CLOG(LWARNING) << "New matcher type is the same as the previous one.";
    } else {
        updateMatcher(new_value);
    }
}

void MatchCorrespondences::initMatcher() {
    if (string("flann").compare(matcher_type_) == 0) {
        matcher_ = makePtr<FlannBasedMatcher>();
    } else {
        CLOG(LWARNING) << "Invalid matcher type: " << matcher_type_ << ". Use default: flann";
        matcher_type_ = "flann";
        matcher_ = makePtr<FlannBasedMatcher>();
    }
}


void MatchCorrespondences::updateMatcher(const std::string &new_matcher_type_) {
    if (string("flann").compare(new_matcher_type_) == 0) {
        matcher_ = makePtr<FlannBasedMatcher>();
    } else {
        CLOG(LWARNING) << "Invalid matcher type: " << new_matcher_type_;
        return;
    }
    matcher_type_ = new_matcher_type_;
}

void MatchCorrespondences::ratioTest(vector<vector<DMatch> >& matches) {
    for (vector<vector<DMatch> >::iterator feature_it = matches.begin(); feature_it != matches.end(); ++feature_it) {
        double nearest_distance = feature_it->front().distance;
        for (vector<DMatch>::iterator match_it = feature_it->begin() + 1; match_it != feature_it->end(); ) {
            if (nearest_distance / match_it->distance < ratio_) {
                ++match_it;
            } else {
                match_it = feature_it->erase(match_it);
            }
        }
    }
}

void MatchCorrespondences::symmetryTest(const vector<vector<DMatch> >& matches1,
                                        const vector<vector<DMatch> >& matches2,
                                        vector<DMatch> &symmetryMatches) {
    DMatch dmatch1, dmatch2;
    for (int i = 0; i < matches1.size(); ++i) {
        for (int j = 0; j < matches1[i].size(); ++j) {
            dmatch1 = matches1[i][j];
            dmatch2 = matches2[i][j];
            if (dmatch1.queryIdx == dmatch2.trainIdx &&
                dmatch1.trainIdx == dmatch2.queryIdx) {
                // add symmetrical match
                symmetryMatches.push_back(dmatch1);
            }
        }
    }
}

} //: namespace MatchCorrespondences
} //: namespace Processors
