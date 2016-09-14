/*!
 * \file
 * \brief
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
        Base::Component(name) {//,
    //test_prop("test_prop", NULL, "test") {
    //registerProperty(test_prop);

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
    registerStream("out_object", &out_object_);

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

    FlannBasedMatcher matcher;
    vector<DMatch> matches;
    matcher.match(scene_descriptors, model_descriptors_, matches);

    vector<DMatch> good_matches;
    good_matches = matches; // TODO filter good matches

    vector<Point3f> object_points3d;
    vector<Point2f> object_points2d;

    for (int i = 0; i < good_matches.size(); ++i) {
        object_points3d.push_back(model_points_[good_matches[i].trainIdx]);
        object_points2d.push_back(scene_keypoints[good_matches[i].queryIdx].pt);
    }

    // write result
    Types::Objects3D::Object3D object_3d;
    object_3d.setModelPoints(object_points3d);
    object_3d.setImagePoints(object_points2d);
    out_object_.write(object_3d);
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

    // TODO get all models
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
    model_descriptors_ = temp_descriptors;
}


} //: namespace MatchCorrespondences
} //: namespace Processors
