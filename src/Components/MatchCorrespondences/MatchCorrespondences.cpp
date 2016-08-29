/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include <boost/bind.hpp>

#include "MatchCorrespondences.hpp"
#include "Common/Logger.hpp"

using namespace cv;
using namespace std;

namespace Processors {
namespace MatchCorrespondences {

MatchCorrespondences::MatchCorrespondences(const std::string &name) :
        Base::Component(name) {//,
    //test_prop("test_prop", NULL, "test") {
    //registerProperty(test_prop);

}

MatchCorrespondences::~MatchCorrespondences() {
}

void MatchCorrespondences::prepareInterface() {
    // Register data streams, events and event handlers HERE!
    registerStream("in_scene_features", &in_scene_features);
    registerStream("in_scene_descriptors", &in_scene_descriptors);
    registerStream("in_model_names", &in_model_names);
    registerStream("in_model_clouds_xyzrgb", &in_model_clouds_xyzrgb);
    registerStream("in_model_clouds_xyzsift", &in_model_clouds_xyzsift);
    registerStream("in_model_vertices_xyz", &in_model_vertices_xyz);
    registerStream("in_model_triangles", &in_model_triangles);
    registerStream("in_model_bounding_boxes", &in_model_bounding_boxes);
    registerStream("out_object", &out_object);
    // Register handlers
    registerHandler("onNewScene", boost::bind(&MatchCorrespondences::onNewScene, this));
    addDependency("onNewScene", &in_scene_features);
    addDependency("onNewScene", &in_scene_descriptors);
    registerHandler("onNewModel", boost::bind(&MatchCorrespondences::onNewModel, this));
    addDependency("onNewModel", &in_model_names);
	addDependency("onNewModel", &in_model_bounding_boxes);
	addDependency("onNewModel", &in_model_triangles);
	addDependency("onNewModel", &in_model_vertices_xyz);
	addDependency("onNewModel", &in_model_clouds_xyzsift);
	addDependency("onNewModel", &in_model_clouds_xyzrgb);
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
    // TODO
}

void MatchCorrespondences::onNewModel() {
    CLOG(LTRACE) << "MatchCorrespondences::onNewModel";

    // TODO
    vector <string> names = in_model_names.read();
    for (int i = 0; i < names.size(); ++i) {
        CLOG(LERROR) << names[i];
    }

}


} //: namespace MatchCorrespondences
} //: namespace Processors
