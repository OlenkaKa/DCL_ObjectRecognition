/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef MATCHCORRESPONDENCES_HPP_
#define MATCHCORRESPONDENCES_HPP_

#include <vector>
#include <opencv2/opencv.hpp>

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/Features.hpp"
#include "Types/Objects3D/Object3D.hpp"
#include "Types/PointXYZSIFT.hpp"
#include "Types/SIFTObjectModelFactory.hpp"

namespace Processors {
namespace MatchCorrespondences {

/*!
 * \class MatchCorrespondences
 * \brief MatchCorrespondences processor class.
 *
 * Description TODO
 */
class MatchCorrespondences : public Base::Component {
public:
    /*!
     * Constructor.
     */
    MatchCorrespondences(const std::string &name = "MatchCorrespondences");

    /*!
     * Destructor
     */
    virtual ~MatchCorrespondences();

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
    Base::DataStreamIn <Types::Features> in_scene_features;
    Base::DataStreamIn <cv::Mat> in_scene_descriptors;
    Base::DataStreamIn <std::vector<std::string> > in_model_names;
    Base::DataStreamIn <std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > in_model_clouds_xyzrgb;
    Base::DataStreamIn <std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> > in_model_clouds_xyzsift;
    Base::DataStreamIn <std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > in_model_vertices_xyz;
    Base::DataStreamIn <std::vector<std::vector<pcl::Vertices> > > in_model_triangles;
    Base::DataStreamIn <std::vector<std::vector<pcl::Vertices> > > in_model_bounding_boxes;

    // Output data streams
    Base::DataStreamOut <Types::Objects3D::Object3D> out_object;

    // Handlers

    // Properties
    //Base::Property<std::string> test_prop;


    // Handlers
    void onNewScene();

    void onNewModel();

};

} //: namespace MatchCorrespondences
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("MatchCorrespondences", Processors::MatchCorrespondences::MatchCorrespondences)

#endif /* MATCHCORRESPONDENCES_HPP_ */
