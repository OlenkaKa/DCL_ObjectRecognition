/*!
 * \file MatchCorrespondences.hpp
 * \author Aleksandra Karbarczyk
 */

#ifndef MATCHCORRESPONDENCES_HPP_
#define MATCHCORRESPONDENCES_HPP_

#include <vector>
#include <opencv2/opencv.hpp>

#include <boost/thread/shared_mutex.hpp>

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
 * Component responsible for finding matches between object model and image based on SIFT features.
 * For now the component use only one model - the first from the list received from input stream.
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

    /// Consts
    static const unsigned int DESCRIPTOR_SIZE;  // size of SIFT descriptor

    /// Model typedefs
    typedef std::vector<std::string> ModelLabels;
    typedef std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ModelCloudsXYZRGB;
    typedef std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> ModelCloudsXYZSIFT;
    typedef std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ModelVerticesXYZ;
    typedef std::vector<std::vector<pcl::Vertices> > ModelTriangles;
    typedef std::vector<std::vector<pcl::Vertices> > ModelBoundingBoxes;

    /// Model data
    ModelLabels model_labels_;
    ModelCloudsXYZRGB model_clouds_xyzrgb_;
    ModelCloudsXYZSIFT model_clouds_xyzsift_;
    ModelVerticesXYZ model_vertices_xyz_;
    ModelTriangles model_triangles_;
    ModelBoundingBoxes model_bounding_boxes_;

    std::string model_name_;
    std::vector<cv::Point3f> model_points_;
    cv::Mat model_descriptors_;
//    std::vector<cv::Point3f> model_vertices_;
//    std::vector<>

    /// Matcher
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    /// Synchronization
    boost::shared_mutex model_lock_;

    /// Input data streams
    /// Scene
    Base::DataStreamIn<Types::Features> in_scene_features_;
    Base::DataStreamIn<cv::Mat> in_scene_descriptors_;
    /// Model
    Base::DataStreamIn<ModelLabels> in_model_labels_;
    Base::DataStreamIn<ModelCloudsXYZRGB> in_model_clouds_xyzrgb_;
    Base::DataStreamIn<ModelCloudsXYZSIFT> in_model_clouds_xyzsift_;
    Base::DataStreamIn<ModelVerticesXYZ> in_model_vertices_xyz_;
    Base::DataStreamIn<ModelTriangles> in_model_triangles_;
    Base::DataStreamIn<ModelBoundingBoxes> in_model_bounding_boxes_;

    /// Output data streams
    Base::DataStreamOut<std::string> out_object_name_;
    Base::DataStreamOut<Types::Objects3D::Object3D> out_object_points_;

    /// Properties
    Base::Property<float> ratio_;
    Base::Property<std::string> matcher_type_;

    /// Handlers
    void onNewScene();
    void onNewModel();
    void onRatioChanged(float old_value, float new_value);
    void onMatcherTypeChanged(const std::string& old_value, const std::string& new_value);

    /// Others
    void initMatcher();
    void updateMatcher(const std::string &new_matcher_type);
    void ratioTest(std::vector<std::vector<cv::DMatch> >& matches);

    void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
                      const std::vector<std::vector<cv::DMatch> >& matches2,
                      std::vector<cv::DMatch>& symmetryMatches);
};

} //: namespace MatchCorrespondences
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("MatchCorrespondences", Processors::MatchCorrespondences::MatchCorrespondences)

#endif /* MATCHCORRESPONDENCES_HPP_ */
