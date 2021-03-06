/******************************************************************************
 * Copyright (c) 2015 Thomas Faeulhammer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

/**
*
*      @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
*      @date August, 2015
*      @brief multiview object instance recognizer
*      Reference(s): Faeulhammer et al, ICRA 2015
*                    Faeulhammer et al, MVA 2015
*/

#ifndef V4R_MULTIVIEW_OBJECT_RECOGNIZER_H__
#define V4R_MULTIVIEW_OBJECT_RECOGNIZER_H__

#include <vector>
#include <iostream>
#include <string>
#include <sstream>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/flann_search.hpp>

#include <v4r_config.h>
#include <v4r/common/noise_models.h>
#include <v4r/registration/noise_model_based_cloud_integration.h>
#include <v4r/recognition/multiview_representation.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>

#ifdef HAVE_SIFTGPU
#include <SiftGPU/SiftGPU.h>
#endif

namespace v4r
{

template<typename PointT>
class V4R_EXPORTS MultiviewRecognizer : public Recognizer<PointT>
{
protected:
    typedef Model<PointT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;

    using Recognizer<PointT>::scene_;
    using Recognizer<PointT>::scene_normals_;
    using Recognizer<PointT>::models_;
    using Recognizer<PointT>::model_or_plane_is_verified_;
    using Recognizer<PointT>::transforms_;
    using Recognizer<PointT>::planes_;
    using Recognizer<PointT>::hv_algorithm_;

    using Recognizer<PointT>::poseRefinement;
    using Recognizer<PointT>::hypothesisVerification;

    boost::shared_ptr<MultiRecognitionPipeline<PointT> > rr_;

    typedef boost::property<boost::edge_weight_t, CamConnect> EdgeWeightProperty;
    typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS, size_t, EdgeWeightProperty> Graph;
    typedef boost::graph_traits < Graph >::vertex_descriptor ViewD;
    typedef boost::graph_traits < Graph >::edge_descriptor EdgeD;
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
    Graph gs_;

    pcl::visualization::PCLVisualizer::Ptr go3d_vis_;
    std::vector<int> go_3d_viewports_;

    typedef typename std::map<std::string, ObjectHypothesis<PointT> > symHyp;

    size_t id_;

    typename std::map<size_t, View<PointT> > views_;

    std::string scene_name_;

    symHyp obj_hypotheses_; /// \brief stores keypoint correspondences

    /** \brief Point-to-point correspondence grouping algorithm */
    typename boost::shared_ptr<v4r::GraphGeometricConsistencyGrouping<PointT, PointT> > cg_algorithm_;

    Eigen::Matrix4f pose_;

    pcl::PointCloud<PointT> scene_keypoints_; /// @brief accumulated scene keypoints
    pcl::PointCloud<pcl::Normal> scene_kp_normals_; /// @brief accumulated scene keypoint normals

#ifdef HAVE_SIFTGPU
    boost::shared_ptr<SiftGPU> sift_;
#endif

    bool computeAbsolutePose(CamConnect & e, bool is_first_edge = false);

    /** \brief removes vertices from graph if max_vertices_in_graph has been reached */
    void pruneGraph();

    void correspondenceGrouping();
    
    bool calcSiftFeatures (const pcl::PointCloud<PointT> &cloud_src,
                           pcl::PointCloud<PointT> &sift_keypoints,
                           std::vector< int > &sift_keypoint_indices,
                           std::vector<std::vector<float> > &sift_signatures,
                           std::vector<float> &sift_keypoint_scales);

    typename NguyenNoiseModel<PointT>::Parameter nm_param_;
    typename NMBasedCloudIntegration<PointT>::Parameter nmInt_param_;

public:
    class Parameter : public Recognizer<PointT>::Parameter
    {
    public:
        using Recognizer<PointT>::Parameter::icp_iterations_;
        using Recognizer<PointT>::Parameter::icp_type_;
        using Recognizer<PointT>::Parameter::normal_computation_method_;
        using Recognizer<PointT>::Parameter::voxel_size_icp_;
        using Recognizer<PointT>::Parameter::merge_close_hypotheses_;
        using Recognizer<PointT>::Parameter::merge_close_hypotheses_dist_;
        using Recognizer<PointT>::Parameter::merge_close_hypotheses_angle_;

        bool scene_to_scene_;  /// @brief if true, tries to register two views based on SIFT background matching
        bool use_robot_pose_;   /// @brief if true, uses given pose between two views as relative camera pose estimate
        bool hyp_to_hyp_;   /// @brief if true adds edges for common object hypotheses (not implemented atm)
        bool use_gc_s2s_;   /// @brief defines method used for SIFT background matching
        double distance_same_keypoint_; /// @brief defines the minimum distance between two keypoints (of same model) to be seperated
        double same_keypoint_dot_product_; /// @brief defines the minimum dot distance between the normals of two keypoints (of same model) to be seperated
        int extension_mode_; /// @brief defines method used to extend information from other views (0 = keypoint correspondences (ICRA2015 paper); 1 = full hypotheses only (MVA2015 paper))
        int max_vertices_in_graph_; /// @brief maximum number of views taken into account (views selected in order of latest recognition calls)
        double chop_z_;  /// @brief points with z-component higher than chop_z_ will be ignored (low chop_z reduces computation time and false positives (noise increase with z)
        bool compute_mst_; /// @brief if true, does point cloud registration by SIFT background matching (given scene_to_scene_ == true), by using given pose (if use_robot_pose_ == true) and by common object hypotheses (if hyp_to_hyp_ == true) from all the possible connection a Mimimum Spanning Tree is computed. If false, it only uses the given pose for each point cloud

        Parameter (
                bool scene_to_scene = true,
                bool use_robot_pose = true,
                bool hyp_to_hyp = false,
                bool use_gc_s2s = true,
                double distance_same_keypoint = 0.005f*0.005f,
                double same_keypoint_dot_product = 0.8f,
                int extension_mode = 0,
                int max_vertices_in_graph = 3,
                double chop_z = std::numeric_limits<double>::max(),
                bool compute_mst = true
                ) :

            Recognizer<PointT>::Parameter(),
            scene_to_scene_ (scene_to_scene),
            use_robot_pose_ (use_robot_pose),
            hyp_to_hyp_ (hyp_to_hyp),
            use_gc_s2s_ (use_gc_s2s),
            distance_same_keypoint_ (distance_same_keypoint),
            same_keypoint_dot_product_ (same_keypoint_dot_product),
            extension_mode_ (extension_mode),
            max_vertices_in_graph_ (max_vertices_in_graph),
            chop_z_ (chop_z),
            compute_mst_ (compute_mst)
        {}
    }param_;

    MultiviewRecognizer(const Parameter &p = Parameter()) : Recognizer<PointT>(p){
        param_ = p;
        id_ = 0;
        pose_ = Eigen::Matrix4f::Identity();
    }

    MultiviewRecognizer(int argc, char ** argv);

    /**
     * @brief sets the underlying single-view recognition
     * @param single-view recognizer
     */
    void setSingleViewRecognizer(const typename boost::shared_ptr<MultiRecognitionPipeline<PointT> > & rec)
    {
        rr_ = rec;
    }

    std::string get_scene_name() const
    {
        return scene_name_;
    }

    void set_scene_name(const std::string &name)
    {
        scene_name_ = name;
    }

#ifdef HAVE_SIFTGPU
    void setSift(boost::shared_ptr<SiftGPU> &sift)
    {
        sift_ = sift;
    }
#endif

    void setNoiseModelParameters(const typename NguyenNoiseModel<PointT>::Parameter &p)
    {
        nm_param_ = p;
    }

    void setNoiseModelIntegrationParameters(const typename NMBasedCloudIntegration<PointT>::Parameter &p)
    {
        nmInt_param_ = p;
    }


    /** \brief Sets the algorithm for Correspondence Grouping (Hypotheses generation from keypoint correspondences) */
    void
    setCGAlgorithm (const typename boost::shared_ptr<GraphGeometricConsistencyGrouping<PointT, PointT> > & alg)
    {
      cg_algorithm_ = alg;
    }

    /** \brief sets the camera pose of the input cloud */
    void setCameraPose(const Eigen::Matrix4f &tf)
    {
        pose_ = tf;
    }

    typename boost::shared_ptr<Source<PointT> >
    getDataSource () const
    {
        return rr_->getDataSource();
    }

    /**
     * @brief clears all stored information from previous views
     */
    void cleanUp()
    {
        models_.clear();
        model_or_plane_is_verified_.clear();
        transforms_.clear();
        planes_.clear();
        views_.clear();
        id_ = 0;
    }

    void recognize();
};
}

#endif
