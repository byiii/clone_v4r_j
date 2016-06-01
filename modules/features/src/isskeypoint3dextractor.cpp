#include <pcl/common/centroid.h>

#include <pcl/keypoints/iss_3d.h>
#include "v4r/features/isskeypoint3dextractor.h"

namespace v4r{

template<typename PointT>
void
ISSKeypoint3DExtractor<PointT>::filterPlanar (const PointInTPtr & input, std::vector<int> &kp_idx)
{
    //create a search object
    typename pcl::search::Search<PointT>::Ptr tree;

    if (input->isOrganized () && !force_unorganized_)
        tree.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
        tree.reset (new pcl::search::KdTree<PointT> (false));
    tree->setInputCloud (input);

    size_t kept = 0;
    for (size_t i = 0; i < kp_idx.size (); i++)
    {
        std::vector<int>  neighborhood_indices;
        std::vector<float> neighborhood_dist;

        if (tree->radiusSearch (kp_idx[i], radius_, neighborhood_indices, neighborhood_dist))
        {
            EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
            Eigen::Vector4f xyz_centroid;
            EIGEN_ALIGN16 Eigen::Vector3f eigenValues;
            EIGEN_ALIGN16 Eigen::Matrix3f eigenVectors;

            //compute planarity of the region
            computeMeanAndCovarianceMatrix (*input, neighborhood_indices, covariance_matrix, xyz_centroid);
            pcl::eigen33 (covariance_matrix, eigenVectors, eigenValues);

            float eigsum = eigenValues.sum ();
            if (!pcl_isfinite(eigsum))
                PCL_ERROR("Eigen sum is not finite\n");

            float t_planar = threshold_planar_;

            if(z_adaptative_)
                t_planar *= (1 + (std::max(input->points[kp_idx[i]].z,1.f) - 1.f));

            //if ((fabs (eigenValues[0] - eigenValues[1]) < 1.5e-4) || (eigsum != 0 && fabs (eigenValues[0] / eigsum) > 1.e-2))
            if ((fabs (eigenValues[0] - eigenValues[1]) < 1.5e-4) || (eigsum != 0 && fabs (eigenValues[0] / eigsum) > t_planar))
            {
                //region is not planar, add to filtered keypoint
                kp_idx[kept] = kp_idx[i];
                kept++;
            }
        }
    }
    kp_idx.resize (kept);
}

template<typename PointT>
void
ISSKeypoint3DExtractor<PointT>::compute (pcl::PointCloud<PointT> & keypoints)
{
    //compute(keypoint_indices_);
    //pcl::copyPointCloud (*input_, keypoint_indices_, keypoints);

    pcl::ISSKeypoint3D<PointT, PointT> keypoint_extractor;
    // set up parameters
    keypoint_extractor.setThreshold21(threshold21_);
    keypoint_extractor.setThreshold32(threshold32_);
    keypoint_extractor.setMinNeighbors(minNeighbors_);
    keypoint_extractor.setNumberOfThreads(numberOfThreads_);
    keypoint_extractor.setSalientRadius(salientRadius_);
    keypoint_extractor.setNonMaxRadius(nonMaxRadius_);
    //create a search object
    typename pcl::search::Search<PointT>::Ptr tree;
    if (input_->isOrganized () && !force_unorganized_)
        tree.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
        tree.reset (new pcl::search::KdTree<PointT> (false));
    tree->setInputCloud(input_);
    keypoint_extractor.setSearchMethod(tree);
    // set input
    keypoint_extractor.setInputCloud (input_);

    //pcl::PointCloud<int> keypoints_idxes;
    keypoint_extractor.compute(keypoints);

    typename pcl::search::Search<PointT>::Ptr tree2;
    tree2.reset (new pcl::search::KdTree<PointT> (false));
    tree2->setInputCloud(input_);

    for (size_t i = 0; i < keypoints.size (); i++)
    {
        std::vector<int>  neighborhood_indices;
        std::vector<float> neighborhood_dist;

        if (tree->radiusSearch (keypoints.points[i], 1, neighborhood_indices, neighborhood_dist))
        {
            keypoint_indices_.push_back(neighborhood_indices[0]);
        }
        else
            keypoint_indices_.push_back(0);
    }

}


//template<typename PointT>
//void
//ISSKeypoint3DExtractor<PointT>::compute (std::vector<int> & indices)
//{
//    pcl::ISSKeypoint3D<PointT, PointT> keypoint_extractor;
//    // set up parameters
//    keypoint_extractor.setThreshold21(threshold21_);
//    keypoint_extractor.setThreshold32(threshold32_);
//    keypoint_extractor.setMinNeighbors(minNeighbors_);
//    keypoint_extractor.setNumberOfThreads(numberOfThreads_);
//    keypoint_extractor.setSalientRadius(salientRadius_);
//    keypoint_extractor.setNonMaxRadius(nonMaxRadius_);
//    //create a search object
//    typename pcl::search::Search<PointT>::Ptr tree;
//    if (input_->isOrganized () && !force_unorganized_)
//        tree.reset (new pcl::search::OrganizedNeighbor<PointT> ());
//    else
//        tree.reset (new pcl::search::KdTree<PointT> (false));
//    keypoint_extractor.setSearchMethod(tree);
//    // set input
//    keypoint_extractor.setInputCloud (input_);

//    pcl::PointCloud<int> keypoints_idxes;
//    keypoint_extractor.compute();
//    keypoint_extractor.compute (keypoints_idxes);

//    if(pcl_isfinite(max_distance_))
//    {
//        int kept = 0;
//        for(size_t i=0; i < keypoints_idxes.points.size(); i++)
//        {
//            if(input_->points[keypoints_idxes.points[i]].z < max_distance_)
//            {
//                keypoints_idxes.points[kept] = keypoints_idxes.points[i];
//                kept++;
//            }
//        }
//        std::cout << "Filtered " << kept << " out of " << keypoints_idxes.points.size()
//                  << " keypoints based on z-distance " << max_distance_ << "m. " << std::endl;
//        keypoints_idxes.points.resize(kept);
//        keypoints_idxes.width = kept;
//    }

//    keypoint_indices_.resize (keypoints_idxes.points.size ());
//    for (size_t i = 0; i < keypoints_idxes.size (); i++)
//        keypoint_indices_[i] = keypoints_idxes.points[i];

//    if (filter_planar_)
//        filterPlanar (input_, keypoint_indices_);
//    indices = keypoint_indices_;
//}

template class V4R_EXPORTS ISSKeypoint3DExtractor<pcl::PointXYZ>;
template class V4R_EXPORTS ISSKeypoint3DExtractor<pcl::PointXYZRGB>;

}
