#ifndef ISSKEYPOINT3DEXTRACTOR_H
#define ISSKEYPOINT3DEXTRACTOR_H

#include "v4r/features/keypoint_extractor.h"

namespace v4r
{
template<typename PointT>
class ISSKeypoint3DExtractor : public KeypointExtractor<PointT>
{
private:
    typedef typename pcl::PointCloud<PointT>::Ptr PointInTPtr;
    using KeypointExtractor<PointT>::input_;
    using KeypointExtractor<PointT>::radius_;
    using KeypointExtractor<PointT>::keypoint_indices_;

    // isskeypoint3d parameters
    float threshold21_;
    float threshold32_;
    int minNeighbors_;
    int numberOfThreads_;
    float salientRadius_;
    float nonMaxRadius_;
    float angleThreshold_;

    bool filter_planar_;
    float max_distance_;
    float threshold_planar_;
    bool z_adaptative_;
    bool force_unorganized_;

    void
    filterPlanar (const PointInTPtr & input, std::vector<int> &kp_idx);
public:
    ISSKeypoint3DExtractor()
    {
        max_distance_ = std::numeric_limits<float>::infinity();
        threshold_planar_ = 1.e-2;
        z_adaptative_ = false;
        force_unorganized_ = false;

        threshold21_ = 0.975;
        threshold32_ = 0.975;
        minNeighbors_ = 10;
        numberOfThreads_ = 4;
        salientRadius_ = 0.015;
        nonMaxRadius_ = 0.01;
        angleThreshold_ = M_PI/2.1;
    }

    void setForceUnorganized(bool b)
    {
        force_unorganized_ = b;
    }

    void zAdaptative(bool b)
    {
        z_adaptative_ = b;
    }

    void setThresholdPlanar(float t)
    {
        threshold_planar_ = t;
    }

    void setMaxDistance(float d)
    {
        max_distance_ = d;
    }

    void
    setFilterPlanar (bool b)
    {
        filter_planar_ = b;
    }

    void
    setThreshold21(float t)
    {
        threshold21_ = t;
    }

    void
    setThreshold32(float t)
    {
        threshold32_ = t;
    }

    void
    setSalientRadius(float r)
    {
        salientRadius_ = r;
    }

    void
    setNonMaxRadius(float r)
    {
        nonMaxRadius_ = r;
    }

    void
    setMinNeighbors(int n)
    {
        minNeighbors_ = n;
    }

    void
    setAngleThreshold(float t)
    {
        angleThreshold_ = t;
    }

    void
    compute (pcl::PointCloud<PointT> & keypoints);

    void
    compute (std::vector<int> & indices);
};
}
#endif // ISSKEYPOINT3DEXTRACTOR_H
