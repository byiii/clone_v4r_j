#include <v4r/recognition/impl/multi_pipeline_recognizer.hpp>
#include <v4r/recognition/impl/multi_pipeline_recognizer_init.hpp>

template class V4R_EXPORTS v4r::MultiRecognitionPipeline<pcl::PointXYZRGB>;
// maybe this doesn't work because of the specialized template in the initialization function (constructor)
template class V4R_EXPORTS v4r::MultiRecognitionPipeline<pcl::PointXYZ>;
