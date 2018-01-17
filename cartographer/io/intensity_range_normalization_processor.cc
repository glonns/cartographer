
#include "cartographer/io/intensity_range_normalization_processor.h"

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

std::unique_ptr<IntensityRangeNormalizationProcessor>
IntensityRangeNormalizationProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  const std::string frame_id =
      dictionary->HasKey("frame_id") ? dictionary->GetString("frame_id") : "";
  const float max_range = dictionary->GetDouble("max_range");
  const float n_surface_type = dictionary->GetDouble("n_surface_type");
  const float R_reference_distance = dictionary->GetDouble("R_reference_distance");
  const float min_intensity_in_data_set = dictionary->GetDouble("min_intensity_in_data_set");
  //calculate maximum intensity possible in normalization
  float max_inte = dictionary->GetDouble("max_intensity_in_data_set");
  const float max_intensity_in_data_set = max_inte*pow((max_range/R_reference_distance), n_surface_type);
  const float min_intensity_scale_to = dictionary->GetDouble("min_intensity_scale_to");
  const float max_intensity_scale_to = dictionary->GetDouble("max_intensity_scale_to");


  return common::make_unique<IntensityRangeNormalizationProcessor>(
      max_range, n_surface_type, R_reference_distance, min_intensity_in_data_set, max_intensity_in_data_set, min_intensity_scale_to, max_intensity_scale_to, frame_id, next);
}

IntensityRangeNormalizationProcessor::IntensityRangeNormalizationProcessor(
    const float max_range, const float n_surface_type, const float R_reference_distance, const float min_intensity_in_data_set, const float max_intensity_in_data_set,
    const float min_intensity_scale_to, const float max_intensity_scale_to, const std::string& frame_id, PointsProcessor* const next)
    : max_range_(max_range),
      n_surface_type_(n_surface_type),
      R_reference_distance_(R_reference_distance),
      min_intensity_in_data_set_(min_intensity_in_data_set),
      max_intensity_in_data_set_(max_intensity_in_data_set),
      min_intensity_scale_to_(min_intensity_scale_to),
      max_intensity_scale_to_(max_intensity_scale_to),
      frame_id_(frame_id),
      next_(next) {}

void IntensityRangeNormalizationProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  if (!batch->intensities.empty() && (frame_id_.empty() || batch->frame_id == frame_id_)) {
    for (size_t point_nbr = 0; point_nbr < batch->intensities.size(); point_nbr++) {
	//range of current point in batch
	float range = (batch->points[point_nbr] - batch->origin).norm();
	//float range = sqrt(pow(batch->points[point_nbr][0],2.0f) + pow(batch->points[point_nbr][1],2.0f) + pow(batch->points[point_nbr][2],2.0f));
	//normalize intensity based on range
	float intensity_unscaled = batch->intensities[point_nbr] * pow((range/R_reference_distance_), n_surface_type_);
	//scale intensity to wanted range
	float normalized_intensity_scaled = ((max_intensity_scale_to_ - min_intensity_scale_to_)*(intensity_unscaled - min_intensity_in_data_set_))/(max_intensity_in_data_set_ - min_intensity_in_data_set_) + min_intensity_scale_to_;
	batch->intensities[point_nbr] = normalized_intensity_scaled;
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult IntensityRangeNormalizationProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
