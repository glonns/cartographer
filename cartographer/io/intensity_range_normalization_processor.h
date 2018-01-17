
#ifndef CARTOGRAPHER_IO_INTENSITY_RANGE_NORMALIZATION_PROCESSOR_H_
#define CARTOGRAPHER_IO_INTENSITY_RANGE_NORMALIZATION_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

class IntensityRangeNormalizationProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "intensity_range_normalization";

  // normalizes point intensity using distance
  IntensityRangeNormalizationProcessor(float max_range, float n_surface_type, float R_reference_distance, float min_intensity_in_data_set, float max_intensity_in_data_set, float min_intensity_scale_to, float max_intensity_scale_to, const std::string& frame_id, PointsProcessor* next);

  static std::unique_ptr<IntensityRangeNormalizationProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~IntensityRangeNormalizationProcessor() override{};

  IntensityRangeNormalizationProcessor(const IntensityRangeNormalizationProcessor&) =
      delete;
  IntensityRangeNormalizationProcessor& operator=(
      const IntensityRangeNormalizationProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const float max_range_;
  const float n_surface_type_;
  const float R_reference_distance_;
  const float min_intensity_in_data_set_;	//min of scaled intensities
  const float max_intensity_in_data_set_;	//max of scaled intensities
  const float min_intensity_scale_to_;	//min of unscaled intensities
  const float max_intensity_scale_to_;	//max of unscaled intensities
  
  const std::string frame_id_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_
