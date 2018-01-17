
#ifndef CARTOGRAPHER_IO_TRAJECTORY_PLY_WRITING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_TRAJECTORY_PLY_WRITING_POINTS_PROCESSOR_H_

#include <fstream>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/mapping/proto/trajectory.pb.h"



namespace cartographer {
namespace io {

// Streams a PCD file to disk. The header is written in 'Flush'.
class TrajectoryPlyWritingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "write_trajectory_ply";
  
  TrajectoryPlyWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                            const std::vector<mapping::proto::Trajectory>& trajectories,
                            PointsProcessor* const next);

  static std::unique_ptr<TrajectoryPlyWritingPointsProcessor> FromDictionary(
      const std::vector<mapping::proto::Trajectory>& trajectories,
      const FileWriterFactory& file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~TrajectoryPlyWritingPointsProcessor() override {}

  TrajectoryPlyWritingPointsProcessor(const TrajectoryPlyWritingPointsProcessor&) = delete;
  TrajectoryPlyWritingPointsProcessor& operator=(const TrajectoryPlyWritingPointsProcessor&) =
      delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  std::unique_ptr<FileWriter> file_writer_;
  std::vector<mapping::proto::Trajectory> trajectories_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer
#endif
