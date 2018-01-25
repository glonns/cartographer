
#include "cartographer/io/trajectory_ply_writing_points_processor.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_batch.h"
#include "glog/logging.h"
#include "cartographer/transform/transform.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace io {

namespace {

// Writes the PLY header claiming 'num_points' will follow it into
// 'output_file'.

void WriteCustomBinaryPlyHeader(const int64 num_points, FileWriter* const file_writer) {

  std::ostringstream stream;
  stream << "ply\n"
         << "format binary_little_endian 1.0\n"
         << "comment Point cloud hypnotized and smoothly lured from the basket by the almighty snake charmer: Kamikaze Viper\n"
         << "element vertex " << std::setw(15) << std::setfill('0')
         << num_points << "\n"
         << "property float x\n"
         << "property float y\n"
         << "property float z\n"
         << "property float x_rot\n"
         << "property float y_rot\n"
         << "property float z_rot\n"
         << "property double time\n"
         << "end_header\n";
  const std::string out = stream.str();
  CHECK(file_writer->WriteHeader(out.data(), out.size()));
}

void WriteCustomBinaryPlyTrajectory(mapping::proto::Trajectory& trajectory,
                                   FileWriter* const file_writer) {

  if (trajectory.node_size() == 0) {
    return;
  }

  for (const mapping::proto::Trajectory::Node& node : trajectory.node()) {//(const auto& node : trajectory.node()) {
	//get transform
	transform::Rigid3d transform = transform::ToRigid3(node.pose());//trajectory.node(trajectory.node_size() - 1).pose());
	//get xyz and rotations
	float x,y,z,x_rot,y_rot,z_rot;
	double time;
	
	x = float(transform.translation().x());
	y = float(transform.translation().y());
	z = float(transform.translation().z());
	auto euler = transform.rotation().toRotationMatrix().eulerAngles(0, 1, 2);
	x_rot = float(euler[0]);
	y_rot = float(euler[1]);
	z_rot = float(euler[2]);

	//cartographer::common::Time Time_from_ticks = cartographer::common::FromUniversal(node.timestamp());
	//time = 0;//cartographer::common::ToSeconds(time_from_ticks);//trajectory.node(trajectory.node_size() - 1).timestamp();
    	//std::cout << std::setprecision(17) << "Duration_from_ticks " << Duration_from_ticks << std::endl;
	//time = cartographer::common::TimeToUnixSeconds(Time_from_ticks);
	//std::cout << std::setprecision(17) << "trajectory.node(trajectory.node_size() - 1).timestamp() " << trajectory.node(trajectory.node_size() - 1).timestamp() << std::endl;
	//std::cout << std::setprecision(17) << "Time_from_ticks " << Time_from_ticks << std::endl;
	time = cartographer::common::TicksToUnixSeconds(node.timestamp());
    	//std::cout << std::setprecision(17) << "time " << time << std::endl;

	//write trajectory node to file
	char buffer[32];
	memcpy(buffer, &x, sizeof(float));
	memcpy(buffer + 4, &y, sizeof(float));
	memcpy(buffer + 8, &z, sizeof(float));
	memcpy(buffer + 12, &x_rot, sizeof(float));
	memcpy(buffer + 16, &y_rot, sizeof(float));
	memcpy(buffer + 20, &z_rot, sizeof(float));
	memcpy(buffer + 24, &time, sizeof(double));
	CHECK(file_writer->Write(buffer, 32));
  }
}
}  // namespace


TrajectoryPlyWritingPointsProcessor::TrajectoryPlyWritingPointsProcessor(
    std::unique_ptr<FileWriter> file_writer, const std::vector<mapping::proto::Trajectory>& trajectories, PointsProcessor* const next)
    : file_writer_(std::move(file_writer)),
      trajectories_(trajectories),
      next_(next){}

std::unique_ptr<TrajectoryPlyWritingPointsProcessor>
TrajectoryPlyWritingPointsProcessor::FromDictionary(
    const std::vector<mapping::proto::Trajectory>& trajectories,
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {

  return common::make_unique<TrajectoryPlyWritingPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")), trajectories, next);
}

PointsProcessor::FlushResult TrajectoryPlyWritingPointsProcessor::Flush() {
  if(trajectories_.size() > 0){
	//write ply header
	int64 num_trajectory_points = 0;
	for (size_t i = 0; i < trajectories_.size(); ++i) {
		num_trajectory_points = num_trajectory_points + trajectories_[i].node_size();
	}
	WriteCustomBinaryPlyHeader(num_trajectory_points, file_writer_.get());
	//write trajectories
	for (size_t i = 0; i < trajectories_.size(); ++i) {
		WriteCustomBinaryPlyTrajectory(trajectories_[i], file_writer_.get());
	}
  }
  


  CHECK(file_writer_->Close()) << "Closing PLY file_writer failed.";

  switch (next_->Flush()) {
    case FlushResult::kFinished:
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(FATAL) << "PLY generation must be configured to occur after any "
                    "stages that require multiple passes.";
  }
  LOG(FATAL);
  // The following unreachable return statement is needed to avoid a GCC bug
  // described at https://gcc.gnu.org/bugzilla/show_bug.cgi?id=81508
  return FlushResult::kFinished;
}

void TrajectoryPlyWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {

	next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer
