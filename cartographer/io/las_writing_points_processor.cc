/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/las_writing_points_processor.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {

// Writes the Las header claiming 'num_points' will follow it into
// 'output_file'.
//void WriteBinaryLasHeader(const bool has_color, const bool has_intensity, const int64 num_points, FileWriter* const file_writer) {
void WriteBinaryLasHeader(const bool has_color, const bool has_intensity, const bool has_ring, const int64 num_points, FileWriter* const file_writer) {
  std::string color_header = !has_color ? ""
                                        : "property uchar red\n"
                                          "property uchar green\n"
                                          "property uchar blue\n";
  std::string intensity_header = !has_intensity ? ""
                                        : "property float intensity\n";
  std::string ring_header = !has_ring ? ""
                                        : "property int ring\n";

  // init header
  LASheader lasheader;
  lasheader.x_scale_factor = 0.001;
  lasheader.y_scale_factor = 0.001;
  lasheader.z_scale_factor = 0.001;
  lasheader.x_offset = 0.0;
  lasheader.y_offset = 0.0;
  lasheader.z_offset = 0.0;
  lasheader.point_data_format = 1;
  lasheader.point_data_record_length = 28;
}

void WriteBinaryLasPointCoordinate(const Eigen::Vector3f& point,
                                   FileWriter* const file_writer) {
  char buffer[12];
  memcpy(buffer, &point[0], sizeof(float));
  memcpy(buffer + 4, &point[1], sizeof(float));
  memcpy(buffer + 8, &point[2], sizeof(float));
  CHECK(file_writer->Write(buffer, 12));
}

void WriteBinaryLasPointColor(const Uint8Color& color,
                              FileWriter* const file_writer) {
  CHECK(file_writer->Write(reinterpret_cast<const char*>(color.data()),
                           color.size()));
}

void WriteBinaryLasPointTime(const double& time, FileWriter* const file_writer) {

  char buffer[8];
  memcpy(buffer, &time, sizeof(double));
  CHECK(file_writer->Write(buffer, 8));
}

void WriteBinaryLasPointIntensity(const float& intensity, FileWriter* const file_writer) {

  char buffer[4];
  memcpy(buffer, &intensity, sizeof(float));
  CHECK(file_writer->Write(buffer, 4));
}

void WriteBinaryLasPointRing(const int& ring, FileWriter* const file_writer) {

  char buffer[sizeof(int)];
  memcpy(buffer, &ring, sizeof(int));
  CHECK(file_writer->Write(buffer, sizeof(int)));
}

}  // namespace

std::unique_ptr<LasWritingPointsProcessor>
LasWritingPointsProcessor::FromDictionary(
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return absl::make_unique<LasWritingPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")), next);
}

LasWritingPointsProcessor::LasWritingPointsProcessor(
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next)
    : next_(next),
      num_points_(0),
      has_colors_(false),
      has_intensity_(false),
      has_rings_(false),
      file_(std::move(file_writer)) {}

/*
PointsProcessor::FlushResult LasWritingPointsProcessor::Flush() {
  WriteBinaryLasHeader(has_colors_, has_intensity_, num_points_, file_.get());
  CHECK(file_->Close()) << "Closing Las file_writer failed.";

  switch (next_->Flush()) {
    case FlushResult::kFinished:
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(FATAL) << "Las generation must be configured to occur after any "
                    "stages that require multiple passes.";
  }
  LOG(FATAL);
}
*/

PointsProcessor::FlushResult LasWritingPointsProcessor::Flush() {
  WriteBinaryLasHeader(has_colors_, has_intensity_, has_rings_, num_points_, file_.get());
  CHECK(file_->Close()) << "Closing Las file_writer failed.";

  switch (next_->Flush()) {
    case FlushResult::kFinished:
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(FATAL) << "Las generation must be configured to occur after any "
                    "stages that require multiple passes.";
  }
  LOG(FATAL);
}

void LasWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (batch->points.empty()) {
    next_->Process(std::move(batch));
    return;
  }

  if (num_points_ == 0) {

    has_colors_ = !batch->colors.empty();
    has_intensity_ = !batch->intensities.empty();
    has_rings_ = !batch->rings.empty();

    //WriteBinaryLasHeader(has_colors_, has_intensity_, has_rings_, 0, file_.get());
  }
  if (has_colors_) {
    CHECK_EQ(batch->points.size(), batch->colors.size())
        << "First PointsBatch had colors, but encountered one without. "
           "frame_id: "
        << batch->frame_id;
  }
  if (has_intensity_) {
    CHECK_EQ(batch->points.size(), batch->intensities.size())
        << "First PointsBatch had intesities, but encountered one without. "
           "frame_id: "
        << batch->frame_id;
  }
  if (has_rings_) {
    CHECK_EQ(batch->points.size(), batch->rings.size())
        << "First PointsBatch had rings, but encountered one without. "
           "frame_id: "
        << batch->frame_id;
  }
  /*for (size_t i = 0; i < batch->points.size(); ++i) {
    WriteBinaryLasPointTime(ToUniversalDouble(batch->start_time), file_.get());
    WriteBinaryLasPointCoordinate(batch->points[i], file_.get());
    if (has_colors_) {
      WriteBinaryLasPointColor(ToUint8Color(batch->colors[i]), file_.get());
    }
    if (has_intensity_) {
      WriteBinaryLasPointIntensity(batch->intensities[i], file_.get());
    }
    if (has_rings_) {
      WriteBinaryLasPointRing(batch->rings[i], file_.get());
    }
    ++num_points_;
  }
  */
  /*
  std::cout << "writing header .las" << std::endl;
  // init header
  LASheader lasheader;
  lasheader.x_scale_factor = 0.001;
  lasheader.y_scale_factor = 0.001;
  lasheader.z_scale_factor = 0.001;
  lasheader.x_offset = 0.0;
  lasheader.y_offset = 0.0;
  lasheader.z_offset = 0.0;
  lasheader.point_data_format = 1;
  lasheader.point_data_record_length = 28;

  //open file for writing
  LASwriteOpener laswriteopener;
  laswriteopener.set_file_name("file.las");
  LASwriter* laswriter = laswriteopener.open(&lasheader);
  if (laswriter == 0){
	std::cout << "ERROR: could not open laswriter\n" << std::endl;
  }

  // init point 
  LASpoint laspoint;
  laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);
  for (size_t i = 0; i < batch->points.size(); ++i) {
	laspoint.set_X(i);
	laspoint.set_Y(i);
	laspoint.set_Z(i);
	laspoint.set_intensity((U16)i);
	laspoint.set_gps_time(0.0006*i);

    	// write the point
    	laswriter->write_point(&laspoint);

    	// add it to the inventory
    	laswriter->update_inventory(&laspoint);
  }
  */
  // update the header
  //laswriter->update_header(&lasheader, TRUE);

  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer
