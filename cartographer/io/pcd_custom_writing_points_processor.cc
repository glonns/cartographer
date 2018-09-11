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

#include "cartographer/io/pcd_custom_writing_points_processor.h"

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

// Writes the PCD header claiming 'num_points' will follow it into
// 'output_file'.
void WriteBinaryPcdCustomHeader(const bool has_color, const bool has_intensity, const bool has_ring, const int64 num_points,
                          FileWriter* const file_writer) {
  //color
  std::string color_header_field = !has_color ? "" : " rgb";
  std::string color_header_type = !has_color ? "" : " F";
  std::string color_header_size = !has_color ? "" : " 4";
  std::string color_header_count = !has_color ? "" : " 1";
  //intensity
  std::string intensity_header_field = !has_intensity ? "" : " intensity";
  std::string intensity_header_type = !has_intensity ? "" : " F";
  std::string intensity_header_size = !has_intensity ? "" : " 4";
  std::string intensity_header_count = !has_intensity ? "" : " 1";
  //time
  std::string time_header_field = " time";
  std::string time_header_type = " F";
  std::string time_header_size = " 8";
  std::string time_header_count = " 1";
  //ring
  std::string ring_header_field = !has_ring ? "" : " ring";
  std::string ring_header_type = !has_ring ? "" : " U";
  std::string ring_header_size = !has_ring ? "" : " 1";
  std::string ring_header_count = !has_ring ? "" : " 1";

  std::ostringstream stream;
  stream << "# Point cloud hypnotized and smoothly lured from the basket by the almighty snake charmer: Kamikaze Viper\n"
         << "VERSION .7\n"
         << "FIELDS x y z" << color_header_field << intensity_header_field << time_header_field << ring_header_field << "\n"
         << "SIZE 4 4 4" << color_header_size << intensity_header_size << time_header_size << ring_header_size << "\n"
         << "TYPE F F F" << color_header_type << intensity_header_type << time_header_type << ring_header_type << "\n"
         << "COUNT 1 1 1" << color_header_count << intensity_header_count << time_header_count << ring_header_count << "\n"
         << "WIDTH " << std::setw(15) << std::setfill('0') << num_points << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << std::setw(15) << std::setfill('0') << num_points
         << "\n"
         << "DATA binary\n";
  const std::string out = stream.str();
  file_writer->WriteHeader(out.data(), out.size());
}

void WriteCustomBinaryPcdPointCoordinate(const Eigen::Vector3f& point,
                                   FileWriter* const file_writer) {
  char buffer[12];
  memcpy(buffer, &point[0], sizeof(float));
  memcpy(buffer + 4, &point[1], sizeof(float));
  memcpy(buffer + 8, &point[2], sizeof(float));
  CHECK(file_writer->Write(buffer, 12));
}

void WriteCustomBinaryPcdPointColor(const Uint8Color& color,
                              FileWriter* const file_writer) {
  //std::cout << "color " << (int)color[0] << " " << (int)color[1] << " "<< (int)color[2] << " " << std::endl;
  char buffer[4];
  buffer[0] = color[2];
  buffer[1] = color[1];
  buffer[2] = color[0];
  buffer[3] = 0;
  CHECK(file_writer->Write(buffer, 4));
}

void WriteCustomBinaryPcdPointColorFloat(const FloatColor& color,
                              FileWriter* const file_writer) {
  //std::cout << "color " << color[0] << " " << color[1] << " "<< color[2] << " " << std::endl;
  //float c = (float)(((double)color[0]/256.0)*pow((double)2, (double)16) + ((double)color[1]/256.0)*pow((double)2, (double)8) + ((double)color[2]/256.0));
  //float c = (float)(((double)color[0])*pow((double)2, (double)16) + ((double)color[1])*pow((double)2, (double)8) + ((double)color[2]));
  uint8_t r = color[0], g = color[1], b = color[2];
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  float c = *reinterpret_cast<float*>(&rgb);
  
  char buffer[4];
  memcpy(buffer, &c, sizeof(float));
  CHECK(file_writer->Write(buffer, 4));
  /*buffer[0] = color[2];
  buffer[1] = color[1];
  buffer[2] = color[0];
  buffer[3] = 0;
  CHECK(file_writer->Write(buffer, 4));
  */
}

void WriteCustomBinaryPcdPointTime(const double& time, FileWriter* const file_writer) {
  //std::cout << "time " << std::setprecision(17) << time << std::endl;
  char buffer[8];
  memcpy(buffer, &time, sizeof(double));
  CHECK(file_writer->Write(buffer, 8));
}

void WriteCustomBinaryPcdPointIntensity(const float& intensity, FileWriter* const file_writer) {

  char buffer[4];
  memcpy(buffer, &intensity, sizeof(float));
  CHECK(file_writer->Write(buffer, 4));
}

void WriteCustomBinaryPcdPointRing(const uint16_t& ring, FileWriter* const file_writer) {
  //std::cout << "ring " << (uint8_t)ring << std::endl;
  uint8_t r = (uint8_t)ring;
  char buffer[sizeof(uint8_t)];
  memcpy(buffer, &r, sizeof(uint8_t));
  CHECK(file_writer->Write(buffer, sizeof(uint8_t)));
}

}  // namespace

std::unique_ptr<PcdCustomWritingPointsProcessor>
PcdCustomWritingPointsProcessor::FromDictionary(
    FileWriterFactory file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return absl::make_unique<PcdCustomWritingPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")), next);
}

PcdCustomWritingPointsProcessor::PcdCustomWritingPointsProcessor(
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next)
    : next_(next),
      num_points_(0),
      has_colors_(false),
      has_intensity_(false),
      has_rings_(false),
      file_writer_(std::move(file_writer)) {}

PointsProcessor::FlushResult PcdCustomWritingPointsProcessor::Flush() {
  WriteBinaryPcdCustomHeader(has_colors_, has_intensity_, has_rings_, num_points_, file_writer_.get());
  CHECK(file_writer_->Close());

  switch (next_->Flush()) {
    case FlushResult::kFinished:
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(FATAL) << "PCD generation must be configured to occur after any "
                    "stages that require multiple passes.";
  }
  LOG(FATAL);
}

void PcdCustomWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (batch->points.empty()) {
    next_->Process(std::move(batch));
    return;
  }

  if (num_points_ == 0) {
    has_colors_ = !batch->colors.empty();
    has_intensity_ = !batch->intensities.empty();
    has_rings_ = !batch->rings.empty();
    WriteBinaryPcdCustomHeader(has_colors_, has_intensity_, has_rings_, 0, file_writer_.get());
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
  for (size_t i = 0; i < batch->points.size(); ++i) {
    WriteCustomBinaryPcdPointCoordinate(batch->points[i].position, file_writer_.get());
    if (!batch->colors.empty()) {
      //WriteCustomBinaryPcdPointColor(ToUint8Color(batch->colors[i]), file_writer_.get());
      WriteCustomBinaryPcdPointColorFloat(batch->colors[i], file_writer_.get());
    }
    if (!batch->intensities.empty()) {
      WriteCustomBinaryPcdPointIntensity(batch->intensities[i], file_writer_.get());
    }
    WriteCustomBinaryPcdPointTime(batch->start_time_unix, file_writer_.get());
    if (!batch->rings.empty()) {
      WriteCustomBinaryPcdPointRing(batch->rings[i], file_writer_.get());
    }
    ++num_points_;
  }
  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer
