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

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"

#include "LAStools/inc/laswriter.hpp" //las writer

namespace cartographer {
namespace io {

// Streams a Las file to disk. The header is written in 'Flush'.
class LasCustomWritingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "write_custom_las";
  LasCustomWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                            PointsProcessor* next,
			    std::string file_name);

  static std::unique_ptr<LasCustomWritingPointsProcessor> FromDictionary(
      const FileWriterFactory& file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~LasCustomWritingPointsProcessor() override {}

  LasCustomWritingPointsProcessor(const LasCustomWritingPointsProcessor&) = delete;
  LasCustomWritingPointsProcessor& operator=(const LasCustomWritingPointsProcessor&) =
      delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

  bool read_matrix_from_CSV(std::string file, Eigen::MatrixXd& matrix, int rows, int cols);

 private:
  PointsProcessor* const next_;

  int64 num_points_;
  bool has_colors_;
  bool has_intensity_;
  bool has_rings_;
  bool has_transformation_matrix_;

  std::unique_ptr<FileWriter> file_;
  //.las file name
  std::string file_name_ = "georeferenced_points.las";
  //opener for .las file
  LASwriteOpener laswriteopener;
  //header for .las file
  LASheader lasheader;
  //.las point object
  LASpoint laspoint;
  // .las writer
  LASwriter* laswriter;
  //transformation matrix
  Eigen::MatrixXd transformation_matrix;
};

}  // namespace io
}  // namespace cartographer
