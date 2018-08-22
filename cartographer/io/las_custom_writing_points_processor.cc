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

#include "cartographer/io/las_custom_writing_points_processor.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_batch.h"
#include "glog/logging.h"

#include <boost/filesystem.hpp>

namespace cartographer {
namespace io {

namespace {


}  // namespace

bool LasCustomWritingPointsProcessor::read_matrix_from_CSV(std::string file, Eigen::MatrixXd& matrix, int rows, int cols) {

  std::ifstream in(file);
  std::string line;

  int row = 0;

  matrix = Eigen::MatrixXd(rows, cols);

  if (in.is_open()) {
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::istream_iterator<std::string> begin(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> vstrings(begin, end);

        for (size_t col = 0; col < vstrings.size(); col++){
            matrix(row, col) = std::stod(vstrings.at(col));
        } 
    row++;
    }

    in.close();
  }else{
      return false;
  } 
  return true;
}

std::unique_ptr<LasCustomWritingPointsProcessor>
LasCustomWritingPointsProcessor::FromDictionary(
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
	std::string name = dictionary->GetString("filename");
  return common::make_unique<LasCustomWritingPointsProcessor>(
      file_writer_factory(name), next, name);
}

LasCustomWritingPointsProcessor::LasCustomWritingPointsProcessor(
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next, std::string file_name)
    : next_(next),
      num_points_(0),
      has_colors_(false),
      has_intensity_(true),
      has_rings_(false),
      file_(std::move(file_writer)),
      file_name_(file_name) {}

PointsProcessor::FlushResult LasCustomWritingPointsProcessor::Flush() {

  // update the header
  laswriter->update_header(&lasheader, TRUE);
  // close the writer
  I64 total_bytes = laswriter->close();
  std::cout << "las bytes written " << total_bytes << std::endl;

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

void LasCustomWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (batch->points.empty()) {
    next_->Process(std::move(batch));
    return;
  }

  if (num_points_ == 0) {
    //get file names
    boost::filesystem::path las_file_path = boost::filesystem::path(file_->GetFilename());
    boost::filesystem::path georeference_folder_path =  las_file_path.parent_path() / boost::filesystem::path("georeference/");
    boost::filesystem::path transformationmatrix_file_path =  georeference_folder_path / boost::filesystem::path("transformation_matrix.txt");

    // load transformation matrix from file
    has_transformation_matrix_ = LasCustomWritingPointsProcessor::read_matrix_from_CSV(transformationmatrix_file_path.string(), transformation_matrix,4,4);
    if(!has_transformation_matrix_){
        std::cout << "Could not read the transformation matrix: " << transformationmatrix_file_path.string() << std::endl << "Exporting ungeoreferenced files" << std::endl;
    }

    // set .las file name
    laswriteopener.set_file_name(las_file_path.string().c_str());
    // init header
    lasheader.x_scale_factor = 0.001;
    lasheader.y_scale_factor = 0.001;
    lasheader.z_scale_factor = 0.001;
    //add translation from transformation matrix to offset
    if(has_transformation_matrix_){
	lasheader.x_offset = transformation_matrix(0,3);
    	lasheader.y_offset = transformation_matrix(1,3);
    	lasheader.z_offset = transformation_matrix(2,3);
    }else{
	lasheader.x_offset = 0.0;
    	lasheader.y_offset = 0.0;
    	lasheader.z_offset = 0.0;
    }

    lasheader.point_data_format = 3;
    lasheader.point_data_record_length = 34;
    //open writing
    laswriter = laswriteopener.open(&lasheader);

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
  
  //init LASpoint 
  LASpoint laspoint;
  laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

  //get rotation matrix
  Eigen::MatrixXd R(3,3);
  R = transformation_matrix.block<3,3>(0,0);
  for (size_t ii = 0; ii < batch->points.size(); ++ii) {
    // populate the .las point
	//coordinates
	Eigen::Vector3d coordinates = Eigen::Vector3d((double)batch->points[ii].position[0], (double)batch->points[ii].position[1], (double)batch->points[ii].position[2]);
	//rotate
	if(has_transformation_matrix_){
		coordinates = R*coordinates;
	}	
	
	//X coordinate
	double xdouble = coordinates[0];
        //round
	I32 x;
	if (xdouble >= lasheader.x_scale_factor){
		x=(I32)((xdouble-lasheader.x_scale_factor)/lasheader.x_scale_factor+0.5);
	}else{
		x=(I32)((xdouble-lasheader.x_scale_factor)/lasheader.x_scale_factor-0.5);
	}
    	laspoint.set_X(x);
	//Y coordinate
	double ydouble = coordinates[1];
        //round
	I32 y;
	if (ydouble >= lasheader.y_scale_factor){
		y=(I32)((ydouble-lasheader.y_scale_factor)/lasheader.y_scale_factor+0.5);
	}else{
		y=(I32)((ydouble-lasheader.y_scale_factor)/lasheader.y_scale_factor-0.5);
	}
	laspoint.set_Y(y);
	//Z coordinate
	double zdouble = coordinates[2];
        //round
	I32 z;
	if (zdouble >= lasheader.z_scale_factor){
		z=(I32)((zdouble-lasheader.z_scale_factor)/lasheader.z_scale_factor+0.5);
	}else{
		z=(I32)((zdouble-lasheader.z_scale_factor)/lasheader.z_scale_factor-0.5);
	}
    	laspoint.set_Z(z);
	//intensity
    	laspoint.set_intensity((U16)batch->intensities[ii]);
	//time
	laspoint.set_gps_time((F64)batch->start_time_unix);
	//ring as scan direction
	laspoint.set_scan_angle_rank((I8)batch->rings[ii]);
	//echo
	laspoint.set_return_number((U8)batch->echoes[ii]);
        //number of echoes
        laspoint.set_number_of_returns((U8)batch->numechoes[ii]);
	//rgb + nir
	U16 rgb[4] = { (U16)((double)batch->colors[ii][0]*256.0), (U16)((double)batch->colors[ii][1]*256.0), (U16)((double)batch->colors[ii][2]*256.0), (U16)0 };
	laspoint.set_RGB(rgb);
	//std::cout << "has_colors_ " << has_colors_ << std::endl;
	//if(has_colors_){
	//}

    	// write the point
    	laswriter->write_point(&laspoint);
    	// add it to the inventory
    	laswriter->update_inventory(&laspoint);
	
	++num_points_;
  }
  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer
