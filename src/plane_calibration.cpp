#include "plane_calibration/plane_calibration.hpp"

#include <iostream>
#include <ecl/geometry/angle.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>

#include "plane_calibration/plane_to_depth_image.hpp"

namespace plane_calibration
{

PlaneCalibration::PlaneCalibration(const CameraModel& camera_model, const CalibrationParametersPtr& parameters,
                                   const VisualizerInterfacePtr& depth_visualizer) :
    plane_to_depth_(camera_model.getParameters())
{
  camera_model_.update(camera_model.getParameters());
  parameters_ = parameters;

  max_deviation_planes_ = std::make_shared<DeviationPlanes>(plane_to_depth_, depth_visualizer);
  temp_deviation_planes_ = std::make_shared<DeviationPlanes>(plane_to_depth_, depth_visualizer);

  depth_visualizer_ = depth_visualizer;
}

std::pair<double, double> PlaneCalibration::calibrate(const Eigen::MatrixXf& filtered_depth_matrix, const int& iterations)
{
  std::lock_guard<std::mutex> lock(mutex_);

  CalibrationParameters::Parameters updated_parameters;
  bool parameters_updated = parameters_->getUpdatedParameters(updated_parameters);

  if (parameters_updated)
  {
    max_deviation_planes_->update(updated_parameters);
  }

  double x_angle_offset = 0.0;
  double y_angle_offset = 0.0;
  double deviation_buffer = ecl::degrees_to_radians(0.5);

  CalibrationParameters::Parameters parameters(updated_parameters);

  std::pair<double, double> angle_offset_estimation = max_deviation_planes_->estimateAngles(filtered_depth_matrix);
  x_angle_offset += angle_offset_estimation.first;
  y_angle_offset += angle_offset_estimation.second;

//  std::cout << "max_angle_deviation: " << ecl::radians_to_degrees(parameters.deviation_) << std::endl;
//  std::cout << "0 angles : " << ecl::radians_to_degrees(angle_offset_estimation.first) << ", "
//      << ecl::radians_to_degrees(angle_offset_estimation.second) << std::endl;
//  std::cout << "0 full   : " << ecl::radians_to_degrees(x_angle_offset) << ", "
//      << ecl::radians_to_degrees(y_angle_offset) << std::endl;

  parameters.rotation_ = parameters.rotation_
      * Eigen::AngleAxisd(angle_offset_estimation.first, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(angle_offset_estimation.second, Eigen::Vector3d::UnitY());

  for (int i = 0; i < iterations; ++i)
  {
    double max_angle_deviation = std::max(std::abs(angle_offset_estimation.first),
                                          std::abs(angle_offset_estimation.second));
    parameters.deviation_ = max_angle_deviation + deviation_buffer;

    temp_deviation_planes_->update(parameters);
    angle_offset_estimation = temp_deviation_planes_->estimateAngles(filtered_depth_matrix);

    parameters.rotation_ = parameters.rotation_
        * Eigen::AngleAxisd(angle_offset_estimation.first, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(angle_offset_estimation.second, Eigen::Vector3d::UnitY());

    x_angle_offset += angle_offset_estimation.first;
    y_angle_offset += angle_offset_estimation.second;

//    std::cout << "max_angle_deviation: " << ecl::radians_to_degrees(parameters.deviation_) << std::endl;
//    std::cout << i + 1 << " angles : " << ecl::radians_to_degrees(angle_offset_estimation.first) << ", "
//        << ecl::radians_to_degrees(angle_offset_estimation.second) << std::endl;
//    std::cout << i + 1 << " full   : " << ecl::radians_to_degrees(x_angle_offset) << ", "
//        << ecl::radians_to_degrees(y_angle_offset) << std::endl;
  }

  return std::make_pair(x_angle_offset, y_angle_offset);
}

} /* end namespace */
