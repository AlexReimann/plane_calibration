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

  if (parameters->getParameters().precompute_planes_)
  {
    precomputed_planes_ = std::make_shared<Planes>(parameters_->getParameters(), plane_to_depth_);
  }
}

std::pair<double, double> PlaneCalibration::calibrate(const Eigen::MatrixXf& filtered_depth_matrix,
                                                      const int& iterations)
{
  std::lock_guard<std::mutex> lock(mutex_);

  CalibrationParameters::Parameters updated_parameters;
  bool parameters_updated = parameters_->getUpdatedParameters(updated_parameters);

  if (parameters_updated)
  {
    max_deviation_planes_->update(updated_parameters);
    precomputed_planes_ = std::make_shared<Planes>(parameters_->getParameters(), plane_to_depth_);
  }

  double x_angle_offset = 0.0;
  double y_angle_offset = 0.0;

  temp_parameters_ = updated_parameters;

  std::pair<double, double> angle_offset_estimation = max_deviation_planes_->estimateAngles(filtered_depth_matrix);
  x_angle_offset += angle_offset_estimation.first;
  y_angle_offset += angle_offset_estimation.second;

//  std::cout << "max_angle_deviation: " << ecl::radians_to_degrees(parameters.deviation_) << std::endl;
//  std::cout << "0 angles : " << ecl::radians_to_degrees(angle_offset_estimation.first) << ", "
//      << ecl::radians_to_degrees(angle_offset_estimation.second) << std::endl;
//  std::cout << "0 full   : " << ecl::radians_to_degrees(x_angle_offset) << ", "
//      << ecl::radians_to_degrees(y_angle_offset) << std::endl;

  temp_parameters_.rotation_ = temp_parameters_.rotation_
      * Eigen::AngleAxisd(angle_offset_estimation.first, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(angle_offset_estimation.second, Eigen::Vector3d::UnitY());

  for (int i = 0; i < iterations; ++i)
  {
    double max_angle_deviation = std::max(std::abs(angle_offset_estimation.first),
                                          std::abs(angle_offset_estimation.second));
    if (!temp_parameters_.precompute_planes_)
    {
      angle_offset_estimation = estimateAngles(filtered_depth_matrix, angle_offset_estimation, max_angle_deviation);
    }
    else
    {
      angle_offset_estimation = estimateAngles(filtered_depth_matrix, x_angle_offset, y_angle_offset,
                                               max_angle_deviation);
    }

    temp_parameters_.rotation_ = temp_parameters_.rotation_
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

std::pair<double, double> PlaneCalibration::estimateAngles(const Eigen::MatrixXf& filtered_depth_matrix,
                                                           const std::pair<double, double>& last_estimation,
                                                           const double& deviation)
{
  double deviation_buffer = ecl::degrees_to_radians(0.5);
  temp_parameters_.deviation_ = deviation + deviation_buffer;

  temp_deviation_planes_->update(temp_parameters_);
  return temp_deviation_planes_->estimateAngles(filtered_depth_matrix);
}

std::pair<double, double> PlaneCalibration::estimateAngles(const Eigen::MatrixXf& filtered_depth_matrix,
                                                           const double& x_angle_offset, const double& y_angle_offset,
                                                           const double& deviation)
{
  std::pair<MatrixPlanePtr, MatrixPlanePtr> x_planes_ = precomputed_planes_->getFittingXTiltPlanes(x_angle_offset,
                                                                                                   deviation);
  std::pair<MatrixPlanePtr, MatrixPlanePtr> y_planes_ = precomputed_planes_->getFittingYTiltPlanes(y_angle_offset,
                                                                                                   deviation);

  bool matrix_has_nans = false;
  double x_distance = DeviationPlanes::getDistance(*x_planes_.first, *x_planes_.second, matrix_has_nans);
  double y_distance = DeviationPlanes::getDistance(*y_planes_.first, *y_planes_.second, matrix_has_nans);

  double x_magic_multiplier = deviation / x_distance;
  double y_magic_multiplier = deviation / y_distance;

  matrix_has_nans = true;
  double x_distance_positive = DeviationPlanes::getDistance(*x_planes_.first, filtered_depth_matrix, matrix_has_nans);
  double x_distance_negative = DeviationPlanes::getDistance(*x_planes_.second, filtered_depth_matrix, matrix_has_nans);
  double y_distance_positive = DeviationPlanes::getDistance(*y_planes_.first, filtered_depth_matrix, matrix_has_nans);
  double y_distance_negative = DeviationPlanes::getDistance(*y_planes_.second, filtered_depth_matrix, matrix_has_nans);

  double x_distance_diff = x_distance_negative - x_distance_positive;
  double y_distance_diff = y_distance_negative - y_distance_positive;

  double px_estimation = x_magic_multiplier * x_distance_diff;
  double py_estimation = y_magic_multiplier * y_distance_diff;

  return std::make_pair(px_estimation, py_estimation);
}

} /* end namespace */
