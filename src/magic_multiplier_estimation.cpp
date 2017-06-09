#include "plane_calibration/magic_multiplier_estimation.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <ecl/geometry/angle.hpp>
#include "plane_calibration/plane_to_depth_image.hpp"

namespace plane_calibration
{

MagicMultiplierEstimation::MagicMultiplierEstimation(CameraModel camera_model, PlaneCalibrationPtr plane_calibration,
                                                     double z_offset, double max_x_angle, double max_y_angle,
                                                     double step_size)
{
  camera_model_.update(camera_model.getParameters());
  plane_calibration_ = plane_calibration;

  z_offset_ = z_offset;
  max_x_angle_ = max_x_angle;
  max_y_angle_ = max_y_angle;
  step_size_ = step_size;
}

MagicMultiplierEstimation::Result MagicMultiplierEstimation::estimate(bool calculate_errors)
{
  std::cout << "MagicMultiplierEstimation: Running estimation with parameters:" << std::endl;
  std::cout << "MagicMultiplierEstimation: z_offset_:    " << z_offset_ << std::endl;
  std::cout << "MagicMultiplierEstimation: max_x_angle_: " << max_x_angle_ << std::endl;
  std::cout << "MagicMultiplierEstimation: max_y_angle_: " << max_y_angle_ << std::endl;
  std::cout << "MagicMultiplierEstimation: step_size_:   " << step_size_ << std::endl;
  Result result;

  double x_distance_diff = getXDistanceDiff(max_x_angle_);
  double y_distance_diff = getYDistanceDiff(max_y_angle_);

  result.multiplier_x = max_x_angle_ / x_distance_diff;
  result.multiplier_y = max_y_angle_ / y_distance_diff;

  if (calculate_errors)
  {
    std::pair<double, double> max_abs_errors = getXYMaxAbsErrors(result.multiplier_x, result.multiplier_y);
    result.max_error_x_degree = ecl::radians_to_degrees(max_abs_errors.first);
    result.max_error_y_degree = ecl::radians_to_degrees(max_abs_errors.second);

    std::pair<double, double> max_abs_cross_errors = getMaxAbsCrossErrors(result.multiplier_x, result.multiplier_y);
    result.max_cross_error_x_degree = ecl::radians_to_degrees(max_abs_cross_errors.first);
    result.max_cross_error_y_degree = ecl::radians_to_degrees(max_abs_cross_errors.second);
  }

  return result;
}

std::pair<double, double> MagicMultiplierEstimation::getXYMaxAbsErrors(double magic_x_multiplier,
                                                                       double magic_y_multiplier) const
{
  std::pair<double, double> x_error = getXYMaxAbsError(magic_x_multiplier, magic_y_multiplier, step_size_, 0.0);
  std::pair<double, double> y_error = getXYMaxAbsError(magic_x_multiplier, magic_y_multiplier, 0.0, step_size_);

  double max_x_error = std::max(x_error.first, x_error.second);
  double max_y_error = std::max(y_error.first, y_error.second);

  return std::make_pair(max_x_error, max_y_error);
}

std::pair<double, double> MagicMultiplierEstimation::getMaxAbsCrossErrors(double magic_x_multiplier,
                                                                          double magic_y_multiplier) const
{
  return getXYMaxAbsError(magic_x_multiplier, magic_y_multiplier, step_size_, step_size_);
}

std::pair<double, double> MagicMultiplierEstimation::getXYMaxAbsError(double magic_x_multiplier,
                                                                      double magic_y_multiplier, double x_step_size,
                                                                      double y_step_size) const
{
  if (x_step_size == 0.0 && y_step_size == 0.0)
  {
    return std::make_pair(0.0, 0.0);
  }

  double x_angle = 0.0;
  double y_angle = 0.0;
  double max_x_error = 0.0;
  double max_y_error = 0.0;

  while (std::abs(x_angle) <= max_x_angle_ && std::abs(y_angle) <= max_y_angle_)
  {
    auto distances = getXYDistanceDiff(x_angle, y_angle);

    double x_angle_estimation = magic_x_multiplier * distances.first;
    double x_error = std::abs(x_angle_estimation - x_angle);

    if (x_error > max_x_error)
    {
      max_x_error = x_error;
    }

    double y_angle_estimation = magic_y_multiplier * distances.second;
    double y_error = std::abs(y_angle_estimation - y_angle);

    if (y_error > max_y_error)
    {
      max_y_error = y_error;
    }

    x_angle += x_step_size;
    y_angle += y_step_size;
  }

  return std::make_pair(max_x_error, max_y_error);
}

double MagicMultiplierEstimation::getXDistanceDiff(double angle) const
{
  return getXYDistanceDiff(angle, 0.0).first;
}

double MagicMultiplierEstimation::getYDistanceDiff(double angle) const
{
  return getXYDistanceDiff(0.0, angle).second;
}

std::pair<double, double> MagicMultiplierEstimation::getXYDistanceDiff(double x_angle, double y_angle) const
{
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(x_angle, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(y_angle, Eigen::Vector3d::UnitY());

  double zero_offset = 0.0;
  Eigen::Affine3d transform = Eigen::Translation3d(Eigen::Vector3d(zero_offset, zero_offset, z_offset_)) * rotation;
  Eigen::MatrixXf plane_image_matrix = PlaneToDepthImage::convert(transform, camera_model_.getParameters());

  return plane_calibration_->getXYDistanceDiff(plane_image_matrix);
}

} /* end namespace */
