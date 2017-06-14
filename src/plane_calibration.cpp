#include "plane_calibration/plane_calibration.hpp"

#include <iostream>
#include <ecl/geometry/angle.hpp>
#include "plane_calibration/plane_to_depth_image.hpp"

namespace plane_calibration
{

PlaneCalibration::PlaneCalibration(const CameraModel& camera_model, const CalibrationParametersPtr& parameters) :
    plane_to_depth_(camera_model.getParameters()), max_deviation_planes_(plane_to_depth_), deviation_planes_(plane_to_depth_), temp_deviation_planes_(
        plane_to_depth_)
{
  camera_model_.update(camera_model.getParameters());
  parameters_ = parameters;
}

std::pair<double, double> PlaneCalibration::calibrate(const Eigen::MatrixXf& filtered_depth_matrix)
{
  std::lock_guard<std::mutex> lock(mutex_);

  CalibrationParameters::Parameters updated_parameters;
  bool parameters_updated = parameters_->getUpdatedParameters(updated_parameters);

  if (parameters_updated)
  {
    deviation_planes_.update(updated_parameters);
    max_deviation_planes_.update(updated_parameters);
  }

  if (!parameters_updated)
  {
    Errors current_errors = getErrorsToBestEstimation(filtered_depth_matrix);
    bool current_errors_above_threshold = true; //TODO
    if (!current_errors_above_threshold)
    {
      return best_estimated_angles_;
    }
  }

  double x_angle_offset = 0.0;
  double y_angle_offset = 0.0;
  double deviation_buffer = ecl::degrees_to_radians(0.5);

  CalibrationParameters::Parameters parameters(updated_parameters);
  std::pair<double, double> angle_offset_estimation = max_deviation_planes_.estimateAngles(filtered_depth_matrix);
  x_angle_offset += angle_offset_estimation.first;
  y_angle_offset += angle_offset_estimation.second;

  parameters.rotation_ = parameters.rotation_
      * Eigen::AngleAxisd(angle_offset_estimation.first, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(angle_offset_estimation.second, Eigen::Vector3d::UnitY());

  int iterations = 3;
  for (int i = 0; i < iterations; ++i)
  {
    double max_angle_deviation = std::max(std::abs(angle_offset_estimation.first),
                                          std::abs(angle_offset_estimation.second));
    parameters.deviation_ = max_angle_deviation + deviation_buffer;

    temp_deviation_planes_.update(parameters);
    angle_offset_estimation = temp_deviation_planes_.estimateAngles(filtered_depth_matrix);

    parameters.rotation_ = parameters.rotation_
        * Eigen::AngleAxisd(angle_offset_estimation.first, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(angle_offset_estimation.second, Eigen::Vector3d::UnitY());
    x_angle_offset += angle_offset_estimation.first;
    y_angle_offset += angle_offset_estimation.second;
  }

  temp_estimated_plane_ = plane_to_depth_.convert(parameters.getTransform());
  Errors errors = getErrors(temp_estimated_plane_, filtered_depth_matrix);

  bool errors_above_threshold = false; //TODO
  bool angle_offsets_too_high = std::abs(x_angle_offset) > parameters.max_deviation_
      || std::abs(y_angle_offset) > parameters.max_deviation_;

  if (errors_above_threshold || angle_offsets_too_high)
  {
    return best_estimated_angles_;
  }

  deviation_planes_ = temp_deviation_planes_;
  best_estimated_plane_ = temp_estimated_plane_;
  best_estimated_angles_ = std::make_pair(x_angle_offset, y_angle_offset);
  return best_estimated_angles_;
}

PlaneCalibration::Errors PlaneCalibration::getErrorsToBestEstimation(const Eigen::MatrixXf& matrix)
{
  return getErrors(best_estimated_plane_, matrix);
}

PlaneCalibration::Errors PlaneCalibration::getErrors(const Eigen::MatrixXf& plane, const Eigen::MatrixXf& matrix)
{
  Eigen::MatrixXf difference = (matrix - plane).cwiseAbs();

  // (nan == nan) gives false
  int not_nan_count = (difference.array() == difference.array()).count();

  //remove nans
  difference = difference.unaryExpr([](float v)
  { return std::isnan(v) ? 0.0 : v;});

  double mean = difference.sum() / not_nan_count;

  Errors errors;
  errors.mean = mean;
  errors.max = difference.maxCoeff();

  return errors;
}

} /* end namespace */
