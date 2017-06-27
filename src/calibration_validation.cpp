#include "plane_calibration/calibration_validation.hpp"

#include <ros/console.h>

namespace plane_calibration
{

CalibrationValidation::CalibrationValidation(const CameraModel& camera_model,
                                             const CalibrationParametersPtr& parameters, const Config& config,
                                             const VisualizerInterfacePtr& depth_visualizer) :
    camera_model_(camera_model)
{
  parameters_ = parameters;
  config_ = config;
  depth_visualizer_ = depth_visualizer;
}

void CalibrationValidation::updateConfig(const Config& new_config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = new_config;
}

bool CalibrationValidation::angleOffsetValid(const std::pair<double, double>& angles)
{
  if (std::isnan(angles.first) || std::isnan(angles.second))
  {
    return false;
  }

  if (std::isinf(angles.first) || std::isinf(angles.second))
  {
    return false;
  }

  CalibrationParameters::Parameters parameters = parameters_->getParameters();
  if (angles.first > parameters.max_deviation_ || angles.second > parameters.max_deviation_)
  {
    return false;
  }
  return true;
}

bool CalibrationValidation::groundPlaneFitsData(const Eigen::MatrixXf& ground_plane, const Eigen::MatrixXf& data,
                                                const bool& debug)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Eigen::MatrixXf difference = (ground_plane - data);

  // (nan == nan) gives false
  int not_nan_count = (difference.array() == difference.array()).count();

  //remove nans
  difference = difference.unaryExpr([](float v)
  { return std::isnan(v) ? 0.0f : v;});

  double too_low_distance = 0.0 - config_.too_low_buffer;
  int is_too_low = (difference.array() < too_low_distance).count();
  double too_low_ratio = is_too_low / (double)not_nan_count;

  double mean = difference.sum() / not_nan_count;
  double max_deviation = difference.cwiseAbs().maxCoeff();

  if (debug)
  {
    depth_visualizer_->publishDouble("debug/result_too_low_ratio", too_low_ratio);
    depth_visualizer_->publishDouble("debug/result_mean", mean);
    depth_visualizer_->publishDouble("debug/result_max_deviation", max_deviation);
  }

  if (too_low_ratio > config_.max_too_low_ratio)
  {
    if (debug)
    {
      ROS_WARN_STREAM(
          "[PlaneCalibrationNodelet]: Calibration result ground has too many points below it, ratio (max: " << config_.max_too_low_ratio << "): " << too_low_ratio);
    }
    return false;
  }

  if (std::abs(mean) > config_.max_mean)
  {
    if (debug)
    {
      ROS_WARN_STREAM(
          "[PlaneCalibrationNodelet]: Calibration result abs mean error is too high (max: " << config_.max_mean << "): " << std::abs(mean));
    }
    return false;
  }

  if (max_deviation > config_.max_deviation)
  {
    if (debug)
    {
      ROS_WARN_STREAM(
          "[PlaneCalibrationNodelet]: Calibration result max error is too high (max: " << config_.max_deviation << "): " << max_deviation);
    }
    return false;
  }

  return true;
}

}
/* end namespace */
