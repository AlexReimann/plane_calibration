#ifndef plane_calibration_SRC_CALIBRATION_VALIDATION_HPP_
#define plane_calibration_SRC_CALIBRATION_VALIDATION_HPP_

#include <memory>
#include <utility>
#include <Eigen/Dense>

#include "camera_model.hpp"
#include "calibration_parameters.hpp"
#include "visualizer_interface.hpp"

namespace plane_calibration
{

class CalibrationValidation
{
public:
  class Config
  {
  public:
    Config()
    {
      too_low_buffer = 0.0;
      max_too_low_ratio = 0.0;
      max_mean = 0.0;
    }

    double too_low_buffer;
    double max_too_low_ratio;
    double max_mean;
  };

  CalibrationValidation(const CameraModel& camera_model, const CalibrationParametersPtr& parameters,
                        const Config& config, const VisualizerInterfacePtr& depth_visualizer);

  void updateConfig(const Config& new_config);

  bool angleOffsetValid(const std::pair<double, double>& angles);
  bool groundPlaneFitsData(const Eigen::MatrixXf& ground_plane, const Eigen::MatrixXf& data, const bool& debug = false);
  bool groundPlaneHasDataBelow(const Eigen::MatrixXf& ground_plane, const Eigen::MatrixXf& data, const bool& debug =
                                   false);

protected:
  void getDifferenceAndNotNanCount(const Eigen::MatrixXf& ground_plane, const Eigen::MatrixXf& data,
                                   Eigen::MatrixXf& difference, int& not_nan_count);
  bool checkTooLow(const Eigen::MatrixXf& difference, const int& not_nan_count, double& too_low_ratio);

  mutable std::mutex mutex_;
  CameraModel camera_model_;
  CalibrationParametersPtr parameters_;
  Config config_;
  VisualizerInterfacePtr depth_visualizer_;

  Eigen::MatrixXf last_ground_plane_;
};
typedef std::shared_ptr<CalibrationValidation> CalibrationValidationPtr;

} /* end namespace */

#endif
