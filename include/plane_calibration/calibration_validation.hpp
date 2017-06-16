#ifndef plane_calibration_SRC_CALIBRATION_VALIDATION_HPP_
#define plane_calibration_SRC_CALIBRATION_VALIDATION_HPP_

#include <memory>
#include <utility>
#include <Eigen/Dense>

#include "camera_model.hpp"
#include "calibration_parameters.hpp"

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
      max_too_low_ratio = 0.0;
      max_mean = 0.0;
      max_deviation = 0.0;
    }

    double max_too_low_ratio;
    double max_mean;
    double max_deviation;
  };

  CalibrationValidation(const CameraModel& camera_model, const CalibrationParametersPtr& parameters, const Config& config);

  void updateConfig(const Config& new_config);

  bool angleOffsetValid(std::pair<double, double> angles);
  bool groundPlaneFitsData(const Eigen::MatrixXf& ground_plane, const Eigen::MatrixXf& data);

protected:
  mutable std::mutex mutex_;
  CameraModel camera_model_;
  CalibrationParametersPtr parameters_;
  Config config_;

  Eigen::MatrixXf last_ground_plane_;
};
typedef std::shared_ptr<CalibrationValidation> CalibrationValidationPtr;

} /* end namespace */

#endif
