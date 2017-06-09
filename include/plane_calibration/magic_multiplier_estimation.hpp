#ifndef plane_calibration_SRC_MAGIC_MULTIPLIER_ESTIMATION_HPP_
#define plane_calibration_SRC_MAGIC_MULTIPLIER_ESTIMATION_HPP_

#include <utility>
#include <memory>

#include "camera_model.hpp"
#include "plane_calibration.hpp"

namespace plane_calibration
{

class MagicMultiplierEstimation
{
public:
  class Result
  {
  public:
    Result()
    {
      max_error_x_degree = 0.0;
      max_error_y_degree = 0.0;
      max_cross_error_x_degree = 0.0;
      max_cross_error_y_degree = 0.0;

      multiplier_x = 0.0;
      multiplier_y = 0.0;
    }

    double max_error_x_degree;
    double max_error_y_degree;
    double max_cross_error_x_degree;
    double max_cross_error_y_degree;

    double multiplier_x;
    double multiplier_y;
  };

  MagicMultiplierEstimation(CameraModel camera_model, PlaneCalibrationPtr plane_calibration, double z_offset = 2.0,
                            double max_x_angle = 0.2, double max_y_angle = 0.2, double step_size = 0.001);

  Result estimate(bool calculate_errors = false);

protected:
  std::pair<double, double> getXYMaxAbsErrors(double magic_x_multiplier, double magic_y_multiplier) const;
  std::pair<double, double> getMaxAbsCrossErrors(double magic_x_multiplier, double magic_y_multiplier) const;
  std::pair<double, double> getXYMaxAbsError(double magic_x_multiplier, double magic_y_multiplier, double x_step_size,
                                             double y_step_size) const;

  double getXDistanceDiff(double angle) const;
  double getYDistanceDiff(double angle) const;
  std::pair<double, double> getXYDistanceDiff(double x_angle, double y_angle) const;

  CameraModel camera_model_;
  PlaneCalibrationPtr plane_calibration_;

  double z_offset_;
  double max_x_angle_;
  double max_y_angle_;
  double step_size_;
};
typedef std::shared_ptr<MagicMultiplierEstimation> MagicMultiplierEstimationPtr;

} /* end namespace */

#endif
