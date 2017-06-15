#ifndef plane_calibration_SRC_PLANE_CALIBRATION_HPP_
#define plane_calibration_SRC_PLANE_CALIBRATION_HPP_

#include <mutex>
#include <vector>
#include <utility>
#include <memory>
#include <Eigen/Dense>

#include "camera_model.hpp"
#include "calibration_parameters.hpp"
#include "deviation_planes.hpp"

namespace plane_calibration
{

class PlaneCalibration
{
public:
  PlaneCalibration(const CameraModel& camera_model, const CalibrationParametersPtr& parameters);

  virtual ~PlaneCalibration()
  {
  }

  std::pair<double, double> calibrate(const Eigen::MatrixXf& filtered_depth_matrix);

protected:
  class Errors
  {
  public:
    double mean;
    double max;
  };

  Errors getErrorsToBestEstimation(const Eigen::MatrixXf& matrix);
  Errors getErrors(const Eigen::MatrixXf& plane, const Eigen::MatrixXf& matrix);

  mutable std::mutex mutex_;
  CameraModel camera_model_;
  CalibrationParametersPtr parameters_;

  PlaneToDepthImage plane_to_depth_;
  DeviationPlanesPtr max_deviation_planes_;
  DeviationPlanesPtr deviation_planes_;

  DeviationPlanesPtr temp_deviation_planes_;
  Eigen::MatrixXf temp_estimated_plane_;

  std::pair<double, double> best_estimated_angles_;
  Eigen::MatrixXf best_estimated_plane_;
};
typedef std::shared_ptr<PlaneCalibration> PlaneCalibrationPtr;

} /* end namespace */

#endif
