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
#include "depth_visualizer.hpp"

namespace plane_calibration
{

class PlaneCalibration
{
public:
  PlaneCalibration(const CameraModel& camera_model, const CalibrationParametersPtr& parameters,
                   const std::shared_ptr<DepthVisualizer>& depth_visualizer);

  virtual ~PlaneCalibration()
  {
  }

  std::pair<double, double> calibrate(const Eigen::MatrixXf& filtered_depth_matrix, int iterations = 3);

protected:

  mutable std::mutex mutex_;
  CameraModel camera_model_;
  CalibrationParametersPtr parameters_;

  PlaneToDepthImage plane_to_depth_;
  DeviationPlanesPtr max_deviation_planes_;

  DeviationPlanesPtr temp_deviation_planes_;
  Eigen::MatrixXf temp_estimated_plane_;

  std::shared_ptr<DepthVisualizer> depth_visualizer_;
};
typedef std::shared_ptr<PlaneCalibration> PlaneCalibrationPtr;

} /* end namespace */

#endif
