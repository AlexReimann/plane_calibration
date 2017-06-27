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
#include "planes.hpp"
#include "visualizer_interface.hpp"

namespace plane_calibration
{

class PlaneCalibration
{
public:
  PlaneCalibration(const CameraModel& camera_model, const CalibrationParametersPtr& parameters,
                   const VisualizerInterfacePtr& depth_visualizer);

  virtual ~PlaneCalibration()
  {
  }

  std::pair<double, double> calibrate(const Eigen::MatrixXf& filtered_depth_matrix, const int& iterations = 3);

protected:
  std::pair<double, double> estimateAngles(const Eigen::MatrixXf& filtered_depth_matrix,
                                           const std::pair<double, double>& last_estimation, const double& deviation);
  std::pair<double, double> estimateAngles(const Eigen::MatrixXf& filtered_depth_matrix, const double& x_angle_offset,
                                           const double& y_angle_offset, const double& deviation);

  mutable std::mutex mutex_;
  CameraModel camera_model_;
  CalibrationParametersPtr parameters_;

  PlaneToDepthImage plane_to_depth_;
  DeviationPlanesPtr max_deviation_planes_;

  bool precompute_planes_;
  int precomputed_plane_pairs_count_;
  PlanesPtr precomputed_planes_;

  std::vector<DeviationPlanesPtr> deviation_planes_;

  CalibrationParameters::Parameters temp_parameters_;
  DeviationPlanesPtr temp_deviation_planes_;
  Eigen::MatrixXf temp_estimated_plane_;

  VisualizerInterfacePtr depth_visualizer_;
};
typedef std::shared_ptr<PlaneCalibration> PlaneCalibrationPtr;

} /* end namespace */

#endif
