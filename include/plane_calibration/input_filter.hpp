#ifndef plane_calibration_SRC_INPUT_FILTER_HPP_
#define plane_calibration_SRC_INPUT_FILTER_HPP_

#include <memory>
#include <Eigen/Dense>

#include "camera_model.hpp"
#include "calibration_parameters.hpp"
#include "depth_visualizer.hpp"

namespace plane_calibration
{

class InputFilter
{
public:
  //TODO automatic threshold from max_deviation
  InputFilter(const CameraModel& camera_model, const CalibrationParametersPtr& parameters,
              const std::shared_ptr<DepthVisualizer>& depth_visualizer, double threshold_from_ground = 0.2,
              double max_error = 0.03);

  void filter(Eigen::MatrixXf& matrix, bool debug = false);

protected:
  mutable std::mutex mutex_;
  CameraModel camera_model_;
  CalibrationParametersPtr parameters_;
  double threshold_from_ground_;
  double max_error_;

  Eigen::MatrixXf min_plane_;
  Eigen::MatrixXf max_plane_;

  std::shared_ptr<DepthVisualizer> depth_visualizer_;
};
typedef std::shared_ptr<InputFilter> InputFilterPtr;

} /* end namespace */

#endif
