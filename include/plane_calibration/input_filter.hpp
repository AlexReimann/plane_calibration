#ifndef plane_calibration_SRC_INPUT_FILTER_HPP_
#define plane_calibration_SRC_INPUT_FILTER_HPP_

#include <memory>
#include <mutex>
#include <Eigen/Dense>

#include "camera_model.hpp"
#include "calibration_parameters.hpp"
#include "depth_visualizer.hpp"

namespace plane_calibration
{

class InputFilter
{
public:
  class Config
  {
  public:
    Config()
    {
      max_error = 0.0;
      threshold_from_ground = 0.0;

      max_nan_ratio = 0.0;
      max_zero_ratio = 0.0;
      min_data_ratio = 0.0;
    }

    double max_error;
    double threshold_from_ground;

    double max_nan_ratio;
    double max_zero_ratio;
    double min_data_ratio;
  };

  //TODO automatic threshold from max_deviation
  InputFilter(const CameraModel& camera_model, const CalibrationParametersPtr& parameters,
              const std::shared_ptr<DepthVisualizer>& depth_visualizer, const Config& config);

  void updateConfig(const Config& config);
  void filter(Eigen::MatrixXf& matrix, bool debug = false);
  bool dataIsUsable(const Eigen::MatrixXf& data);

protected:
  void updateBorders_();

  mutable std::mutex mutex_;
  CameraModel camera_model_;
  CalibrationParametersPtr parameters_;
  Config config_;

  Eigen::MatrixXf min_plane_;
  Eigen::MatrixXf max_plane_;

  std::shared_ptr<DepthVisualizer> depth_visualizer_;
};
typedef std::shared_ptr<InputFilter> InputFilterPtr;

} /* end namespace */

#endif
