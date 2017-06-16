#include "plane_calibration/input_filter.hpp"

#include <iostream>
#include "plane_calibration/plane_to_depth_image.hpp"

namespace plane_calibration
{

InputFilter::InputFilter(const CameraModel& camera_model, const CalibrationParametersPtr& parameters,
                         const std::shared_ptr<DepthVisualizer>& depth_visualizer, double max_error,
                         double threshold_from_ground) :
    camera_model_(camera_model)
{
  parameters_ = parameters;
  max_error_ = max_error;
  threshold_from_ground_ = threshold_from_ground;
  depth_visualizer_ = depth_visualizer;

  updateBorders_();
}

void InputFilter::update(double max_error, double threshold_from_ground)
{
  std::lock_guard<std::mutex> lock(mutex_);
  max_error_ = max_error;
  threshold_from_ground_ = threshold_from_ground;

  updateBorders_();
}

void InputFilter::updateBorders_()
{
  double threshold = threshold_from_ground_ + max_error_;

  Eigen::Affine3d ground_transform = parameters_->getTransform();
  Eigen::Translation3d top_offset(threshold * Eigen::Vector3d::UnitZ());
  Eigen::Translation3d bottom_offset(-threshold * Eigen::Vector3d::UnitZ());

  Eigen::Affine3d top_transform = ground_transform * top_offset;
  Eigen::Affine3d bottom_transform = ground_transform * bottom_offset;

  min_plane_ = PlaneToDepthImage::convert(top_transform, camera_model_.getParameters());
  max_plane_ = PlaneToDepthImage::convert(bottom_transform, camera_model_.getParameters());

}

void InputFilter::filter(Eigen::MatrixXf& matrix, bool debug)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> far_enough = matrix.array() >= min_plane_.array();
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> close_enough = matrix.array() <= max_plane_.array();

  Eigen::MatrixXf valid = (far_enough.array() && close_enough.array()).cast<float>();
  matrix = matrix.cwiseProduct(valid);

  if (debug)
  {
    std::string frame_id = "camera_depth_optical_frame";
    depth_visualizer_->publishCloud("debug/filter/top_border", max_plane_, frame_id);
    depth_visualizer_->publishCloud("debug/filter/bottom_border", min_plane_, frame_id);
    depth_visualizer_->publishImage("debug/filter/min_plane", max_plane_, frame_id);
    depth_visualizer_->publishImage("debug/filter/max_plane", max_plane_, frame_id);

    depth_visualizer_->publishImage("debug/filter/far_enough", far_enough.cast<float>(), frame_id);
    depth_visualizer_->publishImage("debug/filter/close_enough", close_enough.cast<float>(), frame_id);

    depth_visualizer_->publishImage("debug/filter/diff_min", min_plane_ - matrix, frame_id);
    depth_visualizer_->publishImage("debug/filter/diff_max", max_plane_ - matrix, frame_id);
    depth_visualizer_->publishImage("debug/filter/not_filtered", valid, frame_id);
    depth_visualizer_->publishCloud("debug/filter/filtered", matrix, frame_id);
  }
}

} /* end namespace */
