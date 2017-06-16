#ifndef plane_calibration_SRC_PLANE_CALIBRATION_NODELET_HPP_
#define plane_calibration_SRC_PLANE_CALIBRATION_NODELET_HPP_

#include <atomic>
#include <mutex>
#include <memory>
#include <Eigen/Dense>

#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <plane_calibration/PlaneCalibrationConfig.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>

#include "camera_model.hpp"
#include "calibration_parameters.hpp"
#include "input_filter.hpp"
#include "plane_calibration.hpp"
#include "calibration_validation.hpp"
#include "plane_to_depth_image.hpp"
#include "depth_visualizer.hpp"

namespace plane_calibration
{

class PlaneCalibrationNodelet : public nodelet::Nodelet
{
public:
  PlaneCalibrationNodelet();

  virtual void onInit();

protected:
  virtual void reconfigureCB(PlaneCalibrationConfig &config, uint32_t level);
  virtual void cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);
  virtual void depthImageCB(const sensor_msgs::ImageConstPtr& depth_image_msg);

  virtual void runCalibration(Eigen::MatrixXf depth_matrix);
  virtual void publishTransform();

  CameraModelPtr camera_model_;
  CalibrationParametersPtr calibration_parameters_;
  InputFilterPtr input_filter_;
  PlaneCalibrationPtr plane_calibration_;
  CalibrationValidationPtr calibration_validation_;
  PlaneToDepthImagePtr plane_to_depth_converter_;

  std::atomic<double> max_deviation_;
  Eigen::Vector3d ground_plane_offset_;
  Eigen::AngleAxisd ground_plane_rotation_;
  std::atomic<int> iterations_;

  std::pair<double, double> last_valid_calibration_result_;
  Eigen::MatrixXf last_valid_calibration_result_plane_;
  Eigen::Affine3d last_valid_calibration_transformation_;

  InputFilter::Config input_filter_config_;
  CalibrationValidation::Config calibration_validation_config_;

  std::shared_ptr<DepthVisualizer> depth_visualizer_;
  std::shared_ptr<dynamic_reconfigure::Server<PlaneCalibrationConfig>> reconfigure_server_;

  std::atomic<bool> debug_;
  std::atomic<double> x_offset_;
  std::atomic<double> y_offset_;
  std::atomic<double> z_offset_;

  std::atomic<double> px_offset_;
  std::atomic<double> py_offset_;
  std::atomic<double> pz_offset_;

  ros::Publisher pub_candidate_points_;
  ros::Publisher pub_plane_points_;
  ros::Publisher pub_transform_;
  ros::Subscriber sub_camera_info_;
  ros::Subscriber sub_depth_image_;
};

} /* end namespace */

#endif
