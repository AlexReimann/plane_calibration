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
#include "plane_calibration.hpp"
#include "magic_multiplier_estimation.hpp"
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

  virtual void publishMaxDeviationPlanes();

  void test();

  CameraModel camera_model_;
  PlaneCalibrationPtr plane_calibration_;
  MagicMultiplierEstimationPtr magic_estimator_;

  std::atomic<double> max_deviation_;
  Eigen::Vector3d ground_plane_offset_;

  std::shared_ptr<DepthVisualizer> depth_visualizer_;
  std::shared_ptr<dynamic_reconfigure::Server<PlaneCalibrationConfig>> reconfigure_server_;

  std::atomic<bool> debug_;
  std::atomic<double> x_offset_;
  std::atomic<double> y_offset_;
  std::atomic<double> z_offset_;

  std::atomic<double> px_offset_;
  std::atomic<double> py_offset_;
  std::atomic<double> pz_offset_;
  std::atomic<double> max_noise_;

  ros::Publisher pub_candidate_points_;
  ros::Publisher pub_plane_points_;
  ros::Publisher pub_transform_;
  ros::Subscriber sub_camera_info_;
  ros::Subscriber sub_depth_image_;
};

} /* end namespace */

#endif
