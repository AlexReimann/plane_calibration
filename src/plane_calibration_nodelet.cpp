#include "plane_calibration/plane_calibration_nodelet.hpp"

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <tf2_msgs/TFMessage.h>

#include <Eigen/Dense>

#include "plane_calibration/image_msg_eigen_converter.hpp"

namespace plane_calibration
{

PlaneCalibrationNodelet::PlaneCalibrationNodelet()
{
  debug_ = false;
}

void PlaneCalibrationNodelet::onInit()
{
  ROS_INFO("[PlaneCalibrationNodelet]: Initializing");

  reconfigure_server_ = std::shared_ptr<dynamic_reconfigure::Server<PlaneCalibrationConfig> >(
      new dynamic_reconfigure::Server<PlaneCalibrationConfig>());
  dynamic_reconfigure::Server<PlaneCalibrationConfig>::CallbackType reconfigure_cb = boost::bind(
      &PlaneCalibrationNodelet::reconfigureCB, this, _1, _2);
  reconfigure_server_->setCallback(reconfigure_cb);

  ros::NodeHandle node_handle;

  pub_candidate_points_ = node_handle.advertise<sensor_msgs::Image>("candidates", 1);
  pub_plane_points_ = node_handle.advertise<sensor_msgs::Image>("plane_points", 1);
  pub_transform_ = node_handle.advertise<tf2_msgs::TFMessage>("adjusted_tf", 1);

  sub_depth_image_ = node_handle.subscribe<sensor_msgs::Image>("input_depth_image", 1,
                                                               &PlaneCalibrationNodelet::depthImageCB, this);

  ROS_INFO("[PlaneCalibrationNodelet]: Initialization finished");
}

void PlaneCalibrationNodelet::reconfigureCB(PlaneCalibrationConfig &config, uint32_t level)
{
  if (debug_ != config.debug)
  {
    debug_ = config.debug;
    ROS_INFO_STREAM("[PlaneCalibrationNodelet]: Debug " << debug_ ? "enabled" : "disabled");
  }
}

void PlaneCalibrationNodelet::depthImageCB(const sensor_msgs::ImageConstPtr& depth_image_msg)
{
  Eigen::MatrixXf depth_matrix;

  bool converted_successfully = ImageMsgEigenConverter::convert(depth_image_msg, depth_matrix);
  if (!converted_successfully)
  {
    ROS_ERROR_STREAM("[PlaneCalibrationNodelet]: Conversion from image msg to Eigen matrix failed");
    return;
  }
}

} /* end namespace */

PLUGINLIB_EXPORT_CLASS(plane_calibration::PlaneCalibrationNodelet, nodelet::Nodelet)
