#include "plane_calibration/plane_calibration_nodelet.hpp"

#include <Eigen/Dense>
#include <ecl/geometry/angle.hpp>

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <tf2_msgs/TFMessage.h>
#include <image_geometry/pinhole_camera_model.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>

#include "plane_calibration/image_msg_eigen_converter.hpp"
#include "plane_calibration/plane_to_depth_image.hpp"

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

  ros::NodeHandle node_handle = this->getPrivateNodeHandle();
  depth_visualizer_ = std::make_shared<DepthVisualizer>(node_handle);

  pub_candidate_points_ = node_handle.advertise<sensor_msgs::Image>("candidates", 1);
  pub_plane_points_ = node_handle.advertise<sensor_msgs::Image>("plane_points", 1);
  pub_transform_ = node_handle.advertise<tf2_msgs::TFMessage>("adjusted_tf", 1);

  sub_camera_info_ = node_handle.subscribe<sensor_msgs::CameraInfo>("camera_info", 1,
                                                                    &PlaneCalibrationNodelet::cameraInfoCB, this);
  sub_depth_image_ = node_handle.subscribe<sensor_msgs::Image>("input_depth_image", 1,
                                                               &PlaneCalibrationNodelet::depthImageCB, this);

  ROS_INFO("[PlaneCalibrationNodelet]: Initialization finished");
}

void PlaneCalibrationNodelet::reconfigureCB(PlaneCalibrationConfig &config, uint32_t level)
{
  if (debug_ != config.debug)
  {
    debug_ = config.debug;
    std::string debug_status = debug_ ? "enabled" : "disabled";
    ROS_INFO_STREAM("[PlaneCalibrationNodelet]: Debug " << debug_status);
  }

  x_offset_ = config.x;
  y_offset_ = config.y;
  z_offset_ = config.z;

  px_offset_ = config.px;
  py_offset_ = config.py;
  pz_offset_ = config.pz;

  Eigen::Vector3d ground_plane_offset = ground_plane_offset_;
  if (config.use_manual_ground_transform)
  {
    ground_plane_offset = Eigen::Vector3d(x_offset_, y_offset_, z_offset_);
  }

  PlaneCalibration::Parameters parameters;
  parameters.ground_plane_offset_ = ground_plane_offset;
  parameters.max_deviation_ = ecl::degrees_to_radians(config.max_deviation_degrees);
  plane_calibration_.updateParameters(parameters);
}

void PlaneCalibrationNodelet::cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
{
  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info_msg);

  depth_visualizer_->setCameraModel(pinhole_camera_model);

  CameraModel camera_model(pinhole_camera_model.cx(), pinhole_camera_model.cy(), pinhole_camera_model.fx(),
                           pinhole_camera_model.fy(), camera_info_msg->width, camera_info_msg->height);
  plane_calibration_.updateParameters(camera_model);
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

  plane_calibration_.updateMaxDeviationPlanesIfNeeded();

  if (debug_)
  {
    publishMaxDeviationPlanes();
  }
}

void PlaneCalibrationNodelet::publishMaxDeviationPlanes()
{
  static tf2_ros::TransformBroadcaster transform_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  std::string frame_id = "camera_link";
  transformStamped.header.frame_id = frame_id;

  PlaneCalibration::PlanesWithTransforms planes = plane_calibration_.getDeviationPlanes();

  for (int i = 0; i < planes.size(); ++i)
  {
    Eigen::MatrixXf plane_image_matrix = planes[i].plane;
    Eigen::Affine3d transform = planes[i].transform;
    std::string index_string = std::to_string(i);

    depth_visualizer_->publishImage("/debug/images/max_deviation_" + index_string, plane_image_matrix, frame_id);
    depth_visualizer_->publishCloud("/debug/clouds/max_deviation_" + index_string, plane_image_matrix, frame_id);

    transformStamped.child_frame_id = "max_deviation_plane_" + index_string;
    tf::transformEigenToMsg(transform, transformStamped.transform);
    transform_broadcaster.sendTransform(transformStamped);
  }
}

} /* end namespace */

PLUGINLIB_EXPORT_CLASS(plane_calibration::PlaneCalibrationNodelet, nodelet::Nodelet)
