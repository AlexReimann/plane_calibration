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
  update_max_deviation_planes_ = true;
  use_manual_ground_transform_ = false;
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
    ROS_INFO_STREAM("[PlaneCalibrationNodelet]: Debug " << debug_ ? "enabled" : "disabled");
  }

  x_offset_ = config.x;
  y_offset_ = config.y;
  z_offset_ = config.z;

  px_offset_ = config.px;
  py_offset_ = config.py;
  pz_offset_ = config.pz;

  max_deviation_ = ecl::degrees_to_radians(config.max_deviation_degrees);

  std::lock_guard<std::mutex> lock(ground_offset_mutex_);
  if (config.use_manual_ground_transform)
  {
    ground_plane_offset_ = Eigen::Vector3d(x_offset_, y_offset_, z_offset_);
    update_max_deviation_planes_ = true;
  }

  bool switched_off_manual = !config.use_manual_ground_transform
      && use_manual_ground_transform_ != config.use_manual_ground_transform;
  if (switched_off_manual)
  {
    ///TODO switch ground_plane_offset_ back to normal
    update_max_deviation_planes_ = true;
  }

  use_manual_ground_transform_ = config.use_manual_ground_transform;
}

void PlaneCalibrationNodelet::cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
{
  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info_msg);

  if (!camera_model_.initialized())
  {
    ROS_INFO("[PlaneCalibrationNodelet]: Got camera info");
  }
  camera_model_.update(camera_model.cx(), camera_model.cy(), camera_model.fx(), camera_model.fy(),
                       camera_info_msg->width, camera_info_msg->height);

  depth_visualizer_->setCameraModel(camera_model);
}

void PlaneCalibrationNodelet::depthImageCB(const sensor_msgs::ImageConstPtr& depth_image_msg)
{
  if (!camera_model_.initialized())
  {
    return;
  }

  Eigen::MatrixXf depth_matrix;

  bool converted_successfully = ImageMsgEigenConverter::convert(depth_image_msg, depth_matrix);
  if (!converted_successfully)
  {
    ROS_ERROR_STREAM("[PlaneCalibrationNodelet]: Conversion from image msg to Eigen matrix failed");
    return;
  }

  if (update_max_deviation_planes_)
  {
    updateMaxDeviationPlanesImages();
  }

  if (debug_)
  {
    publishMaxDeviationPlanes();
  }
}

void PlaneCalibrationNodelet::updateMaxDeviationPlanesImages()
{
  max_deviation_planes_images_.clear();
  std::vector<Eigen::Affine3d> transforms = getMaxDeviationTransforms();

  for (int i = 0; i < transforms.size(); ++i)
  {
    Eigen::Affine3d transform = transforms[i];
    max_deviation_planes_images_.push_back(getTiltedPlaneImage(transform));
  }

  update_max_deviation_planes_ = false;
}

std::vector<Eigen::Affine3d> PlaneCalibrationNodelet::getMaxDeviationTransforms()
{
  double max_deviation;
  Eigen::Translation3d translation;
  {
    std::lock_guard<std::mutex> lock(ground_offset_mutex_);
    max_deviation = max_deviation_;
    translation = Eigen::Translation3d(ground_plane_offset_);
  }

  Eigen::AngleAxisd x_tilt_positive(max_deviation, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd y_tilt_positive(max_deviation, Eigen::Vector3d::UnitY());

  Eigen::AngleAxisd x_tilt_negative(-max_deviation, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd y_tilt_negative(-max_deviation, Eigen::Vector3d::UnitY());

  std::vector<Eigen::Affine3d> tilt_transforms;
  tilt_transforms.push_back(translation * x_tilt_positive);
  tilt_transforms.push_back(translation * y_tilt_positive);

  tilt_transforms.push_back(translation * x_tilt_negative);
  tilt_transforms.push_back(translation * y_tilt_negative);

  return tilt_transforms;
}

Eigen::MatrixXf PlaneCalibrationNodelet::getTiltedPlaneImage(Eigen::Affine3d tilt_transform)
{
  return PlaneToDepthImage::convert(tilt_transform, camera_model_.getValues());
}

void PlaneCalibrationNodelet::publishMaxDeviationPlanes()
{
  static tf2_ros::TransformBroadcaster transform_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  std::string frame_id = "camera_link";
  transformStamped.header.frame_id = frame_id;

  std::vector<Eigen::Affine3d> transforms = getMaxDeviationTransforms();

  for (int i = 0; i < max_deviation_planes_images_.size(); ++i)
  {
    Eigen::MatrixXf plane_image_matrix = max_deviation_planes_images_[i];
    Eigen::Affine3d transform = transforms[i];
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
