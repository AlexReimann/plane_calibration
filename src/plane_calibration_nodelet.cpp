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
  max_noise_ = 0.0;
  ground_plane_rotation_ = Eigen::AngleAxisd::Identity();
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

  px_offset_ = ecl::degrees_to_radians(config.px_degree);
  py_offset_ = ecl::degrees_to_radians(config.py_degree);
  pz_offset_ = ecl::degrees_to_radians(config.pz_degree);
  max_noise_ = config.max_noise;

  max_deviation_ = ecl::degrees_to_radians(config.max_deviation_degrees);
  iterations_ = config.iterations;

  Eigen::Vector3d ground_plane_offset;
  Eigen::AngleAxisd rotation = ground_plane_rotation_;
  if (config.use_manual_ground_transform)
  {
    ground_plane_offset = Eigen::Vector3d(x_offset_, y_offset_, z_offset_);
    rotation = Eigen::AngleAxisd(px_offset_, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(py_offset_, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(pz_offset_, Eigen::Vector3d::UnitZ());
  }

  if (!calibration_parameters_)
  {
    calibration_parameters_ = std::make_shared<CalibrationParameters>();
  }

  calibration_parameters_->update(ground_plane_offset, ecl::degrees_to_radians(config.max_deviation_degrees), rotation);
}

void PlaneCalibrationNodelet::cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
{
  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info_msg);

  depth_visualizer_->setCameraModel(pinhole_camera_model);

  if (!camera_model_)
  {
    camera_model_ = std::make_shared<CameraModel>();
  }

  camera_model_->update(pinhole_camera_model.cx(), pinhole_camera_model.cy(), pinhole_camera_model.fx(),
                        pinhole_camera_model.fy(), camera_info_msg->width, camera_info_msg->height);
}

void PlaneCalibrationNodelet::depthImageCB(const sensor_msgs::ImageConstPtr& depth_image_msg)
{
  bool wait_for_initialization = !camera_model_ || !calibration_parameters_;
  if (wait_for_initialization)
  {
    return;
  }

  if (!plane_calibration_)
  {
    plane_calibration_ = std::make_shared<PlaneCalibration>(*camera_model_, calibration_parameters_, depth_visualizer_);
  }

  if (!input_filter_)
  {
    input_filter_ = std::make_shared<InputFilter>(*camera_model_, calibration_parameters_, depth_visualizer_);
  }

  Eigen::MatrixXf depth_matrix;

  bool converted_successfully = ImageMsgEigenConverter::convert(depth_image_msg, depth_matrix);
  if (!converted_successfully)
  {
    ROS_ERROR_STREAM("[PlaneCalibrationNodelet]: Conversion from image msg to Eigen matrix failed");
    return;
  }

  input_filter_->filter(depth_matrix, debug_);

  CalibrationParameters::Parameters parameters = calibration_parameters_->getParameters();
  std::pair<double, double> one_shot_result = plane_calibration_->calibrate(depth_matrix, iterations_);

  std::cout << "offset angles: " << ecl::radians_to_degrees(one_shot_result.first) << ", "
      << ecl::radians_to_degrees(one_shot_result.second) << std::endl;

  std::cout << "original angles: " << ecl::radians_to_degrees(px_offset_.load()) << ", "
      << ecl::radians_to_degrees(py_offset_.load()) << std::endl;

  static tf2_ros::TransformBroadcaster transform_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  std::string frame_id = "camera_depth_optical_frame";
  transformStamped.header.frame_id = frame_id;

  Eigen::AngleAxisd rotation;
  rotation = parameters.rotation_ * Eigen::AngleAxisd(one_shot_result.first, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(one_shot_result.second, Eigen::Vector3d::UnitY());

  Eigen::Affine3d transform = Eigen::Translation3d(parameters.ground_plane_offset_) * rotation;

  transformStamped.child_frame_id = "result";
  tf::transformEigenToMsg(transform, transformStamped.transform);
  transform_broadcaster.sendTransform(transformStamped);

  depth_visualizer_->publishCloud("result_plane", transform, camera_model_->getParameters(), frame_id);
  depth_visualizer_->publishCloud("start_plane", parameters.getTransform(), camera_model_->getParameters(), frame_id);

  if (debug_)
  {
    publishMaxDeviationPlanes(parameters.rotation_);
  }
}

void PlaneCalibrationNodelet::publishMaxDeviationPlanes(Eigen::AngleAxisd rotation)
{
  static tf2_ros::TransformBroadcaster transform_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  std::string frame_id = "camera_depth_optical_frame";
  transformStamped.header.frame_id = frame_id;

  Eigen::Affine3d transform = Eigen::Translation3d(Eigen::Vector3d(x_offset_, y_offset_, z_offset_)) * rotation;

  transformStamped.child_frame_id = "offset";
  tf::transformEigenToMsg(transform, transformStamped.transform);
  transform_broadcaster.sendTransform(transformStamped);
}

} /* end namespace */

PLUGINLIB_EXPORT_CLASS(plane_calibration::PlaneCalibrationNodelet, nodelet::Nodelet)
