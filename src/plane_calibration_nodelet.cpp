#include "plane_calibration/plane_calibration_nodelet.hpp"

#include <sstream>
#include <Eigen/Dense>
#include <ecl/geometry/angle.hpp>

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <tf2_msgs/TFMessage.h>
#include <image_geometry/pinhole_camera_model.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>

#include "plane_calibration/image_msg_eigen_converter.hpp"

namespace plane_calibration
{

PlaneCalibrationNodelet::PlaneCalibrationNodelet() :
    transform_listener_buffer_(), transform_listener_(transform_listener_buffer_)
{
  debug_ = false;
  ground_plane_rotation_ = Eigen::AngleAxisd::Identity();

  last_valid_calibration_result_ = std::make_pair(0.0, 0.0);
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

  max_deviation_ = ecl::degrees_to_radians(config.max_deviation_degrees);
  iterations_ = config.iterations;

  input_filter_config_.max_nan_ratio = config.input_max_nan_ratio;
  input_filter_config_.max_zero_ratio = config.input_max_zero_ratio;
  input_filter_config_.min_data_ratio = config.input_min_data_ratio;
  input_filter_config_.max_error = config.input_max_noise;
  input_filter_config_.threshold_from_ground = config.input_threshold_from_ground;

  if (input_filter_)
  {
    input_filter_->updateConfig(input_filter_config_);
  }

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

  calibration_validation_config_.max_too_low_ratio = config.plane_max_too_low_ratio;
  calibration_validation_config_.max_mean = config.plane_max_mean;
  calibration_validation_config_.max_deviation = config.plane_max_deviation;

  if (calibration_validation_)
  {
    calibration_validation_->updateConfig(calibration_validation_config_);
  }
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
    input_filter_ = std::make_shared<InputFilter>(*camera_model_, calibration_parameters_, depth_visualizer_,
                                                  input_filter_config_);
  }

  if (!calibration_validation_)
  {
    calibration_validation_ = std::make_shared<CalibrationValidation>(*camera_model_, calibration_parameters_,
                                                                      calibration_validation_config_);
  }

  if (!plane_to_depth_converter_)
  {
    plane_to_depth_converter_ = std::make_shared<PlaneToDepthImage>(camera_model_->getParameters());
  }

  Eigen::MatrixXf depth_matrix;

  bool converted_successfully = ImageMsgEigenConverter::convert(depth_image_msg, depth_matrix);
  if (!converted_successfully)
  {
    ROS_ERROR_STREAM("[PlaneCalibrationNodelet]: Conversion from image msg to Eigen matrix failed");
    return;
  }

  getTransform();

  if (!transform_)
  {
    return;
  }

  runCalibration(depth_matrix);
  publishTransform();
}

void PlaneCalibrationNodelet::getTransform()
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = transform_listener_buffer_.lookupTransform("base_footprint",
                                                                  "sensor_3d_short_range_depth_optical_frame",
                                                                  ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  if (!transform_)
  {
    ROS_INFO_STREAM("[PlaneCalibrationNodelet]: Got transform to ground, will do calibration from now");
  }

  transform_ = std::make_shared<geometry_msgs::TransformStamped>(transformStamped);
}

void PlaneCalibrationNodelet::runCalibration(Eigen::MatrixXf depth_matrix)
{
  if (debug_)
  {
    static tf2_ros::TransformBroadcaster transform_broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    std::string frame_id = "camera_depth_optical_frame";
    transformStamped.header.frame_id = frame_id;

    CalibrationParameters::Parameters parameters = calibration_parameters_->getParameters();

    transformStamped.child_frame_id = "uncalibrated_ground";
    tf::transformEigenToMsg(parameters.getTransform(), transformStamped.transform);
    transform_broadcaster.sendTransform(transformStamped);

    depth_visualizer_->publishCloud("debug/uncalibrated_ground", parameters.getTransform(),
                                    camera_model_->getParameters(), frame_id);
  }

  input_filter_->filter(depth_matrix, debug_);

  bool input_data_not_usable = !input_filter_->dataIsUsable(depth_matrix, debug_);
  if (input_data_not_usable)
  {
    if (debug_)
    {
      ROS_WARN_STREAM("[PlaneCalibrationNodelet]: Input data not usable, not going to calibrate");
    }
    return;
  }

  if (last_valid_calibration_result_plane_.size() != 0)
  {
    bool parameters_updated = calibration_parameters_->parametersUpdated();
    bool last_calibration_is_still_good = calibration_validation_->groundPlaneFitsData(
        last_valid_calibration_result_plane_, depth_matrix);
    if (!parameters_updated && last_calibration_is_still_good)
    {
      if (debug_)
      {
        ROS_INFO_STREAM("[PlaneCalibrationNodelet]: Last calibration data still works, not going to calibrate");
      }
      return;
    }
  }

  CalibrationParameters::Parameters parameters = calibration_parameters_->getParameters();
  std::pair<double, double> calibration_result = plane_calibration_->calibrate(depth_matrix, iterations_);

  if (debug_)
  {
    ROS_INFO_STREAM(
        "[PlaneCalibrationNodelet]: Calibration result angles [degree]: " << ecl::radians_to_degrees(calibration_result.first) << ", " << calibration_result.second);

    Eigen::AngleAxisd rotation;
    rotation = parameters.rotation_ * Eigen::AngleAxisd(calibration_result.first, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(calibration_result.second, Eigen::Vector3d::UnitY());

    Eigen::Affine3d transform = Eigen::Translation3d(parameters.ground_plane_offset_) * rotation;

    std::string frame_id = "camera_depth_optical_frame";
    depth_visualizer_->publishCloud("debug/calibration_result", transform, camera_model_->getParameters(), frame_id);
  }

  //  std::cout << "offset angles: " << ecl::radians_to_degrees(one_shot_result.first) << ", "
  //      << ecl::radians_to_degrees(one_shot_result.second) << std::endl;
  //  std::cout << "original angles: " << ecl::radians_to_degrees(px_offset_.load()) << ", "
  //      << ecl::radians_to_degrees(py_offset_.load()) << std::endl;

  bool valid_calibration_angles = calibration_validation_->angleOffsetValid(calibration_result);
  if (!valid_calibration_angles)
  {
    if (debug_)
    {
      ROS_WARN_STREAM(
          "[PlaneCalibrationNodelet]: Calibration turned out bad, too big angles ( > " << ecl::radians_to_degrees(parameters.max_deviation_) << " [degree]):");
      ROS_WARN_STREAM(
          "[PlaneCalibrationNodelet]: x,y angles [degree]: " << ecl::radians_to_degrees(calibration_result.first) << ", " << ecl::radians_to_degrees(calibration_result.second));
    }
    //keep old / last one
    return;
  }

  Eigen::AngleAxisd rotation;
  rotation = parameters.rotation_ * Eigen::AngleAxisd(calibration_result.first, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(calibration_result.second, Eigen::Vector3d::UnitY());

  Eigen::Affine3d transform = Eigen::Translation3d(parameters.ground_plane_offset_) * rotation;
  Eigen::MatrixXf new_ground_plane = plane_to_depth_converter_->convert(transform);

  bool good_calibration = calibration_validation_->groundPlaneFitsData(new_ground_plane, depth_matrix, debug_);
  if (!good_calibration)
  {
    if (debug_)
    {
      ROS_WARN_STREAM("[PlaneCalibrationNodelet]: Calibration turned out bad: Data does not fit good enough");
    }
    //keep old / last one
    return;
  }

  std::stringstream angle_change_string;
  angle_change_string << "px [degree]: " << ecl::radians_to_degrees(last_valid_calibration_result_.first) << " -> "
      << ecl::radians_to_degrees(calibration_result.first);
  angle_change_string << ", ";
  angle_change_string << "py [degree]: " << ecl::radians_to_degrees(last_valid_calibration_result_.second) << " -> "
      << ecl::radians_to_degrees(calibration_result.second);

  ROS_INFO_STREAM("[PlaneCalibrationNodelet]: Updated the calibration angles: " << angle_change_string.str());

  last_valid_calibration_result_ = calibration_result;
  last_valid_calibration_result_plane_ = new_ground_plane;
  last_valid_calibration_transformation_ = transform;
}

void PlaneCalibrationNodelet::publishTransform()
{
  static tf2_ros::TransformBroadcaster transform_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  std::string frame_id = "camera_depth_optical_frame";
  transformStamped.header.frame_id = frame_id;

  transformStamped.child_frame_id = "calibrated_plane";
  tf::transformEigenToMsg(last_valid_calibration_transformation_, transformStamped.transform);
  transform_broadcaster.sendTransform(transformStamped);

  if (debug_)
  {
    depth_visualizer_->publishCloud("debug/calibrated_plane", last_valid_calibration_transformation_,
                                    camera_model_->getParameters(), frame_id);
  }
}

} /* end namespace */

PLUGINLIB_EXPORT_CLASS(plane_calibration::PlaneCalibrationNodelet, nodelet::Nodelet)
