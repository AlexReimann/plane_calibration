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
  plane_calibration_ = std::make_shared<PlaneCalibration>();
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

  Eigen::Vector3d ground_plane_offset = ground_plane_offset_;
  if (config.use_manual_ground_transform)
  {
    ground_plane_offset = Eigen::Vector3d(x_offset_, y_offset_, z_offset_);
  }

  PlaneCalibration::Parameters parameters;
  parameters.ground_plane_offset_ = ground_plane_offset;
  parameters.max_deviation_ = ecl::degrees_to_radians(config.max_deviation_degrees);
  plane_calibration_->updateParameters(parameters);
}

void PlaneCalibrationNodelet::cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
{
  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info_msg);

  depth_visualizer_->setCameraModel(pinhole_camera_model);

  camera_model_.update(pinhole_camera_model.cx(), pinhole_camera_model.cy(), pinhole_camera_model.fx(),
                       pinhole_camera_model.fy(), camera_info_msg->width, camera_info_msg->height);
  plane_calibration_->updateParameters(camera_model_);
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

  plane_calibration_->updateMaxDeviationPlanesIfNeeded();

  if (debug_)
  {
    publishMaxDeviationPlanes();
  }

  magic_estimator_ = std::make_shared<MagicMultiplierEstimation>(camera_model_, plane_calibration_, z_offset_,
                                                                 max_deviation_, max_deviation_);

//  testCalibration();
//  test();
}

void PlaneCalibrationNodelet::publishMaxDeviationPlanes()
{
  static tf2_ros::TransformBroadcaster transform_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  std::string frame_id = "camera_link";
  transformStamped.header.frame_id = frame_id;

  PlaneCalibration::PlanesWithTransforms planes = plane_calibration_->getDeviationPlanes();

  for (int i = 0; i < planes.size(); ++i)
  {
    Eigen::MatrixXf plane_image_matrix = planes[i].plane;
    Eigen::Affine3d transform = planes[i].transform;
    std::string index_string = std::to_string(i);

    depth_visualizer_->publishImage("debug/images/max_deviation_" + index_string, plane_image_matrix, frame_id);
    depth_visualizer_->publishCloud("debug/clouds/max_deviation_" + index_string, plane_image_matrix, frame_id);

    transformStamped.child_frame_id = "max_deviation_plane_" + index_string;
    tf::transformEigenToMsg(transform, transformStamped.transform);
    transform_broadcaster.sendTransform(transformStamped);
  }
}

void PlaneCalibrationNodelet::testCalibration()
{
  //construct plane we want to fit
  std::string frame_id = "camera_link";
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(px_offset_, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(py_offset_, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(pz_offset_, Eigen::Vector3d::UnitZ());

  Eigen::Affine3d transform = Eigen::Translation3d(Eigen::Vector3d(x_offset_, y_offset_, z_offset_)) * rotation;
  Eigen::MatrixXf plane_image_matrix = PlaneToDepthImage::convert(transform, camera_model_.getParameters());

  //get initial estimate
  bool calculate_errors = false;
  MagicMultiplierEstimation::Result magic_result = magic_estimator_->estimate(calculate_errors);

  int iterations = 10;
  double step_size = 0.5;
  Eigen::AngleAxisd estimation = plane_calibration_->estimateRotation(plane_image_matrix, magic_result.multiplier_x,
                                                                      magic_result.multiplier_y, iterations, step_size);

  std::pair<double, double> one_shot_result = plane_calibration_->estimateAngles(plane_image_matrix,
                                                                                 magic_result.multiplier_x,
                                                                                 magic_result.multiplier_y);

  static tf2_ros::TransformBroadcaster transform_broadcaster;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_id;

  transformStamped.child_frame_id = "test_plane";
  tf::transformEigenToMsg(transform, transformStamped.transform);
  transform_broadcaster.sendTransform(transformStamped);

  transformStamped.child_frame_id = "result";
  tf::transformEigenToMsg(Eigen::Translation3d(Eigen::Vector3d(x_offset_, y_offset_, z_offset_)) * estimation,
                          transformStamped.transform);
  transform_broadcaster.sendTransform(transformStamped);

  std::cout << "actual: " << std::endl << rotation << std::endl;
  std::cout << "estimated: " << std::endl << estimation.matrix() << std::endl;
}

void PlaneCalibrationNodelet::test()
{
  std::string frame_id = "camera_link";
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(px_offset_, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(py_offset_, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(pz_offset_, Eigen::Vector3d::UnitZ());

  Eigen::Affine3d transform = Eigen::Translation3d(Eigen::Vector3d(x_offset_, y_offset_, z_offset_)) * rotation;
  Eigen::MatrixXf plane_image_matrix = PlaneToDepthImage::convert(transform, camera_model_.getParameters());
  Eigen::MatrixXf noise = Eigen::MatrixXf::Random(plane_image_matrix.rows(), plane_image_matrix.cols());
  Eigen::MatrixXf random_plane_image = plane_image_matrix + max_noise_ * noise;

  depth_visualizer_->publishCloud("debug/test", random_plane_image, frame_id);

  static tf2_ros::TransformBroadcaster transform_broadcaster;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = "test_plane";
  tf::transformEigenToMsg(transform, transformStamped.transform);
  transform_broadcaster.sendTransform(transformStamped);

  auto distances = plane_calibration_->getDistancesToMaxDeviations(random_plane_image);

  depth_visualizer_->publishDouble("debug/x_positive", distances[0]);
  depth_visualizer_->publishDouble("debug/y_positive", distances[1]);
  depth_visualizer_->publishDouble("debug/x_negative", distances[2]);
  depth_visualizer_->publishDouble("debug/y_negative", distances[3]);

  auto distances_diffs = plane_calibration_->getXYDistanceDiff(random_plane_image);
  depth_visualizer_->publishDouble("debug/x_diff", distances_diffs.first);
  depth_visualizer_->publishDouble("debug/y_diff", distances_diffs.second);

  bool calculate_errors = false;
  MagicMultiplierEstimation::Result magic_result = magic_estimator_->estimate(calculate_errors);

  double px_estimation = magic_result.multiplier_x * distances_diffs.first;
  double py_estimation = magic_result.multiplier_y * distances_diffs.second;
  depth_visualizer_->publishDouble("debug/px_degree_estimated", ecl::radians_to_degrees(px_estimation));
  depth_visualizer_->publishDouble("debug/py_degree_estimated", ecl::radians_to_degrees(py_estimation));

  double px_estimation_abs_error = px_estimation - px_offset_;
  double py_estimation_abs_error = py_estimation - py_offset_;

  depth_visualizer_->publishDouble("debug/px_estimation_abs_error_degree",
                                   ecl::radians_to_degrees(px_estimation_abs_error));
  depth_visualizer_->publishDouble("debug/py_estimation_abs_error_degree",
                                   ecl::radians_to_degrees(py_estimation_abs_error));

  double px_estimation_error = px_offset_ == 0.0 ? 0.0 : px_estimation_abs_error / px_offset_;
  double py_estimation_error = py_offset_ == 0.0 ? 0.0 : py_estimation_abs_error / py_offset_;

  depth_visualizer_->publishDouble("debug/px_estimation_error", px_estimation_error);
  depth_visualizer_->publishDouble("debug/py_estimation_error", py_estimation_error);
}

} /* end namespace */

PLUGINLIB_EXPORT_CLASS(plane_calibration::PlaneCalibrationNodelet, nodelet::Nodelet)
