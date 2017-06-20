#include "plane_calibration/depth_visualizer.hpp"

#include <depth_image_proc/depth_conversions.h>
#include <plane_calibration/depth_visualizer.hpp>
#include <std_msgs/Float32.h>

#include "plane_calibration/image_msg_eigen_converter.hpp"
#include "plane_calibration/plane_to_depth_image.hpp"

namespace plane_calibration
{

DepthVisualizer::DepthVisualizer(ros::NodeHandle node_handle, std::string frame_id) :
    node_handle_(node_handle)
{
  frame_id_ = frame_id;
}

void DepthVisualizer::setCameraModel(const image_geometry::PinholeCameraModel& camera_model)
{
  std::lock_guard<std::mutex> lock(camera_model_mutex_);
  camera_model_ = camera_model;
}

void DepthVisualizer::publishImage(const std::string& topic, const Eigen::MatrixXf& image_matrix, std::string frame_id)
{
  addPublisherIfNotExist<sensor_msgs::Image>(topic);

  if (publishers_[topic].getNumSubscribers() == 0)
  {
    return;
  }

  sensor_msgs::Image image_msg;
  image_msg.header.frame_id = frame_id != "" ? frame_id : frame_id_;
  ImageMsgEigenConverter::convert(image_matrix, image_msg);
  publishImage(topic, image_msg);
}

void DepthVisualizer::publishImage(const std::string& topic, const sensor_msgs::Image& image_msg)
{
  addPublisherIfNotExist<sensor_msgs::Image>(topic);

  if (publishers_[topic].getNumSubscribers() > 0)
  {
    publishers_[topic].publish(image_msg);
  }
}

void DepthVisualizer::publishCloud(const std::string& topic, const Eigen::Affine3d& plane_transformation,
                                   const CameraModel::Parameters& camera_model_paramaters, std::string frame_id)
{
  addPublisherIfNotExist<sensor_msgs::PointCloud2>(topic);

  if (publishers_[topic].getNumSubscribers() == 0)
  {
    return;
  }

  Eigen::MatrixXf plane = PlaneToDepthImage::convert(plane_transformation, camera_model_paramaters);

  sensor_msgs::Image image_msg;
  image_msg.header.frame_id = frame_id != "" ? frame_id : frame_id_;
  ImageMsgEigenConverter::convert(plane, image_msg);
  publishCloud(topic, image_msg);
}

void DepthVisualizer::publishCloud(const std::string& topic, const Eigen::MatrixXf& image_matrix, std::string frame_id)
{
  addPublisherIfNotExist<sensor_msgs::PointCloud2>(topic);

  if (publishers_[topic].getNumSubscribers() == 0)
  {
    return;
  }

  sensor_msgs::Image image_msg;
  image_msg.header.frame_id = frame_id != "" ? frame_id : frame_id_;
  ImageMsgEigenConverter::convert(image_matrix, image_msg);
  publishCloud(topic, image_msg);
}

void DepthVisualizer::publishCloud(const std::string& topic, const sensor_msgs::Image& image_msg)
{
  addPublisherIfNotExist<sensor_msgs::PointCloud2>(topic);

  if (publishers_[topic].getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2Ptr point_cloud_msg_ptr = imageMsgToPointCloud(image_msg);
    publishers_[topic].publish(point_cloud_msg_ptr);
  }
}

void DepthVisualizer::publishDouble(const std::string& topic, const double& value)
{
  addPublisherIfNotExist<std_msgs::Float32>(topic);

  if (publishers_[topic].getNumSubscribers() > 0)
  {
    std_msgs::Float32 msg;
    msg.data = value;
    publishers_[topic].publish(msg);
  }
}

template<typename MsgType>
void DepthVisualizer::addPublisherIfNotExist(const std::string& topic)
{
  auto publisher_match = publishers_.find(topic);
  bool publisher_exists_already = publisher_match != publishers_.end();

  if (!publisher_exists_already)
  {
    publishers_.emplace(topic, node_handle_.advertise<MsgType>(topic, 1));
    return;
  }
}

sensor_msgs::PointCloud2Ptr DepthVisualizer::imageMsgToPointCloud(const sensor_msgs::Image& image_msg)
{
  std::lock_guard<std::mutex> lock(camera_model_mutex_);
  if (!camera_model_.initialized())
  {
    return boost::make_shared<sensor_msgs::PointCloud2>();
  }

  return floatImageMsgToPointCloud(image_msg, camera_model_);
}

sensor_msgs::PointCloud2Ptr DepthVisualizer::floatImageMsgToPointCloud(
    const sensor_msgs::Image& image_msg, const image_geometry::PinholeCameraModel& camera_model)
{
  sensor_msgs::ImagePtr image_msg_ptr = boost::make_shared<sensor_msgs::Image>(image_msg);

  sensor_msgs::PointCloud2Ptr cloud_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>();
  cloud_msg_ptr->header = image_msg.header;
  cloud_msg_ptr->height = image_msg.height;
  cloud_msg_ptr->width = image_msg.width;
  cloud_msg_ptr->is_dense = false;
  cloud_msg_ptr->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg_ptr);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  depth_image_proc::convert<float>(image_msg_ptr, cloud_msg_ptr, camera_model);
  return cloud_msg_ptr;
}

} /* end namespace */
