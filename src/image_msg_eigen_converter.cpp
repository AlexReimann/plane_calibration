#include "plane_calibration/image_msg_eigen_converter.hpp"

#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

namespace plane_calibration
{

bool ImageMsgEigenConverter::convert(const sensor_msgs::ImageConstPtr& image_msg, Eigen::MatrixXf& out_matrix)
{
  if (image_msg->is_bigendian != 0)
  {
    ROS_ERROR_STREAM("[ImageMsgEigenConverter]: Big endian not supported!");
    return false;
  }

  //only 32FC1 and 16UC1 observed in the wild
  std::string encoding = image_msg->encoding;

  if (encoding != sensor_msgs::image_encodings::TYPE_16UC1 && encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
    ROS_ERROR_STREAM("[ImageMsgEigenConverter]: Unsupported encoding: " << encoding);
  }

  const unsigned char* data_pointer = &(image_msg->data.front());

  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    out_matrix = dataToMatrix<unsigned short>(data_pointer, image_msg->height, image_msg->width);
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    out_matrix = dataToMatrix<float>(data_pointer, image_msg->height, image_msg->width);
  }

  return true;
}

template<typename DataType>
Eigen::MatrixXf ImageMsgEigenConverter::dataToMatrix(const unsigned char* data_pointer, const int& height,
                                                     const int& width)
{
  const DataType* short_data_pointer = reinterpret_cast<const DataType*>(data_pointer);

  Eigen::Map<const Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map(short_data_pointer,
                                                                                                 height, width);

  // C++ template trickery: https://stackoverflow.com/q/29754251
  return map.template cast<float>();
}

bool ImageMsgEigenConverter::convert(const Eigen::MatrixXf& _matrix, sensor_msgs::Image& out_image_msg)
{
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix(_matrix);

  out_image_msg.height = matrix.rows();
  out_image_msg.width = matrix.cols();
  out_image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_image_msg.is_bigendian = 0;
  out_image_msg.step = matrix.cols() * sizeof(matrix.value());

  size_t size = out_image_msg.step * matrix.rows();
  out_image_msg.data.resize(size);

  char* data_start_char = reinterpret_cast<char*>(&out_image_msg.data.front());
  memcpy(data_start_char, matrix.data(), size);

  return true;
}

} /* end namespace */
