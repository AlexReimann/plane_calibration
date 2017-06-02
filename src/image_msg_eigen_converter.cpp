#include "plane_calibration/image_msg_eigen_converter.hpp"

namespace plane_calibration
{

Eigen::MatrixXd ImageMsgEigenConverter::convert(const sensor_msgs::ImageConstPtr& image_msg)
{
  return Eigen::MatrixXd();
}

sensor_msgs::Image ImageMsgEigenConverter::convert(const Eigen::MatrixXd& image_msg)
{
  return sensor_msgs::Image();
}

} /* end namespace */
