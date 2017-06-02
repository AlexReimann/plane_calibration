#ifndef plane_calibration_SRC_IMAGE_MSG_EIGEN_CONVERTER_HPP_
#define plane_calibration_SRC_IMAGE_MSG_EIGEN_CONVERTER_HPP_

#include <sensor_msgs/Image.h>
#include <Eigen/Dense>

namespace plane_calibration
{

class ImageMsgEigenConverter
{
public:
  inline static Eigen::MatrixXd convert(const sensor_msgs::ImageConstPtr& image_msg);
  inline static sensor_msgs::Image convert(const Eigen::MatrixXd& image_msg);

protected:

};

} /* end namespace */

#endif
