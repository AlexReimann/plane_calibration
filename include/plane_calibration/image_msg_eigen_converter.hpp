#ifndef plane_calibration_SRC_IMAGE_MSG_EIGEN_CONVERTER_HPP_
#define plane_calibration_SRC_IMAGE_MSG_EIGEN_CONVERTER_HPP_

#include <sensor_msgs/Image.h>
#include <Eigen/Dense>

namespace plane_calibration
{

class ImageMsgEigenConverter
{
public:
  //TODO make fixed size (640x480, 320x240) for faster stuff
  static bool convert(const sensor_msgs::ImageConstPtr& image_msg, Eigen::MatrixXf& out_matrix);
  static bool convert(const Eigen::MatrixXf& matrix, sensor_msgs::Image& out_image_msg);

protected:
  template<typename DataType>
  inline static Eigen::MatrixXf dataToMatrix(const unsigned char* data_pointer, const int& height, const int& width);
};

} /* end namespace */

#endif
