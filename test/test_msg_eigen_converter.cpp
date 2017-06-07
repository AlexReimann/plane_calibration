
#include <gtest/gtest.h>
#include <algorithm>
#include <sensor_msgs/image_encodings.h>

#include "plane_calibration/image_msg_eigen_converter.hpp"

using namespace plane_calibration;

TEST(Converter, convertFloat)
{
  Eigen::MatrixXf original_matrix = Eigen::MatrixXf::Random(640, 480);

  sensor_msgs::ImagePtr image = sensor_msgs::ImagePtr(new sensor_msgs::Image());
  ImageMsgEigenConverter::convert(original_matrix, *image);

  Eigen::MatrixXf converted_matrix;
  ImageMsgEigenConverter::convert(image, converted_matrix);

  EXPECT_EQ(original_matrix, converted_matrix);

  original_matrix = original_matrix * 1.1;
  EXPECT_NE(original_matrix, converted_matrix);
}

TEST(Converter, convertShort)
{
  unsigned int height = 640;
  unsigned int width = 480;

  sensor_msgs::ImagePtr original_image = sensor_msgs::ImagePtr(new sensor_msgs::Image());
  original_image->width = width;
  original_image->height = height;
  original_image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  original_image->data.resize(height * width * sizeof(unsigned short));
  std::generate(original_image->data.begin(), original_image->data.end(), std::rand);

  const unsigned char* data_pointer = &(original_image->data.front());
  const unsigned short* short_data_pointer = reinterpret_cast<const unsigned short*>(data_pointer);
  Eigen::Map<const Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map(short_data_pointer, height, width);

  Eigen::MatrixXf matrix;
  ImageMsgEigenConverter::convert(original_image, matrix);

  EXPECT_EQ(matrix.cast<unsigned short>(), map.cast<unsigned short>());

  original_image->data.back() += 1;
  EXPECT_NE(matrix.cast<unsigned short>(), map.cast<unsigned short>());
}

//Run with "catkin_make run_tests_plane_calibration"
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
