#include <gtest/gtest.h>

#include "plane_calibration/plane_to_depth_image.hpp"

using namespace plane_calibration;

TEST(PlaneToDepth, multiplier_positive_only)
{
  CameraModel::Parameters parameters(0.0, 0.0, 1.0, 1.0, 10, 12);

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> result_xy_multiplier = PlaneToDepthImage::depthCalculationXYMultiplier(
      parameters);

  Eigen::MatrixXd x = result_xy_multiplier.first;
  Eigen::MatrixXd y = result_xy_multiplier.second;

  EXPECT_EQ(x(0, 0), 0.0);
  EXPECT_EQ(x(2, 0), 0.0);
  EXPECT_EQ(x(0, 4), 4.0);
  EXPECT_EQ(x(9, 4), 4.0);

  EXPECT_EQ(y(0, 0), 0.0);
  EXPECT_EQ(y(2, 0), 2.0);
  EXPECT_EQ(y(0, 4), 0.0);
  EXPECT_EQ(y(9, 4), 9.0);
}

TEST(PlaneToDepth, multiplier_center)
{
  double cx = 3.1;
  double cy = 8.0001;
  CameraModel::Parameters parameters(cx, cy, 1.0, 1.0, 10, 12);

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> result_xy_multiplier = PlaneToDepthImage::depthCalculationXYMultiplier(
      parameters);

  Eigen::MatrixXd x = result_xy_multiplier.first;
  Eigen::MatrixXd y = result_xy_multiplier.second;

  EXPECT_EQ(x(0, 0), -cx);
  EXPECT_EQ(x(2, 0), -cx);
  EXPECT_EQ(x(0, 4), 4.0 - cx);
  EXPECT_EQ(x(9, 4), 4.0 - cx);

  EXPECT_EQ(y(0, 0), -cy);
  EXPECT_EQ(y(2, 0), 2.0 - cy);
  EXPECT_EQ(y(0, 4), -cy);
  EXPECT_EQ(y(9, 4), 9.0 - cy);
}

TEST(PlaneToDepth, multiplier_focal)
{
  double cx = 3.1;
  double cy = 8.0001;
  double fx = 0.34;
  double fy = 0.541111;
  CameraModel::Parameters parameters(cx, cy, fx, fy, 10, 12);

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> result_xy_multiplier = PlaneToDepthImage::depthCalculationXYMultiplier(
      parameters);

  Eigen::MatrixXd x = result_xy_multiplier.first;
  Eigen::MatrixXd y = result_xy_multiplier.second;

  EXPECT_EQ(x(0, 0), -cx / fx);
  EXPECT_EQ(x(2, 0), -cx / fx);
  EXPECT_EQ(x(0, 4), (4.0 - cx)  / fx);
  EXPECT_EQ(x(9, 4), (4.0 - cx)  / fx);

  EXPECT_EQ(y(0, 0), (-cy) / fy);
  EXPECT_EQ(y(2, 0), (2.0 - cy) / fy);
  EXPECT_EQ(y(0, 4), (-cy) / fy);
  EXPECT_EQ(y(9, 4), (9.0 - cy) / fy);
}

TEST(PlaneToDepth, multiplier_static)
{
  double cx = 3.1;
  double cy = 8.0001;
  double fx = 0.34;
  double fy = 0.541111;
  CameraModel::Parameters parameters(cx, cy, fx, fy, 10, 12);

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> result_xy_multiplier = PlaneToDepthImage::depthCalculationXYMultiplier(
      parameters);

  Eigen::MatrixXd x = result_xy_multiplier.first;
  Eigen::MatrixXd y = result_xy_multiplier.second;

  double epsilon = 0.0001;
  EXPECT_NEAR(x(0, 0),  -9.11765, epsilon);
  EXPECT_NEAR(x(2, 0),  -9.11765, epsilon);
  EXPECT_NEAR(x(0, 4), 2.64706, epsilon);
  EXPECT_NEAR(x(9, 4), 2.64706, epsilon);

  EXPECT_NEAR(y(0, 0), -14.7846, epsilon);
  EXPECT_NEAR(y(2, 0), -11.0885, epsilon);
  EXPECT_NEAR(y(0, 4), -14.7846, epsilon);
  EXPECT_NEAR(y(9, 4), 1.84786, epsilon);
}
