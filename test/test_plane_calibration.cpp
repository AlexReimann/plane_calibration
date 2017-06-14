#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <ecl/geometry/angle.hpp>
#include "plane_calibration/plane_to_depth_image.hpp"
#include "plane_calibration/plane_calibration.hpp"
#include "plane_calibration/magic_multiplier_estimation.hpp"

using namespace plane_calibration;

TEST(PlaneCalibration, one_shot)
{
  int width = 640;
  int height = 480;
  double c_x = 321.3;
  double c_y = 212;
  double f_x = 570.3422;
  double f_y = 570.3422;
  CameraModel camera_model(c_x, c_y, f_x, f_y, width, height);

  double max_deviation = 0.1;
  double px = -0.628319;
  double py = 0.057;
  double pz = 0.0;
  Eigen::AngleAxisd start_rotation;
  start_rotation = Eigen::AngleAxisd(px, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(py, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(pz, Eigen::Vector3d::UnitZ());

  double px_offset = -0.023;
  double py_offset = 0.04;
  Eigen::AngleAxisd rotation_offset;
  rotation_offset = Eigen::AngleAxisd(px_offset, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(py_offset, Eigen::Vector3d::UnitY());

  Eigen::Vector3d ground_plane_offset(0.0, -0.16, 0.96);

  Eigen::Affine3d transform = Eigen::Translation3d(ground_plane_offset) * start_rotation * rotation_offset;
  Eigen::MatrixXf plane = PlaneToDepthImage::convert(transform, camera_model.getParameters());
  Eigen::MatrixXf noise = Eigen::MatrixXf::Random(plane.rows(), plane.cols());
  Eigen::MatrixXf random_plane_image = plane + 0.02 * noise;

  PlaneCalibration::Parameters parameters(ground_plane_offset, max_deviation, start_rotation);

  PlaneCalibrationPtr plane_calibration = std::make_shared<PlaneCalibration>();
  plane_calibration->updateParameters(camera_model);
  plane_calibration->updateParameters(parameters);
  plane_calibration->updateMaxDeviationPlanesIfNeeded();

  MagicMultiplierEstimation magic_estimator(camera_model, plane_calibration, max_deviation, max_deviation);

  bool calculate_errors = false;
  MagicMultiplierEstimation::Result magic_result = magic_estimator.estimate(calculate_errors);

  std::pair<double, double> one_shot_result = plane_calibration->estimateAngles(random_plane_image,
                                                                               magic_result.multiplier_x,
                                                                               magic_result.multiplier_y);

  double estimated_px = one_shot_result.first;
  double estimated_py = one_shot_result.second;

  double epsilon = ecl::degrees_to_radians(0.5);
  EXPECT_NEAR(estimated_px, px_offset, epsilon);
  EXPECT_NEAR(estimated_py, py_offset, epsilon);
}
