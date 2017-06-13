#include "plane_calibration/plane_to_depth_image.hpp"

#include <iostream>

namespace plane_calibration
{

using namespace Eigen;

PlaneToDepthImage::Errors PlaneToDepthImage::getErrors(const Eigen::Affine3d& plane_transformation,
                                                       const CameraModel::Parameters& camera_model_paramaters,
                                                       Eigen::MatrixXf image_matrix)
{
  Eigen::MatrixXf plane = convert(plane_transformation, camera_model_paramaters);

  Eigen::MatrixXf difference = (plane - image_matrix).cwiseAbs();

  int not_nan_count = (difference.array() == difference.array()).count();

  std::cout << "not_nan_count: " << not_nan_count << std::endl;

  difference = difference.unaryExpr([](float v)
  { return std::isnan(v) ? 0.0 : v;});

  double mean = difference.sum() / not_nan_count;

  Errors errors;
  errors.mean = mean;
  errors.min = difference.minCoeff();
  errors.max = difference.maxCoeff();

  return errors;
}

MatrixXf PlaneToDepthImage::convert(const Affine3d& plane_transformation,
                                    const CameraModel::Parameters& camera_model_paramaters)
{
  MatrixXd result_image_matrix;

  Vector3d translation = plane_transformation.translation();
  double distance = translation.norm();
  int height = camera_model_paramaters.height_;

  Vector4d z_axis(0.0, 0.0, 1.0, 0.0);
  Vector4d plane_normal = plane_transformation * z_axis;
  Hyperplane<double, 3> plane(plane_normal.topRows(3), translation);

  // inverting the depth to point cloud calculations, see package depth_image_proc -> depth_conversions.h
  // (u - c) * depth * (1 / f) -> depth * ( (u - c) / f) -> depth * multiplier
  std::pair<MatrixXd, MatrixXd> xy_multipliers = depthCalculationXYMultiplier(camera_model_paramaters);

  // a*x + b*y + c*z + d = 0
  // a * (depth * x_multiplier) + b * (depth * y_multiplier) + c * depth + d = 0
  // depth * ( a * x_multiplier + b * y_multiplier + c) + d = 0
  // depth * ( a * x_multiplier + b * y_multiplier + c) = -d
  // depth = -d / ( a * x_multiplier + b * y_multiplier + c)

  MatrixXd x = plane.coeffs().coeff(0) * xy_multipliers.first;
  MatrixXd y = plane.coeffs().coeff(1) * xy_multipliers.second;
  double z = plane.coeffs().coeff(2); //same for all rays

  result_image_matrix = -plane.coeffs().coeff(3) / ((x + y).array() + z);

  return result_image_matrix.cast<float>();
}

std::pair<MatrixXd, MatrixXd> PlaneToDepthImage::depthCalculationXYMultiplier(
    const CameraModel::Parameters& camera_model_paramaters)
{
  MatrixXd xy_factor_matrix(2 * camera_model_paramaters.height_, camera_model_paramaters.width_);

  VectorXd x_indices_lin = VectorXd::LinSpaced(camera_model_paramaters.width_, 0.0, camera_model_paramaters.width_ - 1);
  VectorXd y_indices_lin = VectorXd::LinSpaced(camera_model_paramaters.height_, 0.0,
                                               camera_model_paramaters.height_ - 1);

  MatrixXd x_indices_matrix = (x_indices_lin * VectorXd::Ones(camera_model_paramaters.height_).transpose()).transpose();
  MatrixXd y_indices_matrix = y_indices_lin * VectorXd::Ones(camera_model_paramaters.width_).transpose();

  // (u - c) * d * (1 / f) -> d * ( (u - c) / f) -> d * multiplier
  MatrixXd x_multiplier = (x_indices_matrix.array() - camera_model_paramaters.center_x_) / camera_model_paramaters.f_x_;
  MatrixXd y_multiplier = (y_indices_matrix.array() - camera_model_paramaters.center_y_) / camera_model_paramaters.f_y_;

  std::pair<MatrixXd, MatrixXd> multiplier_pair(x_multiplier, y_multiplier);
  return multiplier_pair;
}

} /* end namespace */
