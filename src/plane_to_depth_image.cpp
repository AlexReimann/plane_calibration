#include "plane_calibration/plane_to_depth_image.hpp"

#include <iostream>

namespace plane_calibration
{

using namespace Eigen;

MatrixXf PlaneToDepthImage::convert(const Affine3d& plane_transformation, const CameraModel::Parameters& camera_model_paramaters)
{
  MatrixXd result_image_matrix;

  Vector3d translation = plane_transformation.translation().const_cast_derived();
  double distance = translation.norm();
  int height = camera_model_paramaters.height_;

  Vector4d z_axis(0.0, 0.0, 1.0, 0.0);
  Vector4d plane_normal = plane_transformation * z_axis;

  Hyperplane<double, 3> plane(plane_normal.topRows(3), translation);
  std::pair<MatrixXd, MatrixXd> xy_factors = depthVectors(camera_model_paramaters);

  MatrixXd x = plane.coeffs().coeff(0) * xy_factors.first;
  MatrixXd y = plane.coeffs().coeff(1) * xy_factors.second;
  double z = plane.coeffs().coeff(2); //same for all rays

  result_image_matrix = -plane.coeffs().coeff(3) / ((x + y).array() + z);

  return result_image_matrix.cast<float>();
}

std::pair<MatrixXd, MatrixXd> PlaneToDepthImage::depthVectors(const CameraModel::Parameters& camera_model_paramaters)
{
  MatrixXd xy_factor_matrix(2 * camera_model_paramaters.height_, camera_model_paramaters.width_);

  VectorXd x_indices_lin = VectorXd::LinSpaced(camera_model_paramaters.width_, 0.0, camera_model_paramaters.width_ - 1);
  VectorXd y_indices_lin = VectorXd::LinSpaced(camera_model_paramaters.height_, 0.0, camera_model_paramaters.height_ - 1);

  MatrixXd x_indices_matrix = (x_indices_lin * VectorXd::Ones(camera_model_paramaters.height_).transpose()).transpose();
  MatrixXd y_indices_matrix = y_indices_lin * VectorXd::Ones(camera_model_paramaters.width_).transpose();

  MatrixXd x_factor = (x_indices_matrix.array() - camera_model_paramaters.center_x_) / camera_model_paramaters.f_x_;
  MatrixXd y_factor = (y_indices_matrix.array() - camera_model_paramaters.center_y_) / camera_model_paramaters.f_y_;

  std::pair<MatrixXd, MatrixXd> factor_pair (x_factor, y_factor);
  return factor_pair;
}

} /* end namespace */
