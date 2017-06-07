#include "plane_calibration/plane_to_depth_image.hpp"

#include <iostream>

namespace plane_calibration
{

using namespace Eigen;

MatrixXf PlaneToDepthImage::convert(const Affine3d& plane_transformation, const CameraModel::Parameters& camera_model_paramaters)
{
  MatrixXf result_image_matrix;

  Vector3d translation = plane_transformation.translation().const_cast_derived();
  double distance = translation.norm();

  Vector4d z_axis(0.0, 0.0, 1.0, 0.0);
  Vector4d plane_normal = plane_transformation * z_axis;

  Hyperplane<double, 3> plane(plane_normal.topRows(3), translation);
  return result_image_matrix;
}

Eigen::MatrixXd PlaneToDepthImage::depthVectors(const CameraModel::Parameters& camera_model_paramaters)
{
  MatrixXd xy_factor_matrix(2 * camera_model_paramaters.height_, camera_model_paramaters.width_);

  VectorXd x_indices_lin = VectorXd::LinSpaced(camera_model_paramaters.height_, 0.0, camera_model_paramaters.height_ - 1);
  VectorXd y_indices_lin = VectorXd::LinSpaced(camera_model_paramaters.width_, 0.0, camera_model_paramaters.width_ - 1);

  MatrixXd x_indices_matrix = x_indices_lin * VectorXd::Ones(camera_model_paramaters.width_).transpose();
  MatrixXd y_indices_matrix = (y_indices_lin * VectorXd::Ones(camera_model_paramaters.height_).transpose()).transpose();

  MatrixXd x_factor = camera_model_paramaters.f_x_ / (x_indices_matrix.array() - camera_model_paramaters.center_x_);
  MatrixXd y_factor = camera_model_paramaters.f_y_ / (x_indices_matrix.array() - camera_model_paramaters.center_x_);

  xy_factor_matrix << x_factor, y_factor;

  return xy_factor_matrix;
}

} /* end namespace */
