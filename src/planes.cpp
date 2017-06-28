#include "plane_calibration/planes.hpp"

#include <cmath>
#include <iostream>

namespace plane_calibration
{

Planes::Planes(const CalibrationParameters::Parameters& parameters, const PlaneToDepthImage& plane_to_depth) :
    plane_to_depth_(plane_to_depth)
{
  pair_count_ = parameters.precomputed_plane_pairs_count_;
  max_deviation_ = parameters.max_deviation_;

  translation_ = Eigen::Translation3d(parameters.ground_plane_offset_);
  base_rotation_ = parameters.rotation_;

  makePlanes();
}

void Planes::makePlanes()
{
  // When fitting a plane close to the max deviation we have to have
  // some planes outside the max deviation borders
  double buffer_multiplier = 2.2;
  double angle_step_size = max_deviation_ * buffer_multiplier / pair_count_;
  double angle = max_deviation_ * buffer_multiplier;

  for (int i = 0; i < pair_count_; ++i)
  {
    addPlanePairs(angle);
    angle -= angle_step_size;
  }

  addPlanes(0.0);
}

void Planes::addPlanePairs(const double& angle)
{
  addPlanes(angle);
  addPlanes(-angle);
}

void Planes::addPlanes(const double& angle)
{
  Eigen::Vector2d angles_x(angle, 0.0);
  MatrixPlanePtr plane_x = std::make_shared<Eigen::MatrixXf>(makePlane(angles_x));
  x_planes_.insert(std::pair<double, MatrixPlanePtr>(angle, plane_x));

  Eigen::Vector2d angles_y(0.0, angle);
  MatrixPlanePtr plane_y = std::make_shared<Eigen::MatrixXf>(makePlane(angles_y));
  y_planes_.insert(std::pair<double, MatrixPlanePtr>(angle, plane_y));
}

Eigen::MatrixXf Planes::makePlane(const Eigen::Vector2d& angles)
{
  Eigen::AngleAxisd tilt_x(angles.x(), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd tilt_y(angles.y(), Eigen::Vector3d::UnitY());
  Eigen::Affine3d transform = translation_ * base_rotation_ * tilt_x * tilt_y;

  return plane_to_depth_.convert(transform);
}

std::pair<MatrixPlanePtr, MatrixPlanePtr> Planes::getFittingXTiltPlanes(const double& angle, const double& deviation)
{
  return getFittingTiltPlanes(x_planes_, angle, deviation);
}

std::pair<MatrixPlanePtr, MatrixPlanePtr> Planes::getFittingYTiltPlanes(const double& angle, const double& deviation)
{
  return getFittingTiltPlanes(y_planes_, angle, deviation);
}

std::pair<MatrixPlanePtr, MatrixPlanePtr> Planes::getFittingTiltPlanes(const std::map<double, MatrixPlanePtr>& planes,
                                                                       const double& angle, const double& deviation)
{
  std::pair<double, double> plane_indicies = getDeviationPlaneKeys(angle, deviation);

  MatrixPlanePtr first = planes.at(plane_indicies.first);
  MatrixPlanePtr second = planes.at(plane_indicies.second);

  return std::pair<MatrixPlanePtr, MatrixPlanePtr>(first, second);
}

std::pair<double, double> Planes::getDeviationPlaneKeys(const double& angle, const double& deviation)
{
  double upper_angle = angle + deviation;
  double lower_angle = angle - deviation;

  auto upper_it = x_planes_.lower_bound(upper_angle); //maybe confusing, see official docu
  auto lower_it = x_planes_.lower_bound(lower_angle);
  --lower_it; //take the value lower than this

  if (upper_it == x_planes_.end())
  {
    upper_it = x_planes_.begin();
  }

  if (lower_it == x_planes_.end())
  {
    --lower_it;
  }

  return std::make_pair(upper_it->first, lower_it->first);
}

} /* end namespace */
