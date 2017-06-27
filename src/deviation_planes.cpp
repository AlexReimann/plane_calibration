#include "plane_calibration/deviation_planes.hpp"

#include <iostream>
#include <ecl/geometry/angle.hpp>

namespace plane_calibration
{
DeviationPlanes::DeviationPlanes(const PlaneToDepthImage& plane_to_depth, const VisualizerInterfacePtr& depth_visualizer) :
    plane_to_depth_(plane_to_depth)
{
  planes_.resize(4);
  transform_.resize(4);
  deviation_ = 0.0;
  depth_visualizer_ = depth_visualizer;
}

void DeviationPlanes::init(const CalibrationParameters::Parameters& parameters)
{
  bool use_max_deviation = true;
  update(parameters, use_max_deviation);
}

void DeviationPlanes::update(const CalibrationParameters::Parameters& parameters, const bool& use_max_deviation)
{
  deviation_ = use_max_deviation ? parameters.max_deviation_ : parameters.deviation_;

  Eigen::Translation3d translation = Eigen::Translation3d(parameters.ground_plane_offset_);

  Eigen::AngleAxisd x_tilt_positive(deviation_, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd x_tilt_negative(-deviation_, Eigen::Vector3d::UnitX());

  Eigen::AngleAxisd y_tilt_positive(deviation_, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd y_tilt_negative(-deviation_, Eigen::Vector3d::UnitY());

  transform_[indexXPositive()] = translation * parameters.rotation_ * x_tilt_positive;
  transform_[indexXNegative()] = translation * parameters.rotation_ * x_tilt_negative;
  transform_[indexYPositive()] = translation * parameters.rotation_ * y_tilt_positive;
  transform_[indexYNegative()] = translation * parameters.rotation_ * y_tilt_negative;

  for (int i = 0; i < transform_.size(); ++i)
  {
    planes_[i] = plane_to_depth_.convert(transform_[i]);
  }

  bool matrix_has_nans = false;
  double x_distance = getDistance(xPositive(), xNegative(), matrix_has_nans);
  double y_distance = getDistance(yPositive(), yNegative(), matrix_has_nans);

  double x_magic_multiplier = deviation_ / x_distance;
  double y_magic_multiplier = deviation_ / y_distance;

  magic_multipliers_ = std::make_pair(x_magic_multiplier, y_magic_multiplier);
}

std::pair<double, double> DeviationPlanes::estimateAngles(const Eigen::MatrixXf& plane, const bool& debug)
{
  std::pair<double, double> distance_diffs = getDistanceDiffs(plane, debug);

  double px_estimation = magic_multipliers_.first * distance_diffs.first;
  double py_estimation = magic_multipliers_.second * distance_diffs.second;

  if (debug)
  {
    depth_visualizer_->publishCloud("debug/xpositive", xPositive());
    depth_visualizer_->publishCloud("debug/xnegative", xNegative());
    depth_visualizer_->publishCloud("debug/ypositive", yPositive());
    depth_visualizer_->publishCloud("debug/ynegative", yNegative());

    std::cout << "DeviationPlanes/estimateAngles: plane distance diffs: " << distance_diffs.first << ", "
        << distance_diffs.second << std::endl;
    std::cout << "DeviationPlanes/estimateAngles: magic_multipliers_: " << magic_multipliers_.first << ", "
        << magic_multipliers_.second << std::endl;
    std::cout << "DeviationPlanes/estimateAngles: angle estimations: " << ecl::radians_to_degrees(px_estimation) << ", "
        << ecl::radians_to_degrees(py_estimation) << std::endl;
  }

  return std::make_pair(px_estimation, py_estimation);
}

std::pair<double, double> DeviationPlanes::getDistanceDiffs(const Eigen::MatrixXf& plane, const bool& debug)
{
  std::vector<double> distances = getDistances(planes_, plane);

  if (debug)
  {
    depth_visualizer_->publishImage("debug/xpositive_diff", plane - xPositive());
    depth_visualizer_->publishImage("debug/xnegative_diff", plane - xNegative());
    depth_visualizer_->publishImage("debug/ypositive_diff", plane - yPositive());
    depth_visualizer_->publishImage("debug/ynegative_diff", plane - yNegative());

    for (int i = 0; i < distances.size(); ++i)
    {
      std::cout << "DeviationPlanes/getDistanceDiffs: plane distances: " << i << ": " << distances[i] << std::endl;
    }
  }

  double x_diff = distances[indexXNegative()] - distances[indexXPositive()];
  double y_diff = distances[indexYNegative()] - distances[indexYPositive()];

  return std::make_pair(x_diff, y_diff);
}

std::vector<double> DeviationPlanes::getDistances(const std::vector<Eigen::MatrixXf>& from, const Eigen::MatrixXf& to)
{
  std::vector<double> distances;
  for (int i = 0; i < from.size(); ++i)
  {
    bool matrix_has_nans = true;
    distances.push_back(getDistance(from[i], to, matrix_has_nans));
  }
  return distances;
}

double DeviationPlanes::getDistance(const Eigen::MatrixXf& from, const Eigen::MatrixXf& to, const bool& remove_nans)
{
  Eigen::MatrixXf difference = (to - from).cwiseAbs2();

  if (remove_nans)
  {
    //TODO optimize this, so we only have to do it once for angle estimation
    difference = difference.unaryExpr([](float v)
    { return std::isnan(v) ? 0.0f : v;});
  }

  double distance = difference.sum();
  return distance;
}

double DeviationPlanes::getDeviation()
{
  return deviation_;
}

std::pair<double, double> DeviationPlanes::getMultipliers()
{
  return magic_multipliers_;
}

Eigen::MatrixXf DeviationPlanes::xPositive()
{
  return planes_[indexXPositive()];
}

Eigen::MatrixXf DeviationPlanes::xNegative()
{
  return planes_[indexXNegative()];
}

Eigen::MatrixXf DeviationPlanes::yPositive()
{
  return planes_[indexYPositive()];
}

Eigen::MatrixXf DeviationPlanes::yNegative()
{
  return planes_[indexYNegative()];
}

Eigen::Affine3d DeviationPlanes::xPositiveTransform()
{
  return transform_[indexXPositive()];
}

Eigen::Affine3d DeviationPlanes::xNegativeTransform()
{
  return transform_[indexXNegative()];
}

Eigen::Affine3d DeviationPlanes::yPositiveTransform()
{
  return transform_[indexYPositive()];
}

Eigen::Affine3d DeviationPlanes::yNegativeTransform()
{
  return transform_[indexYNegative()];
}

int DeviationPlanes::indexXPositive()
{
  return 0;
}

int DeviationPlanes::indexXNegative()
{
  return 1;
}

int DeviationPlanes::indexYPositive()
{
  return 2;
}

int DeviationPlanes::indexYNegative()
{
  return 3;
}

} /* end namespace */
