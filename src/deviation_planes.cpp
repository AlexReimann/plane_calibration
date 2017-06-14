#include "plane_calibration/deviation_planes.hpp"

namespace plane_calibration
{
DeviationPlanes::DeviationPlanes(PlaneToDepthImage plane_to_depth) :
    plane_to_depth_(plane_to_depth)
{
  planes_.resize(4);
  transform_.resize(4);
  deviation_ = 0.0;
}

void DeviationPlanes::init(CalibrationParameters::Parameters parameters)
{
  bool use_max_deviation = true;
  update(parameters, use_max_deviation);
}

void DeviationPlanes::update(CalibrationParameters::Parameters parameters, bool use_max_deviation)
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

  double x_distance = getDistance(xPositive(), xNegative());
  double y_distance = getDistance(yPositive(), yNegative());

  double x_distance_normalized = x_distance / xPositive().cols();
  double y_distance_normalized = y_distance / xPositive().rows();

  double x_magic_multiplier = deviation_ / x_distance_normalized;
  double y_magic_multiplier = deviation_ / y_distance_normalized;

  magic_multipliers_ = std::make_pair(x_magic_multiplier, y_magic_multiplier);
}

std::pair<double, double> DeviationPlanes::estimateAngles(const Eigen::MatrixXf& plane)
{
  std::pair<double, double> distance_diffs = getDistanceDiffs(plane);

  double px_estimation = magic_multipliers_.first * distance_diffs.first;
  double py_estimation = magic_multipliers_.second * distance_diffs.second;

  return std::make_pair(px_estimation, py_estimation);
}

std::pair<double, double> DeviationPlanes::getDistanceDiffs(const Eigen::MatrixXf& plane)
{
  std::vector<double> distances = getDistances(planes_, plane);

  double x_diff = distances[indexXNegative()] - distances[indexXPositive()];
  double y_diff = distances[indexYNegative()] - distances[indexYPositive()];

  double x_diff_normalized = x_diff / plane.cols();
  double y_diff_normalized = y_diff / plane.rows();

  return std::make_pair(x_diff_normalized, y_diff_normalized);
}

std::vector<double> DeviationPlanes::getDistances(const std::vector<Eigen::MatrixXf>& from, const Eigen::MatrixXf& to)
{
  std::vector<double> distances;
  for (int i = 0; i < from.size(); ++i)
  {
    distances.push_back(getDistance(from[i], to));
  }
  return distances;
}

double DeviationPlanes::getDistance(const Eigen::MatrixXf& from, const Eigen::MatrixXf& to)
{
  Eigen::MatrixXf difference = (to - from).cwiseAbs2();

  difference = difference.unaryExpr([](double v)
  { return std::isnan(v) ? 0.0 : v;});

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
