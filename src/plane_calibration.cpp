#include "plane_calibration/plane_calibration.hpp"

#include <iostream>
#include "plane_calibration/plane_to_depth_image.hpp"

namespace plane_calibration
{

PlaneCalibration::PlaneCalibration()
{
  update_max_deviation_planes_ = true;
}

PlaneCalibration::PlaneCalibration(const PlaneCalibration& object)
{
  std::lock_guard<std::mutex> lock(object.mutex_);

  camera_model_.update(object.camera_model_.getParameters());
  parameters_ = object.parameters_;

  update_max_deviation_planes_ = object.update_max_deviation_planes_;
  max_deviation_planes_images_ = object.max_deviation_planes_images_;

}

void PlaneCalibration::updateParameters(const CameraModel& camera_model)
{
  std::lock_guard<std::mutex> lock(mutex_);
  camera_model_.update(camera_model.getParameters());
  update_max_deviation_planes_ = true;
}

void PlaneCalibration::updateParameters(const Parameters& parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  parameters_ = parameters;
  update_max_deviation_planes_ = true;
}

PlaneCalibration::Parameters PlaneCalibration::getParameters() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return parameters_;
}

bool PlaneCalibration::updateMaxDeviationPlanesIfNeeded()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!update_max_deviation_planes_)
  {
    return false;
  }

  updateMaxDeviationPlanesImages_(parameters_.rotation_);
  return true;
}

void PlaneCalibration::updateMaxDeviationPlanesImages_(const Eigen::AngleAxisd& rotation)
{
  if (!camera_model_.initialized())
  {
    return;
  }

  max_deviation_planes_images_.clear();
  std::vector<Eigen::Affine3d> transforms = getMaxDeviationTransforms_(rotation);

  for (int i = 0; i < transforms.size(); ++i)
  {
    PlaneWithTransform plane;
    plane.transform = transforms[i];
    plane.plane = PlaneToDepthImage::convert(transforms[i], camera_model_.getParameters());

    max_deviation_planes_images_.push_back(plane);
  }

  update_max_deviation_planes_ = false;
}

std::vector<Eigen::Affine3d> PlaneCalibration::getMaxDeviationTransforms_(const Eigen::AngleAxisd& rotation)
{
  double max_deviation = parameters_.max_deviation_;
  Eigen::Translation3d translation = Eigen::Translation3d(parameters_.ground_plane_offset_);

  Eigen::AngleAxisd x_tilt_positive(max_deviation, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd y_tilt_positive(max_deviation, Eigen::Vector3d::UnitY());

  Eigen::AngleAxisd x_tilt_negative(-max_deviation, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd y_tilt_negative(-max_deviation, Eigen::Vector3d::UnitY());

  std::vector<Eigen::Affine3d> tilt_transforms;
  tilt_transforms.push_back(translation * rotation * x_tilt_positive);
  tilt_transforms.push_back(translation * rotation * y_tilt_positive);

  tilt_transforms.push_back(translation * rotation * x_tilt_negative);
  tilt_transforms.push_back(translation * rotation * y_tilt_negative);

  return tilt_transforms;
}

PlaneCalibration::PlanesWithTransforms PlaneCalibration::getDeviationPlanes() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return max_deviation_planes_images_;
}

PlaneCalibration::PlanesWithTransforms PlaneCalibration::getDeviationPlanes(Eigen::AngleAxisd rotation)
{
  std::lock_guard<std::mutex> lock(mutex_);

  updateMaxDeviationPlanesImages_(rotation);
  PlaneCalibration::PlanesWithTransforms max_deviation_planes_images = max_deviation_planes_images_;
  updateMaxDeviationPlanesImages_(parameters_.rotation_);
  return max_deviation_planes_images;
}

Eigen::AngleAxisd PlaneCalibration::estimateRotation(const Eigen::MatrixXf& plane, const double& x_multiplier,
                                                     const double& y_multiplier, const int& iterations,
                                                     const double& step_size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  double px_estimation = 0.0;
  double py_estimation = 0.0;

  Eigen::AngleAxisd rotation = parameters_.rotation_;

  for (int i = 0; i < iterations; ++i)
  {
    auto estimation = estimateAngles_(plane, x_multiplier, y_multiplier);
    px_estimation = estimation.first;
    py_estimation = estimation.second;

    rotation = rotation * Eigen::AngleAxisd(px_estimation * step_size, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(py_estimation * step_size, Eigen::Vector3d::UnitY());
    updateMaxDeviationPlanesImages_(rotation);
  }

  updateMaxDeviationPlanesImages_(parameters_.rotation_);
  return rotation;
}

std::pair<double, double> PlaneCalibration::estimateAngles(const Eigen::MatrixXf& plane, const double& x_multiplier,
                                                           const double& y_multiplier)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return estimateAngles_(plane, x_multiplier, y_multiplier);
}

std::pair<double, double> PlaneCalibration::estimateAngles_(const Eigen::MatrixXf& plane, const double& x_multiplier,
                                                            const double& y_multiplier)
{
  auto distances_diffs = getXYDistanceDiff_(plane);

  std::cout << "diffs: " << distances_diffs.first << ", " << distances_diffs.second << std::endl;

  double px_estimation = x_multiplier * distances_diffs.first;
  double py_estimation = y_multiplier * distances_diffs.second;

  return std::make_pair(px_estimation, py_estimation);
}

std::pair<double, double> PlaneCalibration::getXYDistanceDiff(const Eigen::MatrixXf& plane) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return getXYDistanceDiff_(plane);
}

std::pair<double, double> PlaneCalibration::getXYDistanceDiff_(const Eigen::MatrixXf& plane) const
{
  auto distances = getDistancesToMaxDeviations_(plane);

  double x_diff = distances[2] - distances[0];
  double y_diff = distances[3] - distances[1];

  return std::make_pair(x_diff, y_diff);
}

std::vector<double> PlaneCalibration::getDistancesToMaxDeviations(const Eigen::MatrixXf& plane) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return getDistancesToMaxDeviations_(plane);
}

std::vector<double> PlaneCalibration::getDistancesToMaxDeviations_(const Eigen::MatrixXf& plane) const
{
  std::vector<double> distances;

  for (int i = 0; i < max_deviation_planes_images_.size(); ++i)
  {
    Eigen::MatrixXf deviation_plane = max_deviation_planes_images_[i].plane;
    Eigen::MatrixXf difference = (plane - deviation_plane).cwiseAbs2();

    difference = difference.unaryExpr([](double v)
    { return std::isnan(v) ? 0.0 : v;});

    double distance = difference.sum();

    //TODO refactor because numbers are bad
    //kind of normalization, not perfect though
    if (i == 0 || i == 2)
    {
      distance = distance / plane.rows();
    }
    else
    {
      distance = distance / plane.cols();
    }

    distances.push_back(distance);
  }

  return distances;
}

} /* end namespace */
