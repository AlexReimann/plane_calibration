#include "plane_calibration/plane_calibration.hpp"

#include "plane_calibration/plane_to_depth_image.hpp"

namespace plane_calibration
{

PlaneCalibration::PlaneCalibration()
{
  update_max_deviation_planes_ = true;
}

void PlaneCalibration::updateParameters(const CameraModel& camera_model)
{
  camera_model_.update(camera_model.getValues());
}

void PlaneCalibration::updateParameters(const Parameters& parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  parameters_ = parameters;
  update_max_deviation_planes_ = true;
}

bool PlaneCalibration::updateMaxDeviationPlanesIfNeeded()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!update_max_deviation_planes_)
    {
      return false;
    }
  }
  updateMaxDeviationPlanesImages();
  return true;
}

void PlaneCalibration::updateMaxDeviationPlanesImages()
{
  if (!camera_model_.initialized() || !update_max_deviation_planes_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  max_deviation_planes_images_.clear();
  std::vector<Eigen::Affine3d> transforms = getMaxDeviationTransforms();

  for (int i = 0; i < transforms.size(); ++i)
  {
    PlaneWithTransform plane;
    plane.transform = transforms[i];
    plane.plane = PlaneToDepthImage::convert(transforms[i], camera_model_.getValues());

    max_deviation_planes_images_.push_back(plane);
  }

  update_max_deviation_planes_ = false;
}

std::vector<Eigen::Affine3d> PlaneCalibration::getMaxDeviationTransforms()
{
  double max_deviation = parameters_.max_deviation_;
  Eigen::Translation3d translation = Eigen::Translation3d(parameters_.ground_plane_offset_);

  Eigen::AngleAxisd x_tilt_positive(max_deviation, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd y_tilt_positive(max_deviation, Eigen::Vector3d::UnitY());

  Eigen::AngleAxisd x_tilt_negative(-max_deviation, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd y_tilt_negative(-max_deviation, Eigen::Vector3d::UnitY());

  std::vector<Eigen::Affine3d> tilt_transforms;
  tilt_transforms.push_back(translation * x_tilt_positive);
  tilt_transforms.push_back(translation * y_tilt_positive);

  tilt_transforms.push_back(translation * x_tilt_negative);
  tilt_transforms.push_back(translation * y_tilt_negative);

  return tilt_transforms;
}

PlaneCalibration::PlanesWithTransforms PlaneCalibration::getDeviationPlanes()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return max_deviation_planes_images_;
}

} /* end namespace */
