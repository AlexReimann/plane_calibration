#include "plane_calibration/camera_model.hpp"

namespace plane_calibration
{
CameraModel::CameraModel()
{
  initialized_ = false;
}

CameraModel::CameraModel(const double& center_x, const double& center_y, const double& f_x, const double& f_y,
                         const int& width, const int& height)
{
  values_ = Parameters(center_x, center_y, f_x, f_y, width, height);
  initialized_ = true;
}

CameraModel::CameraModel(const CameraModel &object)
{
  std::lock_guard<std::mutex> lock(object.mutex_);
  initialized_ = object.initialized_;
  values_ = object.values_;
}

bool CameraModel::initialized() const
{
  return initialized_;
}

void CameraModel::update(const double& center_x, const double& center_y, const double& f_x, const double& f_y,
                         const int& width, const int& height)
{
  std::lock_guard<std::mutex> lock(mutex_);

  values_.center_x_ = center_x;
  values_.center_y_ = center_y;
  values_.f_x_ = f_x;
  values_.f_y_ = f_y;
  values_.width_ = width;
  values_.height_ = height;

  initialized_ = true;
}

CameraModel::Parameters CameraModel::getValues() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return values_;
}

} /* end namespace */
