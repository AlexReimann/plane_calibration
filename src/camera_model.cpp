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
  parameters_ = Parameters(center_x, center_y, f_x, f_y, width, height);
  initialized_ = true;
}

CameraModel::CameraModel(const CameraModel &object)
{
  std::lock_guard<std::mutex> lock(object.mutex_);
  initialized_ = object.initialized_;
  parameters_ = object.parameters_;
}

bool CameraModel::initialized() const
{
  return initialized_;
}

void CameraModel::update(const double& center_x, const double& center_y, const double& f_x, const double& f_y,
                         const int& width, const int& height)
{
  std::lock_guard<std::mutex> lock(mutex_);

  parameters_.center_x_ = center_x;
  parameters_.center_y_ = center_y;
  parameters_.f_x_ = f_x;
  parameters_.f_y_ = f_y;
  parameters_.width_ = width;
  parameters_.height_ = height;

  initialized_ = true;
}

void CameraModel::update(const Parameters& parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);

  parameters_ = parameters;
  initialized_ = true;
}

CameraModel::Parameters CameraModel::getParameters() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return parameters_;
}

} /* end namespace */
