#ifndef plane_calibration_SRC_CAMERA_MODEL_HPP_
#define plane_calibration_SRC_CAMERA_MODEL_HPP_

#include <mutex>

namespace plane_calibration
{

class CameraModel
{
public:
  class Parameters
  {
  public:
    Parameters()
    {
      center_x_ = 0.0;
      center_y_ = 0.0;
      f_x_ = 0.0;
      f_y_ = 0.0;
      width_ = 0.0;
      height_ = 0.0;
    }

    Parameters(const double& center_x, const double& center_y, const double& f_x, const double& f_y, const int& width,
               const int& height)
    {
      center_x_ = center_x;
      center_y_ = center_y;
      f_x_ = f_x;
      f_y_ = f_y;
      width_ = width;
      height_ = height;
    }

    double center_x_;
    double center_y_;
    double f_x_;
    double f_y_;
    int width_;
    int height_;
  };

  CameraModel();
  CameraModel(const double& center_x, const double& center_y, const double& f_x, const double& f_y, const int& width,
              const int& height);
  CameraModel(const CameraModel &object);

  bool initialized() const;
  void update(const double& center_x, const double& center_y, const double& f_x, const double& f_y, const int& width,
              const int& height);
  void update(const Parameters& parameters);
  Parameters getValues() const;

protected:

  mutable std::mutex mutex_;
  bool initialized_;
  Parameters values_;
};

} /* end namespace */

#endif
