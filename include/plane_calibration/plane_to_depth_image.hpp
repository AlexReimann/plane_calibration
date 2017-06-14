#ifndef plane_calibration_SRC_PLANE_TO_DEPTH_IMAGE_HPP_
#define plane_calibration_SRC_PLANE_TO_DEPTH_IMAGE_HPP_

#include <Eigen/Dense>
#include <utility>
#include <sstream>

#include "camera_model.hpp"

namespace plane_calibration
{

class PlaneToDepthImage
{
public:
  class Errors
  {
  public:
    std::string asPrintString()
    {
      std::ostringstream string_stream;
      string_stream << mean << ", " << min << ", " << max;
      return string_stream.str();
    }

    double mean;
    double min;
    double max;
  };

  PlaneToDepthImage(const CameraModel::Parameters& camera_model_paramaters);
  Eigen::MatrixXf convert(const Eigen::Affine3d& plane_transformation);

  static Eigen::MatrixXf convert(const Eigen::Affine3d& plane_transformation,
                                 const CameraModel::Parameters& camera_model_paramaters);
  static Eigen::MatrixXf convert(const Eigen::Affine3d& plane_transformation,
                                 const CameraModel::Parameters& camera_model_paramaters,
                                 const std::pair<Eigen::MatrixXd, Eigen::MatrixXd>& xy_multipliers);

  static std::pair<Eigen::MatrixXd, Eigen::MatrixXd> depthCalculationXYMultiplier(
      const CameraModel::Parameters& camera_model_paramaters);

  static Errors getErrors(const Eigen::Affine3d& plane_transformation,
                          const CameraModel::Parameters& camera_model_paramaters, Eigen::MatrixXf image_matrix);

protected:
  CameraModel::Parameters camera_model_paramaters_;
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> xy_multipliers_;
};

} /* end namespace */

#endif
