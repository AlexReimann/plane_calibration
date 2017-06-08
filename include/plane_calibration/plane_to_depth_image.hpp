#ifndef plane_calibration_SRC_PLANE_TO_DEPTH_IMAGE_HPP_
#define plane_calibration_SRC_PLANE_TO_DEPTH_IMAGE_HPP_

#include <Eigen/Dense>
#include <utility>

#include "camera_model.hpp"

namespace plane_calibration
{

class PlaneToDepthImage
{
public:
  static Eigen::MatrixXf convert(const Eigen::Affine3d& plane_transformation,
                                 const CameraModel::Parameters& camera_model_paramaters);

  static std::pair<Eigen::MatrixXd, Eigen::MatrixXd> depthVectors(const CameraModel::Parameters& camera_model_paramaters);

protected:

};

} /* end namespace */

#endif
