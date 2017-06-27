#ifndef plane_calibration_SRC_PLANES_HPP_
#define plane_calibration_SRC_PLANES_HPP_

#include <memory>
#include <map>
#include <utility>
#include <Eigen/Dense>

#include "calibration_parameters.hpp"
#include "plane_to_depth_image.hpp"

namespace plane_calibration
{
typedef std::shared_ptr<Eigen::MatrixXf> MatrixPlanePtr;

class Planes
{
public:
  Planes(const int& count, const CalibrationParameters::Parameters& parameters,
         const PlaneToDepthImage& plane_to_depth);

  std::pair<MatrixPlanePtr, MatrixPlanePtr> getFittingXTiltPlanes(const double& angle, const double& deviation);
  std::pair<MatrixPlanePtr, MatrixPlanePtr> getFittingYTiltPlanes(const double& angle, const double& deviation);

protected:
  void makePlanes();
  Eigen::MatrixXf makePlane(const Eigen::Vector2d& angles);

  void addPlanePairs(const double& angle);
  void addPlanes(const double& angle);

  std::pair<MatrixPlanePtr, MatrixPlanePtr> getFittingTiltPlanes(const std::map<double, MatrixPlanePtr>& planes,
                                                                 const double& angle, const double& deviation);
  std::pair<double, double> getDeviationPlaneKeys(const double& angle, const double& deviation);

  int pair_count_;
  double max_deviation_;
  PlaneToDepthImage plane_to_depth_;

  Eigen::Translation3d translation_;
  Eigen::AngleAxisd base_rotation_;

  std::map<double, MatrixPlanePtr> x_planes_;
  std::map<double, MatrixPlanePtr> y_planes_;
};
typedef std::shared_ptr<Planes> PlanesPtr;

} /* end namespace */

#endif
