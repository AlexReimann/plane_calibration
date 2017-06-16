#ifndef plane_calibration_SRC_DEVIATION_PLANES_HPP_
#define plane_calibration_SRC_DEVIATION_PLANES_HPP_

#include <utility>
#include <memory>
#include <Eigen/Dense>

#include "calibration_parameters.hpp"
#include "plane_to_depth_image.hpp"
#include "depth_visualizer.hpp"

namespace plane_calibration
{

class DeviationPlanes
{
public:
  DeviationPlanes(PlaneToDepthImage plane_to_depth, int width, int height, std::shared_ptr<DepthVisualizer> depth_visualizer);

  void init(CalibrationParameters::Parameters parameters);
  void update(CalibrationParameters::Parameters parameters, bool use_max_deviation = false);

  std::pair<double, double> estimateAngles(const Eigen::MatrixXf& plane, bool debug = false);
  std::pair<double, double> getDistanceDiffs(const Eigen::MatrixXf& plane, bool debug = false);
  static double getDistance(const Eigen::MatrixXf& from, const Eigen::MatrixXf& to, bool remove_nans);

  double getDeviation();
  std::pair<double, double> getMultipliers();

  Eigen::MatrixXf xPositive();
  Eigen::MatrixXf xNegative();

  Eigen::MatrixXf yPositive();
  Eigen::MatrixXf yNegative();

  Eigen::Affine3d xPositiveTransform();
  Eigen::Affine3d xNegativeTransform();

  Eigen::Affine3d yPositiveTransform();
  Eigen::Affine3d yNegativeTransform();

protected:
  static std::vector<double> getDistances(const std::vector<Eigen::MatrixXf>& from, const Eigen::MatrixXf& to);

  int indexXPositive();
  int indexXNegative();
  int indexYPositive();
  int indexYNegative();

  std::shared_ptr<DepthVisualizer> depth_visualizer_;
  PlaneToDepthImage plane_to_depth_;
  double deviation_;
  double x_scaling_;
  double y_scaling_;

  std::vector<Eigen::MatrixXf> planes_;
  std::vector<Eigen::Affine3d> transform_;

  std::pair<double, double> magic_multipliers_;
};
typedef std::shared_ptr<DeviationPlanes> DeviationPlanesPtr;

} /* end namespace */

#endif
