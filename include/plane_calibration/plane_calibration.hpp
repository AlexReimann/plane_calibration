#ifndef plane_calibration_SRC_PLANE_CALIBRATION_HPP_
#define plane_calibration_SRC_PLANE_CALIBRATION_HPP_

#include <mutex>
#include <vector>
#include <utility>
#include <memory>
#include <Eigen/Dense>

#include "camera_model.hpp"

namespace plane_calibration
{

class PlaneCalibration
{
public:
  class Parameters
  {
  public:
    Parameters()
    {
      max_deviation_ = 0.0;
      rotation_ = Eigen::AngleAxisd::Identity();
    }

    Parameters(const Eigen::Vector3d& ground_plane_offset, const double& max_deviation,
               const Eigen::AngleAxisd& rotation = Eigen::AngleAxisd::Identity())
    {
      ground_plane_offset_ = ground_plane_offset;
      max_deviation_ = max_deviation;
      rotation_ = rotation;
    }

    Eigen::Vector3d ground_plane_offset_;
    double max_deviation_;
    Eigen::AngleAxisd rotation_;
  };

  class PlaneWithTransform
  {
  public:
    Eigen::MatrixXf plane;
    Eigen::Affine3d transform;
  };
  typedef std::vector<PlaneWithTransform> PlanesWithTransforms;

  PlaneCalibration();

  PlaneCalibration(const PlaneCalibration& object);

  virtual ~PlaneCalibration()
  {
  }

  void updateParameters(const CameraModel& camera_model);
  void updateParameters(const Parameters& parameters);

  virtual bool updateMaxDeviationPlanesIfNeeded();
  virtual PlanesWithTransforms getDeviationPlanes() const;

  Eigen::AngleAxisd estimateRotation(const Eigen::MatrixXf& plane, const double& x_multiplier,
                                     const double& y_multiplier, const int& iterations, const double& step_size = 0.5);
  std::pair<double, double> estimateAngles(const Eigen::MatrixXf& plane, const double& x_multiplier,
                                           const double& y_multiplier);

  virtual std::pair<double, double> getXYDistanceDiff(const Eigen::MatrixXf& plane) const;
  virtual std::vector<double> getDistancesToMaxDeviations(const Eigen::MatrixXf& plane) const;

protected:
  virtual void updateMaxDeviationPlanesImages_(const Eigen::AngleAxisd& rotation = Eigen::AngleAxisd::Identity());
  virtual std::vector<Eigen::Affine3d> getMaxDeviationTransforms_(const Eigen::AngleAxisd& rotation);

  std::pair<double, double> estimateAngles_(const Eigen::MatrixXf& plane, const double& x_multiplier,
                                           const double& y_multiplier);
  virtual std::pair<double, double> getXYDistanceDiff_(const Eigen::MatrixXf& plane) const;
  virtual std::vector<double> getDistancesToMaxDeviations_(const Eigen::MatrixXf& plane) const;

  CameraModel camera_model_;
  mutable std::mutex mutex_;
  Parameters parameters_;

  bool update_max_deviation_planes_;
  std::vector<PlaneWithTransform> max_deviation_planes_images_;
};
typedef std::shared_ptr<PlaneCalibration> PlaneCalibrationPtr;

} /* end namespace */

#endif
