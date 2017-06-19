#ifndef plane_calibration_SRC_CALIBRATION_PARAMETERS_HPP_
#define plane_calibration_SRC_CALIBRATION_PARAMETERS_HPP_

#include <mutex>
#include <memory>
#include <Eigen/Dense>

namespace plane_calibration
{

class CalibrationParameters
{
public:
  class Parameters
  {
  public:
    Parameters()
    {
      ground_plane_offset_ = Eigen::Vector3d(0.0, 0.0, 0.0);
      rotation_ = Eigen::AngleAxisd::Identity();
      max_deviation_ = 0.0;
      deviation_ = 0.0;
    }

    Parameters(const double& max_deviation, const Eigen::Vector3d& ground_plane_offset,
               const Eigen::AngleAxisd& rotation = Eigen::AngleAxisd::Identity())
    {
      ground_plane_offset_ = ground_plane_offset;
      rotation_ = rotation;
      max_deviation_ = max_deviation;
      deviation_ = max_deviation_;
    }

    Eigen::Affine3d getTransform() const
    {
      return Eigen::Translation3d(ground_plane_offset_) * rotation_;
    }

    Eigen::Vector3d ground_plane_offset_;
    Eigen::AngleAxisd rotation_;

    double max_deviation_;
    double deviation_;
  };

  CalibrationParameters();
  CalibrationParameters(const double& max_deviation, const Eigen::Vector3d& ground_plane_offset,
                        const Eigen::AngleAxisd& rotation = Eigen::AngleAxisd::Identity());
  CalibrationParameters(const CalibrationParameters &object);

  bool getUpdatedParameters(Parameters& updated_parameters);
  Parameters getParameters();
  bool parametersUpdated();
  Eigen::Affine3d getTransform() const;

  void update(const Eigen::Vector3d& ground_plane_offset, const double& max_deviation,
              const Eigen::AngleAxisd& rotation);
  void update(const Eigen::Vector3d& ground_plane_offset, const Eigen::AngleAxisd& rotation);
  void update(const Parameters& parameters);

  void update(const Eigen::Vector3d& ground_plane_offset);
  void update(const Eigen::AngleAxisd& rotation);
  void update(const double& deviation);
  void updateDeviations(const double& value);

  void updateDeviation(const double& deviation);

protected:
  mutable std::mutex mutex_;
  bool updated_;
  Parameters parameters_;
};
typedef std::shared_ptr<CalibrationParameters> CalibrationParametersPtr;

} /* end namespace */

#endif
