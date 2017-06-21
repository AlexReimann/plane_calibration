#ifndef plane_calibration_SRC_VISUALIZER_INTERFACE_HPP_
#define plane_calibration_SRC_VISUALIZER_INTERFACE_HPP_

#include <string>
#include <memory>
#include <Eigen/Dense>

#include "camera_model.hpp"

namespace plane_calibration
{

class VisualizerInterface
{
public:
  virtual ~VisualizerInterface()
  {

  }

  virtual void publishImage(const std::string& topic, const Eigen::MatrixXf& image_matrix, std::string frame_id =
                                std::string("")) =0;
  virtual void publishCloud(const std::string& topic, const Eigen::Affine3d& plane_transformation,
                            const CameraModel::Parameters& camera_model_paramaters,
                            std::string frame_id = std::string("")) =0;
  virtual void publishCloud(const std::string& topic, const Eigen::MatrixXf& image_matrix, std::string frame_id =
                                std::string("")) =0;

  virtual void publishDouble(const std::string& topic, const double& value) =0;

protected:

};
typedef std::shared_ptr<VisualizerInterface> VisualizerInterfacePtr;

}
/* end namespace */

#endif
