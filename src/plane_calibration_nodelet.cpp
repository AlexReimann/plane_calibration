#include "../include/plane_calibration/plane_calibration_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

namespace plane_calibration
{
  PlaneCalibrationNodelet::PlaneCalibrationNodelet()
  {
    debug_ = false;
  }

  void PlaneCalibrationNodelet::onInit()
  {
      ROS_INFO("[PlaneCalibrationNodelet]: Initializing");

      reconfigure_server_ = std::shared_ptr<dynamic_reconfigure::Server<PlaneCalibrationConfig> >(
          new dynamic_reconfigure::Server<PlaneCalibrationConfig>());
      dynamic_reconfigure::Server<PlaneCalibrationConfig>::CallbackType reconfigure_cb = boost::bind(&PlaneCalibrationNodelet::reconfigureCB,
                                                                                                this, _1, _2);
      reconfigure_server_->setCallback(reconfigure_cb);

      ROS_INFO("[PlaneCalibrationNodelet]: Initialization finished");
  }

  void PlaneCalibrationNodelet::reconfigureCB(PlaneCalibrationConfig &config, uint32_t level)
  {
    debug_ = config.debug;
  }
} /* end namespace */

PLUGINLIB_EXPORT_CLASS(plane_calibration::PlaneCalibrationNodelet, nodelet::Nodelet)
