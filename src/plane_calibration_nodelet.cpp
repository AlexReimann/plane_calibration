#include "../include/plane_calibration/plane_calibration_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

namespace plane_calibration
{
  void PlaneCalibrationNodelet::onInit()
  {
      ROS_INFO("[PlaneCalibrationNodelet]: Initializing");

      ROS_INFO("[PlaneCalibrationNodelet]: Initialization finished");
  }
} /* end namespace */

PLUGINLIB_EXPORT_CLASS(plane_calibration::PlaneCalibrationNodelet, nodelet::Nodelet)
