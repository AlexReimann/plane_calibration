#ifndef plane_calibration_SRC_PLANE_CALIBRATION_NODELET_HPP_
#define plane_calibration_SRC_PLANE_CALIBRATION_NODELET_HPP_

#include <atomic>

#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <plane_calibration/PlaneCalibrationConfig.h>
#include <sensor_msgs/Image.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

namespace plane_calibration
{

class PlaneCalibrationNodelet : public nodelet::Nodelet
{
public:
  PlaneCalibrationNodelet();

  virtual void onInit();

protected:
  virtual void reconfigureCB(PlaneCalibrationConfig &config, uint32_t level);
  virtual void depthImageCB(const sensor_msgs::ImageConstPtr& depth_image_msg);

  std::shared_ptr<dynamic_reconfigure::Server<PlaneCalibrationConfig> > reconfigure_server_;

  std::atomic<bool> debug_;

  ros::Subscriber sub_depth_image_;
  ros::Publisher pub_transform_;
};

} /* end namespace */

#endif
