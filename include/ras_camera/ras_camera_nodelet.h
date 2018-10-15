#ifndef RAS_CAMERA_NODELET_H
#define RAS_CAMERA_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

namespace ras_camera
{

class CameraNodelet : public nodelet::Nodelet
{

private:
  ros::Subscriber depth_registered_sub_;

public:
  virtual void onInit();

private:
  void depthRegisteredCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
};
}

#endif // RAS_CAMERA_NODELET_H
