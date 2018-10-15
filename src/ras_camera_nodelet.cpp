#include <pluginlib/class_list_macros.h>

#include <ras_camera/ras_camera_nodelet.h>

PLUGINLIB_EXPORT_CLASS(ras_camera::CameraNodelet, nodelet::Nodelet)

namespace ras_camera
{

void CameraNodelet::onInit()
{
  ros::NodeHandle& nh = getMTNodeHandle(); // Or use: getNodeHandle();
  ros::NodeHandle& nh_priv = getMTPrivateNodeHandle(); // Or use: getPrivateNodeHandle();

  depth_registered_sub_ = nh.subscribe("camera/depth_registered/points", 1,
                                 &CameraNodelet::depthRegisteredCallback, this);
}

void CameraNodelet::depthRegisteredCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
  // Do stuff
}
}
