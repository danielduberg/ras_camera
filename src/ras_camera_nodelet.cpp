#include <pluginlib/class_list_macros.h>

#include <ras_camera/ras_camera_nodelet.h>

PLUGINLIB_EXPORT_CLASS(ras_camera::CameraNodelet, nodelet::Nodelet)

namespace ras_camera
{

void CameraNodelet::onInit()
{
  ros::NodeHandle& nh = getMTNodeHandle();
  ros::NodeHandle& nh_priv = getMTPrivateNodeHandle();
}
}
