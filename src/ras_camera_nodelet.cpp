/**
 * Author: Daniel Duberg (dduberg@kth.se)
 * Date: 2018
 */

#include <pluginlib/class_list_macros.h>

#include <ras_camera/ras_camera_nodelet.h>

PLUGINLIB_EXPORT_CLASS(ras_camera::CameraNodelet, nodelet::Nodelet)

namespace ras_camera
{
// When you are using nodelets (tutorials: http://wiki.ros.org/nodelet/Tutorials) onInit()
// is the first function that is being called that you can control, it is sort of like the
// main function or the constructor. You have to let this function exit, so you should not
// have any spin() or loops that prevents the function for exiting. Spin is handled for
// your under the hood at a certain frequency.
void CameraNodelet::onInit()
{
  ros::NodeHandle& nh = getMTNodeHandle();              // Or use: getNodeHandle();
  ros::NodeHandle& nh_priv = getMTPrivateNodeHandle();  // Or use: getPrivateNodeHandle();

  perception_ = std::make_shared<Perception>(
      nh,
      nh_priv);  // Use a shared pointer so we do not have to manually delete it. Now
                 // it will be deleted when the pointer count is zero. Info:
                 // https://en.cppreference.com/w/cpp/memory/shared_ptr
                 // First available in C++11. Before that you could use boost::shared_ptr,
                 // there are some difference between the two.
}
}  // namespace ras_camera
