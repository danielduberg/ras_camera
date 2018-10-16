/**
 * Author: Daniel Duberg (dduberg@kth.se)
 * Date: 2018
 */

#ifndef RAS_CAMERA_NODELET_H
#define RAS_CAMERA_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <ras_camera/perception.h>

namespace ras_camera
{
class CameraNodelet : public nodelet::Nodelet
{
private:
  std::shared_ptr<Perception> perception_;

public:
  virtual void onInit();
};
}  // namespace ras_camera

#endif  // RAS_CAMERA_NODELET_H
