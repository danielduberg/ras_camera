#ifndef RAS_CAMERA_NODELET_H
#define RAS_CAMERA_NODELET_H

#include <nodelet/nodelet.h>

#include <ros/ros.h>

namespace ras_camera
{

class CameraNodelet : public nodelet::Nodelet
{

private:
  virtual void onInit();
};
}

#endif // RAS_CAMERA_NODELET_H
