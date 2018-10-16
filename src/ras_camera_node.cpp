/**
 * Author: Daniel Duberg (dduberg@kth.se)
 * Date: 2018
 */

#include <ros/ros.h>

#include <ras_camera/perception.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ras_camera");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  ras_camera::Perception p(nh, nh_priv);

  ros::MultiThreadedSpinner spinner;  // Multiple threads
  spinner.spin();  // spin() will not return until the node has been shutdown

  // If you want to use one thread do this instead:
  // nh.spin();

  return 0;
}