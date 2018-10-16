/**
 * Author: Daniel Duberg (dduberg@kth.se)
 * Date: 2018
 */

#ifndef RAS_CAMERA_PERCEPTION_H
#define RAS_CAMERA_PERCEPTION_H

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>

namespace ras_camera
{
class Perception
{
private:
  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_priv_;

  ros::Subscriber depth_registered_cloud_sub_;
  ros::Subscriber depth_cloud_sub_;

  image_transport::Subscriber depth_registered_image_it_sub_;
  image_transport::Subscriber depth_image_it_sub_;
  image_transport::Subscriber rgb_image_it_sub_;

  ros::Subscriber depth_registered_image_sub_;
  ros::Subscriber depth_image_sub_;
  ros::Subscriber rgb_image_sub_;

public:
  Perception(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  void
  depthRegisteredCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

  void depthCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

  void depthRegisteredImageCallback(const sensor_msgs::Image::ConstPtr& image);

  void depthImageCallback(const sensor_msgs::Image::ConstPtr& image);

  void rgbImageCallback(const sensor_msgs::Image::ConstPtr& image);
};
}  // namespace ras_camera

#endif  // RAS_CAMERA_PERCEPTION_H
