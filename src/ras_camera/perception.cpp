/**
 * Author: Daniel Duberg (dduberg@kth.se)
 * Date: 2018
 */

#include <ras_camera/perception.h>

namespace ras_camera
{
Perception::Perception(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh), nh_priv_(nh_priv)
{
  image_transport::ImageTransport it(
      nh);  // we use it to subscribe to images. Tutorials:
            // http://wiki.ros.org/image_transport/Tutorials

  // Comment/remove the subscribers that you do not which to use. Do _NOT_ subscribe
  // to anything from the camera that you are not using. The camera nodes are doing some
  // kind of "on demand" publishing, which means that it only publishes (and more
  // importantly, computes the data that should be published) on a topic if there is a
  // subscriber on that topic. Therefore, if you subscribe to a topic from the camera that
  // you are not using you are wasting CPU resources.

  // You can of course also add other subscribers and change the topics if you which to
  // subscribe to something else. Also change the queue size if you want to.

  // Subscribe to point clouds
  depth_registered_cloud_sub_ =
      nh.subscribe("/camera/depth_registered/points", 1,
                   &Perception::depthRegisteredCloudCallback, this);
  depth_cloud_sub_ =
      nh.subscribe("/camera/depth/points", 1, &Perception::depthCloudCallback, this);

  // Subscribe to depth images using image_transport
  depth_registered_image_it_sub_ =
      it.subscribe("/camera/depth_registered/sw_registered/image_rect", 1,
                   &Perception::depthRegisteredImageCallback, this);
  depth_image_it_sub_ =
      it.subscribe("/camera/depth/image_rect", 1, &Perception::depthImageCallback, this);

  // Subscribe to RGB images using image_transport
  rgb_image_it_sub_ = it.subscribe("/camera/rgb/image_rect_color", 1,
                                   &Perception::rgbImageCallback, this);

  // Subscribe to depth images with "normal" subscriber
  depth_registered_image_sub_ =
      nh.subscribe("/camera/depth_registered/sw_registered/image_rect", 1,
                   &Perception::depthRegisteredImageCallback, this);
  depth_image_sub_ =
      nh.subscribe("/camera/depth/image_rect", 1, &Perception::depthImageCallback, this);

  // Subscribe to RGB images with "normal" subscriber
  rgb_image_sub_ = nh.subscribe("/camera/rgb/image_rect_color", 1,
                                &Perception::rgbImageCallback, this);
}

void Perception::depthRegisteredCloudCallback(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
  // Do stuff
}

void Perception::depthCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  // Do stuff
}

void Perception::depthRegisteredImageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  // Do stuff
}

void Perception::depthImageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  // Do stuff
}

void Perception::rgbImageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  // Do stuff
}
}  // namespace ras_camera
