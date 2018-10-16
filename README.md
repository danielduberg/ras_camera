# ras_camera

## Nodelet
http://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html

http://wiki.ros.org/nodelet/Tutorials

## image_transport
http://wiki.ros.org/image_transport/Tutorials

## std::shared_ptr
https://en.cppreference.com/w/cpp/memory/shared_ptr

## dynamic_reconfigure
http://wiki.ros.org/dynamic_reconfigure/Tutorials

## Intel Realsense SR300
https://www.mouser.com/pdfdocs/intel_realsense_camera_sr300.pdf

## Benchmark
Using top I measured the difference between running it as a node compared to a nodelet:

| Launch file        | CPU usage camera nodelet          | CPU usage ras_camera node  | Total CPU usage
| ------------- |:-------------:|:-------------:|:-------------:|
| roslaunch ras_camera ras_camera_nodelet.launch      | ~53% | 0% | ~53% |
| roslaunch ras_camera ras_camera_node.launch      | ~55%      |   ~18% | ~73% |
