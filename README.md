# Why this package
We created this package for you such that you will have a better experience with the Intel Realsense SR300 camera running on the Intel NUC. As the default settings for the camera will use most of the CPU power on the NUC.

Below you can read what you can do with this package and how to get it running. We have also provided some code, in the `ras_camera/src` folder, such that you can use nodelets instead for the camera to increase performance. If you take a look in the `ras_camera/src` you will see a good way of writing your code such that it is easy to switch from node to nodelet.

## Installation
### Option #1
Using [wstool](http://wiki.ros.org/wstool) and rosinstall:
```bash
# Go to src in catkin workspace
cd ~/catkin_ws/src

# Create ras_camera.rosinstall file
echo "- git: {local-name: ras_project/ras_camera, uri: 'https://github.com/danielduberg/ras_camera.git', version: master}" >> ras_camera.rosinstall

# Merge ras_camera.rosinstall into your .rosinstall file
wstool merge ras_camera.rosinstall

# Clean up. Remove the ras_camera.rosinstall file since we do not need it anymore
rm ras_camera.rosinstall

# Update all repos in .rosinstall
# This will download ras_camera into ~/catkin_ws/src/ras_project/ras_camera
wstool update

# Go to root folder of catkin workspace
cd ~/catkin_ws

# Compile
catkin_make
```

### Option #2
Clone it from Github:
```bash
# Go to src/ras_project in catkin workspace (or just src if you want the package there)
cd ~/catkin_ws/src/ras_project

# Clone the repository
git clone https://github.com/danielduberg/ras_camera.git

# Go to root folder of catkin workspace
cd ~/catkin_ws

# Compile
catkin_make
```

## Use the Release build type when compile your code
In order to speed up your code (C++ code that is) as much as possible you can run this command:
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```
instead of
```bash
catkin_make
```
which you have done so far.

By using the `Release` build type you turn on most (all?) of the optimizations that the compiler can use when compiling your code. Note that using the `Release` build type you do not get all the output that you will get using a different build type. So if you are debugging something it might be useful to use the `Debug` build type instead:
```bash
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

## Usage
There are three launch files in this project that will be described next. You can find them in the `launch` folder.

### sr300_rgbd_nodelet.launch
This launch file will replace `sr300_nodelet_rgbd.launch` that is found in the realsense_camera package. So if you want to keep your setup exactly as it is you can simply just run `roslaunch ras_camera sr300_rgbd_nodelet.launch` instead of `roslaunch realsense_camera sr300_nodelet_rgbd.launch` and everything should be the same, except that it is easier for you to configure now. If you read `What you can configure to get the performance you want` below you can find out what you can configure when you are using this package.

To run you simply type:
```bash
roslaunch ras_camera sr300_rgbd_nodelet.launch
```

### ras_camera_nodelet.launch
This launch file will launch `sr300_rgbd_nodelet.launch` (so you should not launch it separately) together with another node called `ras_camera` under the same `nodelet manager` (see below for an explaination why nodelets are awesome!).

We strongly recommend using this launch file (if you are not already using nodelets, in the correct way). Since you can get a good performance boost by using nodelets compared to just normal nodes, especially when running things that are publishing a lot of data, which the camera is doing.

If you go to `ras_camera/src` you will find a file called `ras_camera_nodelet.cpp` this just simply initializes the node and creates a `ras_camera::Perception` object. So if you go to `ras_camera/src/ras_camera` you will find a file called `perception.cpp`, in here you can see the code for `ras_camera::Perception`.

You are free and very welcome to edit the `perception.cpp` file found in `ras_camera/src/ras_camera` such that it does what your perception system should do. You can also copy your already written code here. In the `perception.cpp` we have created a number of subscribers and written some callback functions just to get you started. We recommend you to read the comments in that file. The most important thing is that you do **not** subscribe to topics from the camera that you do not use. This is because the camera is doing some kind of "on demand" publishing, meaning that it only processes and publishes to a specific topic if there is a subscriber on that topic.

To run you simply type:
```bash
roslaunch ras_camera ras_camera_nodelet.launch
```

### ras_camera_node.launch
Same as the one above but ras_camera will be running as a node instead of a nodelet. This might be useful when you are debugging since it can be a bit trickier to do with nodelets. However, since it is running as a node it is a different process compared to the camera. This means that the camera nodes and the ras_camera node are not allowed to share memory and therefor when you subscribe to a camera topic in the ras_camera node it has to copy every message and it is a lot of data! Just run `rostopic bw /camera/depth_registered/points` after you have launched the camera and you will see how much bandwidth that single topic is using, it seems wasteful to copy all of these messages, right? Especially when you do not need to, becuase you can use nodelets.

If you go to `ras_camera/src` you will find a file called `ras_camera_node.cpp` this just simply initializes the node and creates a `ras_camera::Perception` object. So if you go to `ras_camera/src/ras_camera` you will find a file called `perception.cpp`, in here you can see the code for `ras_camera::Perception`.

You are free and very welcome to edit the `perception.cpp` file found in `ras_camera/src/ras_camera` such that it does what your perception system should do. You can also copy your already written code here. In the `perception.cpp` we have created a number of subscribers and written some callback functions just to get you started. We recommend you to read the comments in that file. The most important thing is that you do **not** subscribe to topics from the camera that you do not use. This is because the camera is doing some kind of "on demand" publishing, meaning that it only processes and publishes to a specific topic if there is a subscriber on that topic.

To run you simply type:
```bash
roslaunch ras_camera ras_camera_node.launch
```

## What you can configure to get the performance you want
### sr300_rgbd_nodelet.launch
If you open `ras_camera/launch/sr300_rgbd_nodelet.launch` you will see a number of `<arg>` in that file. You can change these to your liking. I would suggest leaving the `depth_fps` and `color_fps` at `10`, especially if you are using the `depth_registered` topics, since running only the camera will be very heavy for the NUC otherwise. You can see what values the arguments can take [here](https://www.mouser.com/pdfdocs/intel_realsense_camera_sr300.pdf) on page 16: 4.3 Camera Video Stream Modes.

In that file you also see that you can turn on/off different types of precessing and change the resolution.

### camera.yaml
You can also open `ras_camera/params/camera.yaml`. In there you will find a bunch of parameters that you can change. You can change the values of the parameters right in that file. These parameters can also be changed on the fly after you have started the camera by running the command `rosrun rqt_reconfigure rqt_reconfigure`. If you then select `camera` and then `driver` you will see all of these parameters, change them, and see the result right away. It is a good method of finding good values for these parameters. 

If you want to save the values for the parameters that you have set using `rqt_reconfigure` you have to press the save button in `rqt_reconfigure` and then overwrite the file `ras_camera/params/camera.yaml`. It is important that you overwrite the file `ras_camera/params/camera.yaml` with the new values. Because if you look into the file `ras_camera/launch/sr300_rgbd_nodelet.launch` you will see that the parameters from that specific file is loaded at the end of the launch file.

The parameters are explained in more detail [here](https://www.mouser.com/pdfdocs/intel_realsense_camera_sr300.pdf) from page 18 to 22: 6 Client Software.

The most interesting might be that you can control the range and range versus motion. Also white balancing, accuracy and things like that.

## Nodelet
Nodelets are awesome!!!

You can read why to use nodelets [here](http://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html)

[Here](http://wiki.ros.org/nodelet/Tutorials) you can find tutorials about nodelets.

### What you need to know when using nodelets
You have to have a nodelet manager. All the nodes that you want to be able to communicate with each other with zero copy between them has to be running other the same nodelet manager. All the nodes that are running under a manager are running in the same process and therefore can share memory between each other. You should not change a message after you have published it or recieved it, if you want to do that you should create a copy and change that. This is because you are sending out a pointer to a memory adress, so all subscribers in the same nodelet will use the exact same memory for that message. So if you alter the message in one node it will be altered for all nodes.

**I am _not_ suggesting that you should make all of your packages into nodelets. Only the one that is using the camera. This is because more things can go wrong when using nodelets, so you will spend more time debugging.**

### Pros of using nodelets compared to nodes
* Nodes that are under the same manager is running in the same process. This means that they can subscribe and publish to each other without having to copy the data.
* Your code goes super fast, VROM VROM!!!
* I will fill this in when I think of something.

### Cons of using nodelets compared to nodes
* It can be harder to debug your code, so it might be good to have a way to easily run your code as a nodelet or a node depending on the situation.
* If you are running the nodes on different computers it still has to copy the data from one computer to the other.
* You are not allowed to alter the messages that you get without creating a copy of the message. Then you can alter the copy. The reason for this is that you get a pointer to a memory adress where the message is stored, so if you change the message in that memory it will be change for all other nodes that are also subscribed to that topic.

## image_transport
You can read more about `image_transport`, that is being used in `ras_camera/src/ras_camera/perception.cpp`, [here](http://wiki.ros.org/image_transport/Tutorials).

## std::shared_ptr
You can read more about `std::shared_ptr`, that is being used in `ras_camera/src/ras_camera_nodelet.cpp`, [here](https://en.cppreference.com/w/cpp/memory/shared_ptr).

## dynamic_reconfigure
You can read more about `dynamic_reconfigure`, which is used to configure parameters on the fly, [here](http://wiki.ros.org/dynamic_reconfigure/Tutorials).

## Intel Realsense SR300
[Here](https://www.mouser.com/pdfdocs/intel_realsense_camera_sr300.pdf) is a document with information about the Intel Realsense SR300 camera.

## Benchmark
We did a quick test with `top` to see the difference between running it as a node compared to a nodelet:

| Launch file        | CPU usage camera nodelet          | CPU usage ras_camera node  | Total CPU usage
| ------------- |:-------------:|:-------------:|:-------------:|
| roslaunch ras_camera ras_camera_nodelet.launch      | ~53% | 0% | ~53% |
| roslaunch ras_camera ras_camera_node.launch      | ~55%      |   ~18% | ~73% |

It should be noted that this test is not that scientific since we ran it only once (for 5 minutes as both a node and nodelet) on a single computer. But it might give you some indication of the performance you can gain from using nodelets (in the correct way) instead of nodes. The computer that this test was performed on is also significantly more powerful than the NUCs you are using.
