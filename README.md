# motioncapture_ros
ROS Package that supports several motion capture systems, including Vicon, Qualisys, Optitrack, and VRPN.

This package is a slim ROS package around libmotioncapture, a C++ library that unifies different mocap SDKs.

Objects are broadcasted using tf, unlabeled markers (if supported) as pointcloud.

## Usage

```
cd catkin_ws/src
git clone --recursive https://github.com/whoenig/motioncapture_ros.git
cd ..
catkin_make
roslaunch motioncapture motioncapture.launch
```

See motioncapture.launch for configuration options.

