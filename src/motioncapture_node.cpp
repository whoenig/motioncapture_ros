#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud.h>

// Motion Capture
#ifdef ENABLE_VICON
#include <libmotioncapture/vicon.h>
#endif
#ifdef ENABLE_OPTITRACK
#include <libmotioncapture/optitrack.h>
#endif
#ifdef ENABLE_PHASESPACE
#include <libmotioncapture/phasespace.h>
#endif
#ifdef ENABLE_QUALISYS
#include <libmotioncapture/qualisys.h>
#endif
#ifdef ENABLE_VRPN
#include <libmotioncapture/vrpn.h>
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motioncapture_node");

  ros::NodeHandle nl("~");

  std::string motionCaptureType;
  nl.param<std::string>("motion_capture_type", motionCaptureType, "vicon");

  bool publish_pointcloud;
  nl.param<bool>("publish_pointcloud", publish_pointcloud, true);
  bool broadcast_tf;
  nl.param<bool>("broadcast_tf", broadcast_tf, true);

  // Make a new client
  libmotioncapture::MotionCapture* mocap = nullptr;
  if (false)
  {
  }
#ifdef ENABLE_VICON
  else if (motionCaptureType == "vicon")
  {
    std::string hostName;
    nl.getParam("vicon_host_name", hostName);
    mocap = new libmotioncapture::MotionCaptureVicon(hostName,
      /*enableObjects*/ true,
      /*enablePointcloud*/ true);
  }
#endif
#ifdef ENABLE_OPTITRACK
  else if (motionCaptureType == "optitrack")
  {
    std::string hostName;
    nl.getParam("optitrack_host_name", hostName);
    mocap = new libmotioncapture::MotionCaptureOptitrack(hostName);
  }
#endif
#ifdef ENABLE_PHASESPACE
  else if (motionCaptureType == "phasespace")
  {
    std::string ip;
    int numMarkers;
    nl.getParam("phasespace_ip", ip);
    nl.getParam("phasespace_num_markers", numMarkers);
    std::map<size_t, std::pair<int, int> > cfs;
    cfs[231] = std::make_pair<int, int>(10, 11);
    mocap = new libmotioncapture::MotionCapturePhasespace(ip, numMarkers, cfs);
  }
#endif
#ifdef ENABLE_QUALISYS
  else if (motionCaptureType == "qualisys")
  {
    std::string hostname;
    int port;
    nl.getParam("qualisys_host_name", hostname);
    nl.getParam("qualisys_base_port", port);
    mocap = new libmotioncapture::MotionCaptureQualisys(hostname, port,
      /*enableObjects*/ true,
      /*enablePointcloud*/ true);
  }
#endif
#ifdef ENABLE_VRPN
  else if (motionCaptureType == "vrpn")
  {
    std::string hostname;
    int port;
    nl.getParam("vrpn_host_name", hostname);
    mocap = new libmotioncapture::MotionCaptureVrpn(hostname);
  }
#endif
  else {
    throw std::runtime_error("Unknown motion capture type!");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<libmotioncapture::Object> objects;
  tf::TransformBroadcaster br;

  ros::Publisher pubPointCloud;
  sensor_msgs::PointCloud msgPointCloud;

  if (publish_pointcloud) {
    if (mocap->supportsPointCloud()) {
      pubPointCloud = nl.advertise<sensor_msgs::PointCloud>("pointCloud", 1);
      msgPointCloud.header.seq = 0;
      msgPointCloud.header.frame_id = "world";
    } else {
      ROS_WARN("Motion capture backend does not support point clouds!");
      publish_pointcloud = false;
    }
  }
  if (broadcast_tf && !mocap->supportsObjectTracking()) {
    ROS_WARN("Motion capture backend does not support objects!");
    broadcast_tf = false;
  }

  while (ros::ok()) {

    // Get a frame
    mocap->waitForNextFrame();

    if (broadcast_tf) {
      mocap->getObjects(objects);

      for (auto const& object: objects) {
        if (!object.occluded()) {
          Eigen::Vector3f position = object.position();
          Eigen::Quaternionf rotation = object.rotation();

          tf::Quaternion tf_rotation(rotation.x(), rotation.y(), rotation.z(), rotation.w());
          tf::Vector3 tf_position(position(0), position(1), position(2));

          tf::Transform transform(tf_rotation, tf_position);
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", object.name()));
        }
      }
    }

    if (publish_pointcloud) {
      mocap->getPointCloud(markers);

      msgPointCloud.header.seq += 1;
      msgPointCloud.header.stamp = ros::Time::now();
      msgPointCloud.points.resize(markers->size());
      for (size_t i = 0; i < markers->size(); ++i) {
        const pcl::PointXYZ& point = markers->at(i);
        msgPointCloud.points[i].x = point.x;
        msgPointCloud.points[i].y = point.y;
        msgPointCloud.points[i].z = point.z;
      }
      pubPointCloud.publish(msgPointCloud);
    }

    ros::spinOnce();
  }

  // for (size_t frameId = 0; ros::ok(); ++frameId) {
  //   std::cout << "frame " << frameId << ":" << std::endl;
  //   // Get a frame
  //   mocap->waitForNextFrame();
  //   mocap->getPointCloud(markers);

  //   std::cout << "    points:" << std::endl;

  //   for (size_t i = 0; i < markers->size(); ++i) {
  //     const pcl::PointXYZ& point = markers->at(i);
  //     std::cout << "      \"" << i << "\": [" << point.x << "," << point.y << "," << point.z << "]" << std::endl;
  //   }

  //   if (mocap->supportsObjectTracking()) {
  //     mocap->getObjects(objects);

  //     std::cout << "    objects:" << std::endl;

  //     for (auto const& object: objects) {
  //       std::cout << "      \"" << object.name() << "\":" << std::endl;
  //       std::cout << "         occluded: " << object.occluded() << std::endl;

  //       if (object.occluded() == false) {
  //         Eigen::Vector3f position = object.position();
  //         Eigen::Quaternionf rotation = object.rotation();
  //         std::cout << "         position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
  //         std::cout << "         rotation: [" << rotation.w() << ", " << rotation.vec()(0) << ", "
  //                                             << rotation.vec()(1) << ", " << rotation.vec()(2) << "]" << std::endl;
  //       }
  //     }
  //   }


  //   ros::spinOnce();
  // }

  return 0;
}
