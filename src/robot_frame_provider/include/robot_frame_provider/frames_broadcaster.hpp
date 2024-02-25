#ifndef FRAMES_BROADCASTER_HPP
#define FRAMES_BROADCASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class FramesBroadcaster : public rclcpp::Node {
public:
  FramesBroadcaster();

private:
  struct Frame {
    std::string parent;
    std::string child;
    double x, y, z;
    double roll, pitch, yaw;
  };

  void getPose(Frame* frame);

  void broadcastTransforms();

  void broadcastTransform(const std::string& parent, const std::string& child,
                          double x, double y, double z, double roll, double pitch, double yaw);

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // FRAMES_BROADCASTER_HPP
