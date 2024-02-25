#include "robot_frame_provider/frames_broadcaster.hpp"

FramesBroadcaster::FramesBroadcaster() : Node("frames_broadcaster_node")
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                             std::bind(&FramesBroadcaster::broadcastTransforms, this));
}

void FramesBroadcaster::getPose(Frame* frame)
{
  // TO DO: Get the current position and orientation of the frame
  frame->x = 1.0;
  frame->y = 0.0;
  frame->z = 0.0;
  frame->roll = 0.0;
  frame->pitch = 0.0;
  frame->yaw = 0.0;
}

void FramesBroadcaster::broadcastTransforms()
{
  std::vector<Frame> frames = {
    {"world", "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {"base_link", "r_sole", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {"base_link", "l_sole", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {"world", "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };

  for (Frame& frame : frames) {
    getPose(&frame);
    broadcastTransform(frame.parent, frame.child,
                       frame.x, frame.y, frame.z,
                       frame.roll, frame.pitch, frame.yaw);
  }
}

void FramesBroadcaster::broadcastTransform(const std::string& parent, const std::string& child,
                                             double x, double y, double z, double roll, double pitch, double yaw) 
{
  geometry_msgs::msg::TransformStamped transform;
  
  transform.header.stamp = now();
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();

  tf_broadcaster_->sendTransform(transform);
}
