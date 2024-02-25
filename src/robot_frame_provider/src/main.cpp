#include "rclcpp/rclcpp.hpp"
#include "robot_frame_provider/frames_broadcaster.hpp"
#include "robot_frame_provider/phase_publisher.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto framesBroadcasterNode = std::make_shared<FramesBroadcaster>();
  auto phasePublisherNode = std::make_shared<PhasePublisher>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(framesBroadcasterNode);
  executor.add_node(phasePublisherNode);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
