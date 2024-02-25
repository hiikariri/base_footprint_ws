#include "robot_frame_provider/phase_publisher.hpp"

PhasePublisher::PhasePublisher() : Node("phase_publisher_node")
{
  publisher_ = create_publisher<biped_interfaces::msg::Phase>("walk_support_state", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                             std::bind(&PhasePublisher::publishMessage, this));
}

void PhasePublisher::publishMessage()
{
  auto message = biped_interfaces::msg::Phase();

  message.header.stamp = this->get_clock()->now();
  message.phase = biped_interfaces::msg::Phase::RIGHT_STANCE; // TODO: Make algorithm to determine the actual phase

  publisher_->publish(message);
}
