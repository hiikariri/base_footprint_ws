#ifndef PHASE_PUBLISHER_HPP
#define PHASE_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "biped_interfaces/msg/phase.hpp"

class PhasePublisher : public rclcpp::Node {
public:
  PhasePublisher();

private:
  void publishMessage();

  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // PHASE_PUBLISHER_HPP
