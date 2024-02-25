#include "rclcpp/rclcpp.hpp"
#include "biped_interfaces/msg/phase.hpp"

class PhasePublisher : public rclcpp::Node {
public:
  PhasePublisher()
    : Node("phase_publisher") {
    publisher_ = create_publisher<biped_interfaces::msg::Phase>("walk_support_state", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                               std::bind(&PhasePublisher::publishMessage, this));
  }

private:
  void publishMessage() {
    auto message = biped_interfaces::msg::Phase();

    message.header.stamp = now();
    message.phase = biped_interfaces::msg::Phase::LEFT_STANCE; // TODO: Make algorithm to determine the actual phase

    publisher_->publish(message);
  }

  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PhasePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
