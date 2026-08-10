#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "whycode_ros/CWhycodeROSNode.h"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<whycode_ros2::CWhycodeROSNode>());
  rclcpp::shutdown();

  return 0;
}
