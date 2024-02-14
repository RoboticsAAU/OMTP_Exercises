#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "move_panda",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("move_panda");

  rclcpp::shutdown();
  return 0;
}
