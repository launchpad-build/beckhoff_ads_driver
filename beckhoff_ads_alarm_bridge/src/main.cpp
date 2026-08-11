#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "beckhoff_ads_alarm_bridge/alarm_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<beckhoff_ads_alarm_bridge::AlarmBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
