#include "rotor_state_estimator/node/ckf_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CkfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}