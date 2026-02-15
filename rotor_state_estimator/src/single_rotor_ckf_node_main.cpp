#include "rotor_state_estimator/node/single_rotor_ckf_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SingleRotorCkfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
