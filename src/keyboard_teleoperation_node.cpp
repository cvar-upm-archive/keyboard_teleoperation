// "Copyright [year] <Copyright Owner>"

#include "as2_core/core_functions.hpp"
#include "keyboard_teleoperation.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleoperation>();
  node->preset_loop_frequency(50);  // Node frequency for run and callbacks
  // Node with only callbacks
  //as2::spinLoop(node);
  // Node with run
  as2::spinLoop(node,std::bind(&KeyboardTeleoperation::run, node));

  rclcpp::shutdown();
  return 0;
}
