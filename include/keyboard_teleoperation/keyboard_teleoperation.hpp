#ifndef KEYBOARD_TELEOPERATION_HPP_
#define KEYBOARD_TELEOPERATION_HPP_

#include <sstream>
#include <stdio.h>
#include <curses.h>
#include <iostream>
#include <string>
#include <math.h>
#include <tf2/utils.h>
#include <locale.h>

#include <rclcpp/rclcpp.hpp>
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#define ASCII_KEY_UP 65
#define ASCII_KEY_DOWN 66
#define ASCII_KEY_RIGHT 67
#define ASCII_KEY_LEFT 68

#define CTE_SPEED (1.00)
#define CTE_ALTITUDE (1.00)
#define CTE_YAW (0.1)

class KeyboardTeleoperation : public as2::Node {
 public:
  KeyboardTeleoperation();
  void run();
  void setupNode();
  void printPoseControls();
  void printSpeedControls();
  void takeOff();
  void hover();
  void land();
  void emergencyStop();
  void go();
  bool setControlMode(int new_control_mode);

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

 private:
  int SPEED = 1;
  int POSE = 2;
  int LEFT = 1;
  int RIGHT = 2;
  int UP = 3;
  int DOWN = 4;
  
  rclcpp::Subscription <geometry_msgs::msg::PoseStamped>::SharedPtr self_localization_pose_sub_;
  rclcpp::Subscription <geometry_msgs::msg::TwistStamped>::SharedPtr self_localization_speed_sub_;
  rclcpp::Subscription <as2_msgs::msg::PlatformInfo>::SharedPtr status_sub_;

  geometry_msgs::msg::PoseStamped self_localization_pose_;
  geometry_msgs::msg::TwistStamped self_localization_twist_;
  as2_msgs::msg::PlatformInfo platform_info_; 

  void poseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void twistCallback (const geometry_msgs::msg::TwistStamped::SharedPtr _msg); 
  void platformCallback (const as2_msgs::msg::PlatformInfo::SharedPtr _msg);

  int current_mode;
  char command = 0;
};

#endif 
