// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COLLECT_CPP__NAVIGATE_ENTITY_HPP_
#define COLLECT_CPP__NAVIGATE_ENTITY_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "collect_cpp/PIDController.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace collect_cpp
{

class NavigateEntity : public BT::ActionNodeBase
{
public:
  NavigateEntity(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  void halt();

  void callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort("target")
      });
  }
  std::string target;

  double distancia, angulo;
  geometry_msgs::msg::Twist vel;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transform_sub_;
  PIDController lin_pid_, ang_pid_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace collect_cpp

#endif  // COLLECT_CPP__NAVIGATE_ENTITY_HPP_
