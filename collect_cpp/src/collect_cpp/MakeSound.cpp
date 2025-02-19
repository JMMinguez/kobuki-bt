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


#include "rclcpp/rclcpp.hpp"

#include "collect_cpp/MakeSound.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "kobuki_ros_interfaces/msg/sound.hpp"

namespace collect_cpp
{

MakeSound::MakeSound(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  sound_pub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Sound>("/commands/sound", 10);
}

BT::NodeStatus MakeSound::tick()
{
  auto sound_msg = kobuki_ros_interfaces::msg::Sound();
  sound_msg.value = kobuki_ros_interfaces::msg::Sound::ON;

  sound_pub_->publish(sound_msg);

  return BT::NodeStatus::SUCCESS;
}

void
MakeSound::halt()
{
}

}  // namespace collect_cpp

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<collect_cpp::MakeSound>("MakeSound");
}
