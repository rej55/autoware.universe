// Copyright 2022 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

namespace behavior_path_planner
{

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(const std::string & name, rclcpp::Node & node)
  : name_{name},
    logger_{node.get_logger().get_child(name)},
    clock_{node.get_clock()}
  {
    std::string module_ns;
    module_ns.resize(name.size());
    std::transform(name.begin(), name.end(), module_ns.begin(), tolower);

    const auto ns = std::string("~/debug/") + module_ns;
    pub_debug_marker_ = node.create_publisher<MarkerArray>(ns, 20);
  }

private:
  std::string name_;
  rclcpp::Logger logger_;

protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;
  mutable AvoidanceDebugMsgArray::SharedPtr debug_avoidance_msg_array_ptr_{};
  mutable MarkerArray debug_marker_;
}

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
