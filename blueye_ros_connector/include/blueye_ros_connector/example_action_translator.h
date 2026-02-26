/**
 * Copyright 2016-2025 California Institute of Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "rclcpp_action/rclcpp_action.hpp"
#include "example_msgs/action/example_action.hpp"

namespace trace {
namespace ros_msgs_autocoder {

typedef std::unordered_map<std::string, std::string> PropertyMap;

class ExampleActionTranslator {
public:
    explicit ExampleActionTranslator(const rclcpp::Node::SharedPtr& node)
        : node_(node) {}

    example_msgs::action::ExampleAction::Goal ToGoal(const PropertyMap &properties);
    PropertyMap FromResult(const example_msgs::action::ExampleAction::Result::SharedPtr &result);

private:
    rclcpp::Node::SharedPtr node_;
};

} // namespace ros_msgs_autocoder
} // namespace trace
