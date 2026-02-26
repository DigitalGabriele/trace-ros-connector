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

#include "example_ros_connector/example_action_translator.h"
#include "trace/connector/ConnectorInterface.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stdexcept>

namespace trace {
namespace ros_msgs_autocoder {

example_msgs::action::ExampleAction::Goal ExampleActionTranslator::ToGoal(const PropertyMap& properties) {
    example_msgs::action::ExampleAction::Goal goal;
    try {
        goal.server_input = boost::lexical_cast<int32_t>(properties.at("server_input"));
    } catch (...) {
        throw std::out_of_range("Missing property (server_input) in mission model.");
    }
    return goal;
}

PropertyMap ExampleActionTranslator::FromResult(const example_msgs::action::ExampleAction::Result::SharedPtr &result) {
    PropertyMap properties;
    properties["server_output"] = std::to_string(result->server_output);
    return properties;
}

} // namespace ros_msgs_autocoder
} // namespace trace

