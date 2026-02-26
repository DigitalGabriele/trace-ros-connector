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

// #include "ros_msgs_autocoder/action_client_factory.h"
// #include "action_client.h"

#include "example_msgs/action/example_action.hpp" // Updated path for ROS2
#include "example_ros_connector/example_action_translator.h" // Ensure this is also converted

#include <stdexcept>

namespace trace {
namespace ros_msgs_autocoder {

std::shared_ptr<trace::ServiceTaskInterface>
CreateActionClientByType(const std::string &type, const std::string &action_topic) {
  if (type == "example_msgs/action/ExampleAction") {
    return std::make_shared<ActionClient<example_msgs::action::ExampleAction, ExampleActionTranslator>>(action_topic);
  }
  throw std::out_of_range("We are in action_client_factory.cc: Unable to instantiate a client for action (" + type + ").");
}

} // namespace ros_msgs_autocoder
} // namespace trace
