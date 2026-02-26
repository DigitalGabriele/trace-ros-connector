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

// example-ros-connector-ros2

#pragma once

#include "blueye_ros_connector/service_task_interface.h"
#include <trace/connector/ConnectorInterface.hpp>
#include <trace/types/SeverityLevel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace trace {
namespace blueye_ros_connector {

class Connector : public ConnectorInterface, public rclcpp::Node {
public:

  Connector();

  Connector(const std::string &node_name);
//   Connector(const std::string &node_name)
//       : Node(node_name),
//         resource_to_client_map() {}

  // interface overrides
  void initialize() override;

  void destroy() override;

  bool check_resource(const std::string &resource_name,
                      const PropertyMap &properties) override;

  bool reserve_resource(
      const std::string &task_uuid,
      const std::unordered_map<std::string, std::string> &properties) override;

 std::shared_future<Outcome> command_resource(
      const std::string &task_uuid,
      const std::unordered_map<std::string, std::string> &properties) override;

  void abort_command(
      const std::string &task_uuid,
      const std::unordered_map<std::string, std::string> &properties) override;

  void release_resource(
     const std::string &task_uuid, 
     const PropertyMap &properties) override;

 void log(const SeverityLevel level, const std::string &function_name,
         const std::string &file_name, int line_number,
         const std::string &msg) override;


  std::chrono::milliseconds
  to_connector_time(const std::chrono::milliseconds &duration_ms) override;

  bool wait_for(std::condition_variable &timer_interrupt,
                std::unique_lock<std::mutex> &timer_lock,
                const std::chrono::milliseconds &duration_ms);

  bool wait_until(std::condition_variable &timer_interrupt,
                  std::unique_lock<std::mutex> &timer_lock,
                  const std::chrono::time_point<std::chrono::system_clock>
                      time_utc) override;

private:
  bool create_action_clients_from_xml();

  std::unordered_map<std::string, std::shared_ptr<::trace::ServiceTaskInterface>>
      resource_to_client_map;

  void timer_callback(std::shared_ptr<rclcpp::TimerBase> timer,
                    std::atomic<bool> *is_timeout,
                    std::condition_variable *timer_interrupt);
};

} // namespace blueye_ros_connector
} // namespace trace


