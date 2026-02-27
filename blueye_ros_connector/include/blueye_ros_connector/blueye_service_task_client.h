#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "service_task_interface.h"
#include "blueye_interfaces/srv/run_waypoint_controller.hpp"
#include "blueye_interfaces/srv/add_waypoint.hpp"
#include "blueye_interfaces/srv/go_to_waypoints.hpp"
#include "blueye_interfaces/srv/clear_waypoints.hpp"
#include "blueye_interfaces/srv/get_waypoint_status.hpp"
#include "blueye_interfaces/srv/get_waypoints.hpp"
#include "blueye_interfaces/srv/insert_waypoint.hpp"
#include "blueye_interfaces/srv/remove_waypoint.hpp"

#include <atomic>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <chrono>

namespace trace {
namespace blueye_ros_connector {

class BlueyeServiceTaskClient : public ServiceTaskInterface, public rclcpp::Node
{
public:
  BlueyeServiceTaskClient();
  explicit BlueyeServiceTaskClient(const std::string &node_name);

  void Connect() override;
  std::future<trace::Outcome> SendCommand(
      const std::string &service_task_uuid,
      const PropertyMap &properties) override;

  void AbortCommand(const std::string &service_task_uuid) override;

  virtual ~BlueyeServiceTaskClient();

private:
  using RunWaypointController = blueye_interfaces::srv::RunWaypointController;
  using AddWaypoint = blueye_interfaces::srv::AddWaypoint;
  using GoToWaypoints = blueye_interfaces::srv::GoToWaypoints;
  using ClearWaypoints = blueye_interfaces::srv::ClearWaypoints;
  using GetWaypointStatus = blueye_interfaces::srv::GetWaypointStatus;
  using GetWaypoints = blueye_interfaces::srv::GetWaypoints;
  using InsertWaypoint = blueye_interfaces::srv::InsertWaypoint;
  using RemoveWaypoint = blueye_interfaces::srv::RemoveWaypoint;

  void start_executor_();
  void stop_executor_();

  rclcpp::Client<RunWaypointController>::SharedPtr run_wp_client_;
  rclcpp::Client<AddWaypoint>::SharedPtr add_waypoint_client_;
  rclcpp::Client<GoToWaypoints>::SharedPtr go_to_waypoints_client_;
  rclcpp::Client<ClearWaypoints>::SharedPtr clear_waypoints_client_;
  rclcpp::Client<GetWaypointStatus>::SharedPtr get_waypoint_status_client_;
  rclcpp::Client<GetWaypoints>::SharedPtr get_waypoints_client_;
  rclcpp::Client<InsertWaypoint>::SharedPtr insert_waypoint_client_;
  rclcpp::Client<RemoveWaypoint>::SharedPtr remove_waypoint_client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
  std::atomic<bool> executor_running_{false};
  std::mutex executor_mutex_;
  std::chrono::milliseconds service_wait_timeout_{std::chrono::seconds(5)};
  std::chrono::milliseconds service_response_timeout_{std::chrono::seconds(5)};

  trace::Outcome RunTask(
      const std::string &service_task_uuid,
      const PropertyMap &properties);
};

}  // namespace blueye_ros_connector
}  // namespace trace
