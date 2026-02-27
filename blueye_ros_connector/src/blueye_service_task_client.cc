#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <chrono>
#include <mutex>
#include <cmath>
#include <cstdint>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "blueye_interfaces/srv/run_waypoint_controller.hpp"
#include "blueye_ros_connector/blueye_service_task_client.h"

namespace trace {
namespace blueye_ros_connector {

using RunWaypointController = blueye_interfaces::srv::RunWaypointController;
using AddWaypoint = blueye_interfaces::srv::AddWaypoint;
using GoToWaypoints = blueye_interfaces::srv::GoToWaypoints;
using ClearWaypoints = blueye_interfaces::srv::ClearWaypoints;
using GetWaypointStatus = blueye_interfaces::srv::GetWaypointStatus;
using GetWaypoints = blueye_interfaces::srv::GetWaypoints;
using InsertWaypoint = blueye_interfaces::srv::InsertWaypoint;
using RemoveWaypoint = blueye_interfaces::srv::RemoveWaypoint;

namespace {

std::string trim(const std::string &s)
{
  const std::string whitespace = " \n\r\t";
  const auto start = s.find_first_not_of(whitespace);
  if (start == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(whitespace);
  return s.substr(start, end - start + 1);
}

std::string bool_to_string(bool value)
{
  return value ? "true" : "false";
}

bool parse_bool(const std::string &raw, bool &value)
{
  const std::string v = trim(raw);
  if (v == "true" || v == "1" || v == "True" || v == "TRUE") {
    value = true;
    return true;
  }
  if (v == "false" || v == "0" || v == "False" || v == "FALSE") {
    value = false;
    return true;
  }
  return false;
}

bool parse_float(const std::string &raw, float &value)
{
  const std::string v = trim(raw);
  if (v.empty()) {
    return false;
  }
  try {
    std::size_t idx = 0;
    const float parsed = std::stof(v, &idx);
    if (idx != v.size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_int32(const std::string &raw, std::int32_t &value)
{
  const std::string v = trim(raw);
  if (v.empty()) {
    return false;
  }
  try {
    std::size_t idx = 0;
    const long parsed = std::stol(v, &idx, 10);
    if (idx != v.size()) {
      return false;
    }
    if (parsed < std::numeric_limits<std::int32_t>::min() ||
        parsed > std::numeric_limits<std::int32_t>::max()) {
      return false;
    }
    value = static_cast<std::int32_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

std::string float_to_string(float value)
{
  std::ostringstream ss;
  ss << value;
  return ss.str();
}

std::chrono::milliseconds seconds_to_duration(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return std::chrono::milliseconds(0);
  }
  return std::chrono::milliseconds(
    static_cast<std::chrono::milliseconds::rep>(seconds * 1000.0));
}

std::string int_to_string(std::int32_t value)
{
  return std::to_string(value);
}

}  // namespace

BlueyeServiceTaskClient::BlueyeServiceTaskClient()
: Node("blueye_service_task_client")
{
  // Create service client
  run_wp_client_ = this->create_client<RunWaypointController>("/blueye/run_waypoint_controller");
  add_waypoint_client_ = this->create_client<AddWaypoint>("/blueye/add_waypoint");
  go_to_waypoints_client_ = this->create_client<GoToWaypoints>("/blueye/go_to_waypoints");
  clear_waypoints_client_ = this->create_client<ClearWaypoints>("/blueye/clear_waypoints");
  get_waypoint_status_client_ = this->create_client<GetWaypointStatus>("/blueye/get_waypoint_status");
  get_waypoints_client_ = this->create_client<GetWaypoints>("/blueye/get_waypoints");
  insert_waypoint_client_ = this->create_client<InsertWaypoint>("/blueye/insert_waypoint");
  remove_waypoint_client_ = this->create_client<RemoveWaypoint>("/blueye/remove_waypoint");
  service_wait_timeout_ = seconds_to_duration(
      this->declare_parameter<double>("service_wait_timeout_s", 5.0));
  service_response_timeout_ = seconds_to_duration(
      this->declare_parameter<double>("service_response_timeout_s", 5.0));
  start_executor_();
}

BlueyeServiceTaskClient::BlueyeServiceTaskClient(const std::string &node_name)
: Node(node_name)
{
  run_wp_client_ = this->create_client<RunWaypointController>("/blueye/run_waypoint_controller");
  add_waypoint_client_ = this->create_client<AddWaypoint>("/blueye/add_waypoint");
  go_to_waypoints_client_ = this->create_client<GoToWaypoints>("/blueye/go_to_waypoints");
  clear_waypoints_client_ = this->create_client<ClearWaypoints>("/blueye/clear_waypoints");
  get_waypoint_status_client_ = this->create_client<GetWaypointStatus>("/blueye/get_waypoint_status");
  get_waypoints_client_ = this->create_client<GetWaypoints>("/blueye/get_waypoints");
  insert_waypoint_client_ = this->create_client<InsertWaypoint>("/blueye/insert_waypoint");
  remove_waypoint_client_ = this->create_client<RemoveWaypoint>("/blueye/remove_waypoint");
  service_wait_timeout_ = seconds_to_duration(
      this->declare_parameter<double>("service_wait_timeout_s", 5.0));
  service_response_timeout_ = seconds_to_duration(
      this->declare_parameter<double>("service_response_timeout_s", 5.0));
  start_executor_();
}

BlueyeServiceTaskClient::~BlueyeServiceTaskClient()
{
  stop_executor_();
}

void BlueyeServiceTaskClient::start_executor_()
{
  std::lock_guard<std::mutex> lock(executor_mutex_);
  if (executor_running_.load()) {
    return;
  }

  executor_.add_node(this->get_node_base_interface());
  executor_running_.store(true);
  executor_thread_ = std::thread([this]() {
    executor_.spin();
  });
}

void BlueyeServiceTaskClient::stop_executor_()
{
  std::lock_guard<std::mutex> lock(executor_mutex_);
  if (!executor_running_.load()) {
    return;
  }

  executor_running_.store(false);
  executor_.cancel();

  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  executor_.remove_node(this->get_node_base_interface());
}

void BlueyeServiceTaskClient::Connect()
{
  // Best effort connectivity checks.
  const bool run_available =
      run_wp_client_->wait_for_service(service_wait_timeout_);
  const bool add_wp_available =
      add_waypoint_client_->wait_for_service(service_wait_timeout_);
  const bool go_to_available =
      go_to_waypoints_client_->wait_for_service(service_wait_timeout_);
  const bool clear_wp_available =
      clear_waypoints_client_->wait_for_service(service_wait_timeout_);
  const bool get_wp_status_available =
      get_waypoint_status_client_->wait_for_service(service_wait_timeout_);
  const bool get_wps_available =
      get_waypoints_client_->wait_for_service(service_wait_timeout_);
  const bool insert_wp_available =
      insert_waypoint_client_->wait_for_service(service_wait_timeout_);
  const bool remove_wp_available =
      remove_waypoint_client_->wait_for_service(service_wait_timeout_);

  RCLCPP_INFO(this->get_logger(),
              "Service availability: /blueye/run_waypoint_controller=%d, /blueye/add_waypoint=%d, /blueye/go_to_waypoints=%d, /blueye/clear_waypoints=%d, /blueye/get_waypoint_status=%d, /blueye/get_waypoints=%d, /blueye/insert_waypoint=%d, /blueye/remove_waypoint=%d",
              static_cast<int>(run_available), static_cast<int>(add_wp_available),
              static_cast<int>(go_to_available), static_cast<int>(clear_wp_available),
              static_cast<int>(get_wp_status_available), static_cast<int>(get_wps_available),
              static_cast<int>(insert_wp_available), static_cast<int>(remove_wp_available));
}

std::future<trace::Outcome> BlueyeServiceTaskClient::SendCommand(
  const std::string &service_task_uuid,
  const PropertyMap &properties)
{
  return std::async(std::launch::async, &BlueyeServiceTaskClient::RunTask, this, service_task_uuid, properties);
}

void BlueyeServiceTaskClient::AbortCommand(const std::string &service_task_uuid)
{
  (void)service_task_uuid;
  // ROS2 services have no standard cancel. No-op.
}

trace::Outcome BlueyeServiceTaskClient::RunTask(
  const std::string &service_task_uuid,
  const PropertyMap &properties)
{
  (void)service_task_uuid;

  const auto resource_it = properties.find("resource_name");
  if (resource_it == properties.end()) {
    return trace::Outcome(StatusCode::ERROR, "Missing property: resource_name");
  }
  const std::string resource_name = trim(resource_it->second);

  // RUN WAYPOINT CONTROLLER SERVICE
  if (resource_name == "run_waypoint_controller") {
    if (!run_wp_client_->wait_for_service(service_wait_timeout_)) {
      return trace::Outcome(StatusCode::ERROR, "Service not available: /blueye/run_waypoint_controller");
    }

    // In BPMN you can set: <camunda:inputParameter name="run">true</camunda:inputParameter>
    bool run = true;
    auto it = properties.find("run");
    if (it != properties.end()) {
      const std::string trimmed = trim(it->second);
      if (!trimmed.empty() && !parse_bool(trimmed, run)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid boolean property: run");
      }
    }

    auto req = std::make_shared<RunWaypointController::Request>();
    req->run = run;
    auto future = run_wp_client_->async_send_request(req);

    const auto status = future.wait_for(service_response_timeout_);
    if (status != std::future_status::ready) {
      return trace::Outcome(StatusCode::ERROR, "Timeout waiting for /blueye/run_waypoint_controller response");
    }

    auto resp = future.get();
    if (!resp) {
      return trace::Outcome(StatusCode::ERROR, "Null response from /blueye/run_waypoint_controller");
    }

    trace::Outcome out(
      resp->accepted ? StatusCode::OK : StatusCode::ERROR,
      resp->accepted ? "Waypoint controller command accepted" : "Waypoint controller command rejected");

    out.add_result("accepted", bool_to_string(resp->accepted));
    out.add_result("run", bool_to_string(run));
    return out;
  }

  // GO TO WAYPOINTS SERVICE
  if (resource_name == "go_to_waypoints") {
    if (!go_to_waypoints_client_->wait_for_service(service_wait_timeout_)) {
      return trace::Outcome(StatusCode::ERROR, "Service not available: /blueye/go_to_waypoints");
    }

    bool run = true;
    auto it = properties.find("run");
    if (it != properties.end()) {
      const std::string trimmed = trim(it->second);
      if (!trimmed.empty() && !parse_bool(trimmed, run)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid boolean property: run");
      }
    }

    auto req = std::make_shared<GoToWaypoints::Request>();
    req->run = run;
    auto future = go_to_waypoints_client_->async_send_request(req);

    const auto status = future.wait_for(service_response_timeout_);
    if (status != std::future_status::ready) {
      return trace::Outcome(StatusCode::ERROR, "Timeout waiting for /blueye/go_to_waypoints response");
    }

    auto resp = future.get();
    if (!resp) {
      return trace::Outcome(StatusCode::ERROR, "Null response from /blueye/go_to_waypoints");
    }

    trace::Outcome out(
      resp->accepted ? StatusCode::OK : StatusCode::ERROR,
      resp->accepted ? "Go-to-waypoints accepted" : "Go-to-waypoints rejected");
    out.add_result("accepted", bool_to_string(resp->accepted));
    out.add_result("status_code", trim(resp->status_code));
    out.add_result("run", bool_to_string(run));
    return out;
  }

  // CLEAR WAYPOINTS SERVICE
  if (resource_name == "clear_waypoints") {
    if (!clear_waypoints_client_->wait_for_service(service_wait_timeout_)) {
      return trace::Outcome(StatusCode::ERROR, "Service not available: /blueye/clear_waypoints");
    }

    bool clear = true;
    auto it = properties.find("clear");
    if (it != properties.end()) {
      const std::string trimmed = trim(it->second);
      if (!trimmed.empty() && !parse_bool(trimmed, clear)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid boolean property: clear");
      }
    }

    auto req = std::make_shared<ClearWaypoints::Request>();
    req->clear = clear;
    auto future = clear_waypoints_client_->async_send_request(req);

    const auto status = future.wait_for(service_response_timeout_);
    if (status != std::future_status::ready) {
      return trace::Outcome(StatusCode::ERROR, "Timeout waiting for /blueye/clear_waypoints response");
    }

    auto resp = future.get();
    if (!resp) {
      return trace::Outcome(StatusCode::ERROR, "Null response from /blueye/clear_waypoints");
    }

    trace::Outcome out(
      resp->accepted ? StatusCode::OK : StatusCode::ERROR,
      resp->accepted ? "Clear waypoints accepted" : "Clear waypoints rejected");
    out.add_result("accepted", bool_to_string(resp->accepted));
    out.add_result("clear", bool_to_string(clear));
    return out;
  }

  // GET WAYPOINT STATUS SERVICE
  if (resource_name == "get_waypoint_status") {
    if (!get_waypoint_status_client_->wait_for_service(service_wait_timeout_)) {
      return trace::Outcome(StatusCode::ERROR, "Service not available: /blueye/get_waypoint_status");
    }

    auto req = std::make_shared<GetWaypointStatus::Request>();
    auto future = get_waypoint_status_client_->async_send_request(req);

    const auto status = future.wait_for(service_response_timeout_);
    if (status != std::future_status::ready) {
      return trace::Outcome(StatusCode::ERROR, "Timeout waiting for /blueye/get_waypoint_status response");
    }

    auto resp = future.get();
    if (!resp) {
      return trace::Outcome(StatusCode::ERROR, "Null response from /blueye/get_waypoint_status");
    }

    trace::Outcome out(
      resp->accepted ? StatusCode::OK : StatusCode::ERROR,
      resp->accepted ? "Get waypoint status accepted" : "Get waypoint status rejected");
    out.add_result("accepted", bool_to_string(resp->accepted));
    out.add_result("status_code", trim(resp->status_code));
    return out;
  }

  // GET WAYPOINTS SERVICE
  if (resource_name == "get_waypoints") {
    if (!get_waypoints_client_->wait_for_service(service_wait_timeout_)) {
      return trace::Outcome(StatusCode::ERROR, "Service not available: /blueye/get_waypoints");
    }

    auto req = std::make_shared<GetWaypoints::Request>();
    auto future = get_waypoints_client_->async_send_request(req);

    const auto status = future.wait_for(service_response_timeout_);
    if (status != std::future_status::ready) {
      return trace::Outcome(StatusCode::ERROR, "Timeout waiting for /blueye/get_waypoints response");
    }

    auto resp = future.get();
    if (!resp) {
      return trace::Outcome(StatusCode::ERROR, "Null response from /blueye/get_waypoints");
    }

    trace::Outcome out(
      resp->accepted ? StatusCode::OK : StatusCode::ERROR,
      resp->accepted ? "Get waypoints accepted" : "Get waypoints rejected");
    out.add_result("accepted", bool_to_string(resp->accepted));
    out.add_result("waypoint_count", std::to_string(resp->waypoints.size()));

    for (std::size_t i = 0; i < resp->waypoints.size(); ++i) {
      const auto &wp = resp->waypoints[i];
      std::ostringstream wp_str;
      wp_str << "x=" << wp.x
             << ",y=" << wp.y
             << ",z=" << wp.z
             << ",desired_speed=" << wp.desired_speed
             << ",fixed_heading=" << bool_to_string(wp.fixed_heading)
             << ",heading=" << wp.heading;
      out.add_result("waypoint_" + std::to_string(i), wp_str.str());
    }
    return out;
  }

  // INSERT WAYPOINT SERVICE
  if (resource_name == "insert_waypoint") {
    if (!insert_waypoint_client_->wait_for_service(service_wait_timeout_)) {
      return trace::Outcome(StatusCode::ERROR, "Service not available: /blueye/insert_waypoint");
    }

    auto get_required_float = [&](const std::string &key, float &value) -> trace::Outcome {
      const auto it = properties.find(key);
      if (it == properties.end()) {
        return trace::Outcome(StatusCode::ERROR, "Missing required property: " + key);
      }
      if (!parse_float(it->second, value)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid float property: " + key);
      }
      return trace::Outcome(StatusCode::OK, "");
    };

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float desired_velocity = 0.0f;
    float heading = 0.0f;
    bool fixed_heading = false;
    std::int32_t index = 0;

    trace::Outcome parse_out = get_required_float("x", x);
    if (!parse_out.ok()) return parse_out;
    parse_out = get_required_float("y", y);
    if (!parse_out.ok()) return parse_out;
    parse_out = get_required_float("z", z);
    if (!parse_out.ok()) return parse_out;

    const auto v_it = properties.find("desired_velocity");
    if (v_it == properties.end()) {
      return trace::Outcome(StatusCode::ERROR, "Missing required property: desired_velocity");
    }
    if (!parse_float(v_it->second, desired_velocity)) {
      return trace::Outcome(StatusCode::ERROR, "Invalid float property: desired_velocity");
    }

    const auto index_it = properties.find("index");
    if (index_it == properties.end()) {
      return trace::Outcome(StatusCode::ERROR, "Missing required property: index");
    }
    if (!parse_int32(index_it->second, index)) {
      return trace::Outcome(StatusCode::ERROR, "Invalid int property: index");
    }

    const auto fixed_it = properties.find("fixed_heading");
    if (fixed_it != properties.end() && !trim(fixed_it->second).empty()) {
      if (!parse_bool(fixed_it->second, fixed_heading)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid boolean property: fixed_heading");
      }
    }

    const auto heading_it = properties.find("heading");
    if (heading_it != properties.end() && !trim(heading_it->second).empty()) {
      if (!parse_float(heading_it->second, heading)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid float property: heading");
      }
    }

    auto req = std::make_shared<InsertWaypoint::Request>();
    req->x = x;
    req->y = y;
    req->z = z;
    req->desired_velocity = desired_velocity;
    req->fixed_heading = fixed_heading;
    req->heading = heading;
    req->index = index;

    auto future = insert_waypoint_client_->async_send_request(req);
    const auto status = future.wait_for(service_response_timeout_);
    if (status != std::future_status::ready) {
      return trace::Outcome(StatusCode::ERROR, "Timeout waiting for /blueye/insert_waypoint response");
    }

    auto resp = future.get();
    if (!resp) {
      return trace::Outcome(StatusCode::ERROR, "Null response from /blueye/insert_waypoint");
    }

    trace::Outcome out(
      resp->accepted ? StatusCode::OK : StatusCode::ERROR,
      resp->accepted ? "Insert waypoint accepted" : "Insert waypoint rejected");
    out.add_result("accepted", bool_to_string(resp->accepted));
    out.add_result("x", float_to_string(x));
    out.add_result("y", float_to_string(y));
    out.add_result("z", float_to_string(z));
    out.add_result("desired_velocity", float_to_string(desired_velocity));
    out.add_result("fixed_heading", bool_to_string(fixed_heading));
    out.add_result("heading", float_to_string(heading));
    out.add_result("index", int_to_string(index));
    return out;
  }

  // REMOVE WAYPOINT SERVICE
  if (resource_name == "remove_waypoint") {
    if (!remove_waypoint_client_->wait_for_service(service_wait_timeout_)) {
      return trace::Outcome(StatusCode::ERROR, "Service not available: /blueye/remove_waypoint");
    }

    const auto index_it = properties.find("index");
    if (index_it == properties.end()) {
      return trace::Outcome(StatusCode::ERROR, "Missing required property: index");
    }

    std::int32_t index = 0;
    if (!parse_int32(index_it->second, index)) {
      return trace::Outcome(StatusCode::ERROR, "Invalid int property: index");
    }

    auto req = std::make_shared<RemoveWaypoint::Request>();
    req->index = index;
    auto future = remove_waypoint_client_->async_send_request(req);

    const auto status = future.wait_for(service_response_timeout_);
    if (status != std::future_status::ready) {
      return trace::Outcome(StatusCode::ERROR, "Timeout waiting for /blueye/remove_waypoint response");
    }

    auto resp = future.get();
    if (!resp) {
      return trace::Outcome(StatusCode::ERROR, "Null response from /blueye/remove_waypoint");
    }

    trace::Outcome out(
      resp->accepted ? StatusCode::OK : StatusCode::ERROR,
      resp->accepted ? "Remove waypoint accepted" : "Remove waypoint rejected");
    out.add_result("accepted", bool_to_string(resp->accepted));
    out.add_result("index", int_to_string(index));
    out.add_result("error_code", trim(resp->error_code));
    return out;
  }

  if (resource_name == "add_waypoint") {
    if (!add_waypoint_client_->wait_for_service(service_wait_timeout_)) {
      return trace::Outcome(StatusCode::ERROR, "Service not available: /blueye/add_waypoint");
    }

    auto get_required_float = [&](const std::string &key, float &value) -> trace::Outcome {
      const auto it = properties.find(key);
      if (it == properties.end()) {
        return trace::Outcome(StatusCode::ERROR, "Missing required property: " + key);
      }
      if (!parse_float(it->second, value)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid float property: " + key);
      }
      return trace::Outcome(StatusCode::OK, "");
    };

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float desired_velocity = 0.0f;
    float heading = 0.0f;
    bool fixed_heading = false;

    trace::Outcome parse_out = get_required_float("x", x);
    if (!parse_out.ok()) return parse_out;
    parse_out = get_required_float("y", y);
    if (!parse_out.ok()) return parse_out;
    parse_out = get_required_float("z", z);
    if (!parse_out.ok()) return parse_out;

    const auto v_it = properties.find("desired_velocity");
    if (v_it == properties.end()) {
      return trace::Outcome(StatusCode::ERROR, "Missing required property: desired_velocity");
    }
    if (!parse_float(v_it->second, desired_velocity)) {
      return trace::Outcome(StatusCode::ERROR, "Invalid float property: desired_velocity");
    }

    const auto fixed_it = properties.find("fixed_heading");
    if (fixed_it != properties.end() && !trim(fixed_it->second).empty()) {
      if (!parse_bool(fixed_it->second, fixed_heading)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid boolean property: fixed_heading");
      }
    }

    const auto heading_it = properties.find("heading");
    if (heading_it != properties.end() && !trim(heading_it->second).empty()) {
      if (!parse_float(heading_it->second, heading)) {
        return trace::Outcome(StatusCode::ERROR, "Invalid float property: heading");
      }
    }

    auto req = std::make_shared<AddWaypoint::Request>();
    req->x = x;
    req->y = y;
    req->z = z;
    req->desired_velocity = desired_velocity;
    req->fixed_heading = fixed_heading;
    req->heading = heading;

    auto future = add_waypoint_client_->async_send_request(req);
    const auto status = future.wait_for(service_response_timeout_);
    if (status != std::future_status::ready) {
      return trace::Outcome(StatusCode::ERROR, "Timeout waiting for /blueye/add_waypoint response");
    }

    auto resp = future.get();
    if (!resp) {
      return trace::Outcome(StatusCode::ERROR, "Null response from /blueye/add_waypoint");
    }

    trace::Outcome out(
      resp->accepted ? StatusCode::OK : StatusCode::ERROR,
      resp->accepted ? "Add waypoint accepted" : "Add waypoint rejected");

    out.add_result("accepted", bool_to_string(resp->accepted));
    out.add_result("x", float_to_string(x));
    out.add_result("y", float_to_string(y));
    out.add_result("z", float_to_string(z));
    out.add_result("v", float_to_string(desired_velocity));
    out.add_result("desired_velocity", float_to_string(desired_velocity));
    out.add_result("fixed_heading", bool_to_string(fixed_heading));
    out.add_result("heading", float_to_string(heading));
    return out;
  }

  return trace::Outcome(StatusCode::ERROR, "Unsupported resource_name: " + resource_name);
}

}  // namespace blueye_ros_connector
}  // namespace trace
