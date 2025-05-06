#include "ftt_ros_interface/ros2api.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cpr/cpr.h"
#include "cv_bridge/cv_bridge.hpp"
#include "ftt_ros_interface/base64.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"

using json = nlohmann::json;

Ros2api::Ros2api() : Node("ros2api")
{
  RCLCPP_INFO_STREAM(get_logger(), "Initiating Ros2api.");

  start_logging = false;
  image_count = 0;
  last_image_time = get_clock()->now();

  // Declare parameters
  declare_parameter<bool>("params.use_tf", true);
  declare_parameter<std::string>("params.map_frame", "map");
  declare_parameter<std::string>("params.robot_frame", "base_link");
  declare_parameter<std::string>("params.server_address", "localhost:5000");
  declare_parameter<double>("params.send_pose_period", 2.0);
  declare_parameter<double>("params.send_map_period", 2.0);
  declare_parameter<int>("params.image_buffer_size", 1);
  declare_parameter<double>("params.image_buffer_step", 1.0);

  declare_parameter<std::string>("topics.robot_mode", "robot_mode");
  declare_parameter<std::string>("topics.gps_fix", "gps_fix");
  declare_parameter<std::string>("topics.local_pose", "local_pose");
  declare_parameter<std::string>("topics.map", "map");
  declare_parameter<std::string>("topics.image", "image");

  // Service servers
  set_logging_service = create_service<std_srvs::srv::SetBool>(
    std::string(get_name()) + "/set_status",
    std::bind(&Ros2api::setLogging, this, std::placeholders::_1, std::placeholders::_2));
  get_logging_service = create_service<std_srvs::srv::Trigger>(
    std::string(get_name()) + "/get_status",
    std::bind(&Ros2api::getLogging, this, std::placeholders::_1, std::placeholders::_2));
  save_params_service = create_service<std_srvs::srv::Trigger>(
    std::string(get_name()) + "/save_params",
    std::bind(&Ros2api::saveParams, this, std::placeholders::_1, std::placeholders::_2));
}

Ros2api::~Ros2api() { RCLCPP_INFO_STREAM(get_logger(), "Closing Ros2api."); }

void Ros2api::resetVariables()
{
  robot_mode_received = false;
  last_robot_mode = false;
  last_ts = 0;
  last_lat = 0;
  last_lng = 0;
  last_pose_stamped.reset();
  last_map.reset();
  map_sent = false;
}

std::string Ros2api::printRespJson(std::string resp)
{
  std::string print_msg("");
  std::string resp_message;
  try {
    json resp_json = json::parse(resp);
    if (resp_json.contains("message")) {
      resp_message = resp_json["message"];
    } else {
      resp_message = resp_json.get<std::string>();
    }
    size_t tag_pos;
    if ((tag_pos = resp_message.find("[ERROR]")) != std::string::npos) {
      print_msg = resp_message.substr(tag_pos + 8);
      RCLCPP_ERROR_STREAM(get_logger(), print_msg);
    } else if ((tag_pos = resp_message.find("[WARN]")) != std::string::npos) {
      print_msg = resp_message.substr(tag_pos + 7);
      RCLCPP_WARN_STREAM(get_logger(), print_msg);
    } else if ((tag_pos = resp_message.find("[INFO]")) != std::string::npos) {
      print_msg = resp_message.substr(tag_pos + 7);
      RCLCPP_INFO_STREAM(get_logger(), print_msg);
    } else {
      print_msg = resp_message;
      RCLCPP_INFO_STREAM(get_logger(), print_msg);
    }
  } catch (const json::parse_error & e) {
    print_msg = "Server internal error.";
    RCLCPP_ERROR_STREAM(get_logger(), print_msg);
  }
  return print_msg;
}

void Ros2api::getParams()
{
  // Read parameters
  use_tf = get_parameter("params.use_tf").as_bool();
  map_frame = get_parameter("params.map_frame").as_string();
  robot_frame = get_parameter("params.robot_frame").as_string();
  server_address = get_parameter("params.server_address").as_string();
  send_pose_period = get_parameter("params.send_pose_period").as_double();
  send_map_period = get_parameter("params.send_map_period").as_double();
  image_buffer_size = get_parameter("params.image_buffer_size").as_int();
  image_buffer_step = get_parameter("params.image_buffer_step").as_double();

  robot_mode_topic = get_parameter("topics.robot_mode").as_string();
  gps_fix_topic = get_parameter("topics.gps_fix").as_string();
  local_pose_topic = get_parameter("topics.local_pose").as_string();
  map_topic = get_parameter("topics.map").as_string();
  image_topic = get_parameter("topics.image").as_string();
}

void Ros2api::subscribeData()
{
  RCLCPP_INFO_STREAM(get_logger(), "Subscribing to robot data.");
  // Reset variables
  resetVariables();
  // Clear the image buffer
  image_buffer.clear();
  // Make sure image transport is initialized
  if (!it) {
    it = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  }
  // Create robot data subscribers
  data_subscribers.clear();
  data_subscribers.push_back(create_subscription<std_msgs::msg::Bool>(
    robot_mode_topic, 2, std::bind(&Ros2api::autonomousModeCallback, this, std::placeholders::_1)));
  data_subscribers.push_back(create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_fix_topic, 2, std::bind(&Ros2api::navSatFixCallback, this, std::placeholders::_1)));
  data_subscribers.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
    local_pose_topic, 2, std::bind(&Ros2api::poseCallback, this, std::placeholders::_1)));
  data_subscribers.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, 2, std::bind(&Ros2api::mapCallback, this, std::placeholders::_1)));

  image_sub =
    it->subscribe(image_topic, 1, std::bind(&Ros2api::imageCallback, this, std::placeholders::_1));

  // If using tf, create tf listener for local position
  if (use_tf) {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }
}

void Ros2api::unsubscribeData()
{
  RCLCPP_INFO_STREAM(get_logger(), "Unsubscribing from robot data.");
  data_subscribers.clear();
  image_sub.shutdown();
}

void Ros2api::createPostTimers()
{
  // Create data POST timers
  post_timers.clear();
  post_timers.push_back(create_wall_timer(
    std::chrono::duration<double>(send_pose_period), std::bind(&Ros2api::sendPoseTimerCb, this)));
  post_timers.push_back(create_wall_timer(
    std::chrono::duration<double>(send_map_period), std::bind(&Ros2api::sendMapTimerCb, this)));
}

void Ros2api::stopPostTimers() { post_timers.clear(); }

std::pair<bool, std::string> Ros2api::closeCurrentSegment()
{
  std::string body{"{\"id\": \"Current Segment\", \"action\": \"close\"}"};
  cpr::Response r = cpr::Put(
    cpr::Url{std::string("http://") + server_address + std::string("/segment")}, cpr::Body{body},
    cpr::Header{{"Content-Type", "application/json"}});
  if (r.status_code == 200) {
    robot_mode_received = false;
    last_robot_mode = false;
    std::string msg = printRespJson(r.text);
    return std::make_pair(true, msg);
  } else if (r.status_code == 400) {
    printRespJson(r.text);
    return std::make_pair(true, std::string("Segment already closed."));
  } else {
    std::string msg = "Server unavailable!";
    RCLCPP_ERROR_STREAM(get_logger(), msg);
    return std::make_pair(false, msg);
  }
}

void Ros2api::setLogging(
  std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data && !start_logging) {
    start_logging = true;
    // Get params
    getParams();
    // Subscribe to robot data
    subscribeData();
    // Create POST timers
    createPostTimers();
  } else if (!req->data && start_logging) {
    // Close open segment
    std::pair<bool, std::string> close_resp = closeCurrentSegment();
    if (close_resp.first) {
      start_logging = false;
      // Unsubscribe to robot data
      unsubscribeData();
      // Stop POST timers
      stopPostTimers();
    } else {
      res->success = false;
      res->message = close_resp.second;
    }
  } else {
    if (req->data) {
      RCLCPP_WARN_STREAM(get_logger(), "Tried to start logging, but it's already running.");
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "Tried to stop logging, but it's not running.");
    }
  }
  res->success = true;
  res->message =
    std::string("Logging is ") + (req->data ? std::string("started.") : std::string("stopped."));
}

void Ros2api::getLogging(
  std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
  // Return current logging status
  res->success = start_logging;
}

void Ros2api::saveParams(
  std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
  // Update params
  getParams();
  // Get the config file's path
  std::string package_path = ament_index_cpp::get_package_share_directory("ftt_ros_interface");
  std::string file_path = package_path + std::string("/config/ros2api_config.yaml");
  // Load the config file
  YAML::Node config;
  try {
    config = YAML::LoadFile(file_path);
  } catch (const YAML::BadFile & e) {
    res->success = false;
    res->message = "Error while opening config file.";
    return;
  }
  // Lambda to ensure stringification of parameter with one decimal place
  auto toStr = [](rclcpp::Parameter param) -> std::string {
    std::string out = param.value_to_string();
    if (param.get_type_name() == "double") {
      std::size_t dot_pos = out.find(".");
      if (dot_pos == std::string::npos) {
        out += ".0";
      } else {
        while (out.size() > dot_pos + 2 && out.back() == '0') {
          out.pop_back();
        }
      }
    }
    return out;
  };
  // Overwrite parameters with current values
  auto parameters = list_parameters({}, 10);
  for (const auto & name : parameters.names) {
    auto parameter = get_parameter(name);
    RCLCPP_INFO_STREAM(
      get_logger(), "Saving parameter " << name << " with value " << toStr(parameter));
    std::string param_ns;
    if (name.find("params.") != std::string::npos) {
      param_ns = "params";
    } else if (name.find("topics.") != std::string::npos) {
      param_ns = "topics";
    }
    if (!param_ns.empty()) {
      config["ftt_ros"]["ros__parameters"][param_ns][name.substr(name.find_last_of('.') + 1)] =
        toStr(parameter);
    }
  }
  // Save the updated config to file
  try {
    std::ofstream fout(file_path);
    fout << config;
    fout.close();
    // Return successfully
    res->success = true;
    res->message = std::string("FTT parameters saved to ") + file_path;
    return;
  } catch (const std::exception & e) {
    // Error
    res->success = false;
    res->message = "Error while saving parameters to file.";
    return;
  }
}

void Ros2api::autonomousModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  auto time_now = get_clock()->now();
  // Check robot mode change
  if (!robot_mode_received || last_robot_mode != msg->data) {
    unsigned int vehicle_mode;
    if (msg->data) {
      vehicle_mode = SEGMENT_TYPE_AUTO;
    } else {
      vehicle_mode = SEGMENT_TYPE_ITO;
    }
    RCLCPP_INFO_STREAM(
      get_logger(), "Reported vehicle mode changed to "
                      << (vehicle_mode == SEGMENT_TYPE_ITO ? "MANUAL" : "AUTO"));
    // Get local position data
    bool valid_local_pose = false;
    double local_x;
    double local_y;
    if (use_tf) {
      geometry_msgs::msg::TransformStamped transform_stamped{};
      try {
        transform_stamped = tf_buffer->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);
        local_x = transform_stamped.transform.translation.x;
        local_y = transform_stamped.transform.translation.y;
        valid_local_pose = true;
      } catch (tf2::TransformException & ex) {
      }
    } else {
      if (
        last_pose_stamped &&
        (time_now - rclcpp::Time(last_pose_stamped->header.stamp)).seconds() < send_pose_period) {
        local_x = last_pose_stamped->pose.position.x;
        local_y = last_pose_stamped->pose.position.y;
        valid_local_pose = true;
      }
    }
    // Check gps position data
    bool valid_gps_pose = last_lng != 0 && last_lat != 0;
    if (!valid_local_pose && !valid_gps_pose) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "No position data available. Will not change the operation mode or post a new segment.");
      return;
    } else if (!valid_local_pose) {
      RCLCPP_WARN_STREAM(get_logger(), "Posting new segment without local position data.");
    } else if (!valid_gps_pose){
      RCLCPP_WARN_STREAM(get_logger(), "Posting new segment without gps position data.");
    }
    // Build the body of the HTTP request
    json data;
    data["leg_id"] = "Current Leg";
    data["segment_type_id"] = vehicle_mode;
    data["ito_reason_id"] = ITO_REASON_UNASSIGNED;
    if (valid_gps_pose) {
      data["lng"] = last_lng;
      data["lat"] = last_lat;
    } else {
      data["lng"] = nullptr;
      data["lat"] = nullptr;
    }
    if (valid_local_pose) {
      data["local_x"] = local_x;
      data["local_y"] = local_y;
    } else {
      data["local_x"] = nullptr;
      data["local_y"] = nullptr;
    }
    data["orig_starttime_secs"] = time_now.seconds();
    // Post new segment
    cpr::Response r = cpr::Post(
      cpr::Url{std::string("http://") + server_address + std::string("/segment")},
      cpr::Body{data.dump()}, cpr::Header{{"Content-Type", "application/json"}});
    if (r.status_code == 200) {
      robot_mode_received = true;
      last_robot_mode = msg->data;
      sendImageBuffer();
      printRespJson(r.text);
    } else if (r.status_code == 400) {
      printRespJson(r.text);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Server unavailable!");
    }
  }
}

void Ros2api::navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  last_ts = rclcpp::Time(msg->header.stamp).seconds();
  last_lat = msg->latitude;
  last_lng = msg->longitude;
}

void Ros2api::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_pose_stamped = msg;
}

void Ros2api::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Copy the message
  last_map = msg;
}

void Ros2api::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  auto time_now = get_clock()->now();
  if ((time_now - last_image_time).seconds() > image_buffer_step) {
    while (image_buffer.size() >= static_cast<std::size_t>(image_buffer_size)) {
      image_buffer.pop_front();
    }
    image_buffer.push_back(msg);
    last_image_time = time_now;
  }
}

void Ros2api::sendPoseTimerCb()
{
  if (robot_mode_received) {
    sendLastGpsPose();
    sendLastLocalPose();
  }
}

void Ros2api::sendMapTimerCb()
{
  if (last_map) {
    if (map_sent) {
      updateMap(last_map);
    } else {
      sendNewMap(last_map);
    }
  }
}

void Ros2api::sendLastGpsPose()
{
  if (last_lng != 0 && last_lat != 0) {
    // Build the body of the HTTP request
    json data;
    data["segment_id"] = "Current Segment";
    data["lng"] = last_lng;
    data["lat"] = last_lat;
    data["pose_source_id"] = 1;
    data["orig_secs"] = last_ts;
    // Post new gps pose
    cpr::Response r = cpr::Post(
      cpr::Url{std::string("http://") + server_address + std::string("/pose")},
      cpr::Body{data.dump()}, cpr::Header{{"Content-Type", "application/json"}});
    if (r.status_code == 200) {
      last_lng = 0;
      last_lat = 0;
      printRespJson(r.text);
    } else if (r.status_code == 400) {
      printRespJson(r.text);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Server unavailable!");
    }
  }
}

void Ros2api::sendLastLocalPose()
{
  if (map_sent) {
    auto time_now = get_clock()->now();
    bool valid_local_pose = false;
    double local_x;
    double local_y;
    rclcpp::Time pose_ts;
    if (use_tf) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      try {
        transform_stamped = tf_buffer->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Unable to get transfrom from " << map_frame << " to " << robot_frame);
      }
      local_x = transform_stamped.transform.translation.x;
      local_y = transform_stamped.transform.translation.y;
      pose_ts = time_now;
      valid_local_pose = true;
    } else if (
      last_pose_stamped &&
      (time_now - rclcpp::Time(last_pose_stamped->header.stamp)).seconds() < send_pose_period) {
      local_x = last_pose_stamped->pose.position.x;
      local_y = last_pose_stamped->pose.position.y;
      pose_ts = rclcpp::Time(last_pose_stamped->header.stamp);
      valid_local_pose = true;
    }
    if (valid_local_pose) {
      // Build the body of the HTTP request
      json data;
      data["segment_id"] = "Current Segment";
      data["frame_id"] = map_frame;
      data["x"] = local_x;
      data["y"] = local_y;
      data["orig_secs"] = pose_ts.seconds();
      // Post new local pose
      cpr::Response r = cpr::Post(
        cpr::Url{std::string("http://") + server_address + std::string("/local_pose")},
        cpr::Body{data.dump()}, cpr::Header{{"Content-Type", "application/json"}});
      if (r.status_code == 200) {
        last_pose_stamped.reset();
        printRespJson(r.text);
      } else if (r.status_code == 400) {
        printRespJson(r.text);
      } else {
        RCLCPP_ERROR_STREAM(get_logger(), "Server unavailable!");
      }
    }
  }
}

std::string Ros2api::encodeMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Convert Occupancy Grid to cv::Mat
  int idx = 0;
  cv::Mat image(msg->info.height, msg->info.width, CV_8UC1);
  for (std::vector<int8_t>::const_iterator it = msg->data.begin(); it != msg->data.end();
       ++it, ++idx) {
    // Default value (Unkown)
    int value = 127;
    // Rescale 0-100 to 0-255
    if (*it >= 0) {
      value = 255 - static_cast<int>(*it) * 255 / 100;
    }
    image.at<uchar>(idx / image.cols, idx % image.cols) = static_cast<uchar>(value);
  }
  // Flip image
  cv::Mat flipped_image;
  cv::flip(image, flipped_image, 0);
  // Transform to jpeg
  std::vector<uchar> buf;
  cv::imencode(".jpg", flipped_image, buf);
  // Encode in base64
  auto * enc_msg = reinterpret_cast<unsigned char *>(buf.data());
  std::string encoded = base64_encode(enc_msg, buf.size());
  return encoded;
}

void Ros2api::sendNewMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN_STREAM(get_logger(), "Received an empty map. Doing nothing.");
    return;
  }
  std::string map_img_encoded = encodeMap(msg);
  // Build the body of the HTTP request
  json data;
  data["shift_id"] = "Current Shift";
  data["frame_id"] = msg->header.frame_id;
  data["width"] = msg->info.width;
  data["height"] = msg->info.height;
  data["resolution"] = msg->info.resolution;
  data["origin_x"] = msg->info.origin.position.x;
  data["origin_y"] = msg->info.origin.position.y;
  data["image_data"] = map_img_encoded;
  data["orig_secs"] = rclcpp::Time(msg->header.stamp).seconds();
  // Post new map
  cpr::Response r = cpr::Post(
    cpr::Url{std::string("http://") + server_address + std::string("/map_image")},
    cpr::Body{data.dump()}, cpr::Header{{"Content-Type", "application/json"}});
  if (r.status_code == 200) {
    map_sent = true;
    last_map.reset();
    printRespJson(r.text);
  } else if (r.status_code == 400) {
    printRespJson(r.text);
    updateMap(msg);
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Server unavailable!");
  }
}

void Ros2api::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN_STREAM(get_logger(), "Received an empty map. Doing nothing.");
    return;
  }
  std::string map_img_encoded = encodeMap(msg);
  // Build the body of the HTTP request
  json data;
  data["shift_id"] = "Current Shift";
  data["frame_id"] = msg->header.frame_id;
  data["width"] = msg->info.width;
  data["height"] = msg->info.height;
  data["resolution"] = msg->info.resolution;
  data["origin_x"] = msg->info.origin.position.x;
  data["origin_y"] = msg->info.origin.position.y;
  data["image_data"] = map_img_encoded;
  data["orig_secs"] = rclcpp::Time(msg->header.stamp).seconds();
  // Update map
  cpr::Response r = cpr::Put(
    cpr::Url{std::string("http://") + server_address + std::string("/map_image")},
    cpr::Body{data.dump()}, cpr::Header{{"Content-Type", "application/json"}});
  if (r.status_code == 200) {
    map_sent = true;
    last_map.reset();
    printRespJson(r.text);
  } else if (r.status_code == 400) {
    printRespJson(r.text);
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Server unavailable!");
  }
}

void Ros2api::sendImageBuffer()
{
  for (auto & img : image_buffer) {
    sendImage(img);
  }
}

std::string Ros2api::encodeImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Convert Image to cv::Mat
  cv::Mat image = cv_bridge::toCvShare(msg)->image;
  // Transform to jpeg
  std::vector<uchar> buf;
  cv::imencode(".jpg", image, buf);
  // Encode in base64
  auto * enc_msg = reinterpret_cast<unsigned char *>(buf.data());
  std::string encoded = base64_encode(enc_msg, buf.size());
  return encoded;
}

void Ros2api::sendImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (msg->data.size() > 0) {
    std::string img_encoded = encodeImage(msg);
    // Build the body of the HTTP request
    json data;
    data["segment_id"] = "Current Segment";
    data["image_filename"] = std::string("rgb_img") + std::to_string(image_count);
    data["image_data"] = img_encoded;
    data["description"] = "Automatic image capture for segment transition";
    data["orig_secs"] = rclcpp::Time(msg->header.stamp).seconds();
    // Post new map
    cpr::Response r = cpr::Post(
      cpr::Url{std::string("http://") + server_address + std::string("/image")},
      cpr::Body{data.dump()}, cpr::Header{{"Content-Type", "application/json"}});
    if (r.status_code == 200) {
      image_count = image_count + 1;
      printRespJson(r.text);
    } else if (r.status_code == 400) {
      printRespJson(r.text);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Server unavailable!");
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ros2api>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
