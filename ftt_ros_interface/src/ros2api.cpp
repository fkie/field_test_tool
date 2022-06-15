#include <ftt_ros_interface/ros2api.h>
#include <ftt_ros_interface/base64.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

Ros2api::Ros2api()
{
  ROS_INFO_STREAM("Initiating Ros2api.");
  nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
  priv_nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle("~"));

  start_logging = false;

  // Read absolute parameters
  nh->param<std::string>("/set_ftt_logging_service", set_logging_service_name , "/set_ftt_logging");
  nh->param<std::string>("/get_ftt_logging_service", get_logging_service_name , "/get_ftt_logging");
  nh->param<std::string>("/save_ftt_params_service", save_params_service_name , "/save_ftt_params");

  // Service servers
  set_logging_service = nh->advertiseService(set_logging_service_name, &Ros2api::setLogging, this);
  get_logging_service = nh->advertiseService(get_logging_service_name, &Ros2api::getLogging, this);
  save_params_service = nh->advertiseService(save_params_service_name, &Ros2api::saveParams, this);
}

Ros2api::~Ros2api()
{
  ROS_INFO_STREAM("Closing Ros2api.");
}

void Ros2api::resetVariables()
{
  last_robot_mode = -1;
  last_ts = 0;
  last_lat = 0;
  last_lng = 0;
  last_pose_stamped = geometry_msgs::PoseStamped();
  last_map = nav_msgs::OccupancyGrid();
  map_sent = false;
}

std::string Ros2api::printRespJson(std::string resp)
{
  std::string print_msg("");
  std::string resp_message;
  if(resp.length() >= 2)
  {
    try
    {
      json resp_json = json::parse(resp);
      if (resp_json.contains("message"))
      {
        resp_message = resp_json["message"];
      }
    }
    catch (const std::exception& e)
    {
      size_t json_lim_ini = resp.find("{");
      size_t json_lim_end = resp.find("}");
      if (json_lim_ini != std::string::npos && json_lim_end != std::string::npos)
      {
        resp_message = resp.substr(json_lim_ini+1, json_lim_end-1);
        size_t msg_ini = resp_message.find("\"");
        if (msg_ini != std::string::npos)
        {
          size_t msg_end = resp_message.find("\"", msg_ini+1);
          if (msg_end != std::string::npos)
          {
            resp_message = resp_message.substr(msg_ini+1, msg_end-2);
          }
        }
      }
    }
    size_t tag_pos;
    if ((tag_pos = resp_message.find("[ERROR]")) != std::string::npos)
    {
      print_msg = resp_message.substr(tag_pos+8);
      ROS_ERROR_STREAM(print_msg);
    }
    else if ((tag_pos = resp_message.find("[WARN]")) != std::string::npos)
    {
      print_msg = resp_message.substr(tag_pos+7);
      ROS_WARN_STREAM(print_msg);
    }
    else if ((tag_pos = resp_message.find("[INFO]")) != std::string::npos)
    {
      print_msg = resp_message.substr(tag_pos+7);
      ROS_INFO_STREAM(print_msg);
    }
    else if (resp_message.length() > 0)
    {
      print_msg = resp_message;
      ROS_INFO_STREAM(print_msg);
    }
  }
  if (print_msg.length() == 0)
  {
    print_msg = "Server internal error.";
    ROS_ERROR_STREAM(print_msg);
  }
  return print_msg;
}

void Ros2api::getParams()
{
  // Read parameters
  priv_nh->param<bool>("params/use_tf", use_tf , true);
  priv_nh->param<std::string>("params/map_frame", map_frame , "map");
  priv_nh->param<std::string>("params/robot_frame", robot_frame , "base_link");
  priv_nh->param<std::string>("params/server_address", server_address , "localhost:5000");
  priv_nh->param<double>("params/send_pose_period", send_pose_period , 2.0);
  priv_nh->param<double>("params/send_map_period", send_map_period , 2.0);
  
  priv_nh->param<std::string>("topics/robot_mode", robot_mode_topic , "robot_mode");
  priv_nh->param<std::string>("topics/gps_fix", gps_fix_topic , "gps_fix");
  priv_nh->param<std::string>("topics/local_pose", local_pose_topic , "local_pose");
  priv_nh->param<std::string>("topics/map", map_topic , "map");
}

void Ros2api::subscribeData()
{
  ROS_INFO_STREAM("Subscribing to robot data.");
  // Reset variables
  resetVariables();
  // Create robot data subscribers
  data_subscribers.push_back(nh->subscribe(robot_mode_topic, 1, &Ros2api::robotModeCallback, this));
  data_subscribers.push_back(nh->subscribe(gps_fix_topic, 1, &Ros2api::navSatFixCallback, this));
  data_subscribers.push_back(nh->subscribe(local_pose_topic, 1, &Ros2api::poseCallback, this));
  data_subscribers.push_back(nh->subscribe(map_topic, 1, &Ros2api::mapCallback, this));
  
  // If using tf, create tf listener for local position
  if (use_tf)
  {
    tf2_listener = new tf2_ros::TransformListener(tf2_buffer);
  }
}

void Ros2api::unsubscribeData()
{
  ROS_INFO_STREAM("Unsubscribing from robot data.");
  // for (auto & subscriber: data_subscribers)
  // {
  //   subscriber.shutdown();
  // }
  data_subscribers.clear();

  if (use_tf)
  {
      delete tf2_listener;
      tf2_listener = nullptr;
  }
}

void Ros2api::createPostTimers()
{
  // Create data POST timers
  post_timers.push_back(nh->createTimer(ros::Duration(send_pose_period), &Ros2api::sendPoseTimerCb, this));
  post_timers.push_back(nh->createTimer(ros::Duration(send_map_period), &Ros2api::sendMapTimerCb, this));
}

void Ros2api::stopPostTimers()
{
  // for (auto const& timer: post_timers)
  // {
  //   timer.stop();
  // }
  post_timers.clear();
}

std::pair<bool,std::string> Ros2api::closeCurrentSegment()
{
  std::string body{"{\"id\": \"Current Segment\", \"action\": \"close\"}"};
  cpr::Response r = cpr::Put(
    cpr::Url{std::string("http://") + server_address + std::string("/segment")},
    cpr::Body{body},
    cpr::Header{{"Content-Type", "application/json"}}
  );
  if (r.status_code == 200)
  {
    last_robot_mode = -1;
    std::string msg = printRespJson(r.text);
    return std::make_pair(true, msg);
  }
  else if (r.status_code == 400)
  {
    printRespJson(r.text);
    return std::make_pair(true, std::string("Segment already closed."));
  }
  else
  {
    std::string msg = "Server unavailable!";
    ROS_ERROR_STREAM(msg);
    return std::make_pair(false, msg);
  }
}

bool Ros2api::setLogging(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data && !start_logging)
  {
    start_logging = true;
    // Get params
    getParams();
    // Subscribe to robot data
    subscribeData();
    // Create POST timers
    createPostTimers();
  }
  else if (!req.data && start_logging)
  {
    // Close open segment
    std::pair<bool,std::string> close_resp = closeCurrentSegment();
    if (close_resp.first)
    {
      start_logging = false;
      // Unsubscribe to robot data
      unsubscribeData();
      // Stop POST timers
      stopPostTimers();
    }
    else
    {
      res.success = false;
      res.message = close_resp.second;
    }
  }
  else
  {
    if (req.data)
    {
      ROS_WARN_STREAM("Tried to start logging, but it's already running.");
    }
    else
    {
      ROS_WARN_STREAM("Tried to stop logging, but it's not running.");
    }
  }
  res.success = true;
  res.message = std::string("Logging is ") + (req.data ? std::string("started.") : std::string("stopped."));
  return true;
}

bool Ros2api::getLogging(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // Return current logging status
  res.success = start_logging;
  return true;
}

bool Ros2api::saveParams(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // TODO
}

void Ros2api::robotModeCallback(const industrial_msgs::RobotMode &msg)
{
  // Check robot mode change
  if (last_robot_mode != msg.val)
  {
    unsigned int vehicle_mode;
    if (msg.val == msg.MANUAL)
    {
      vehicle_mode = SEGMENT_TYPE_ITO;
    }
    else
    {
      if (msg.val == msg.AUTO)
      {
        vehicle_mode = SEGMENT_TYPE_AUTO;
      }
      else
      {
        ROS_WARN_STREAM("Unknown robot mode. Supported modes are MANUAL (" <<  msg.MANUAL << ") and AUTO (" << msg.AUTO << "). Please check your system.");
        return;
      }
    }
    ROS_INFO_STREAM("Reported vehicle mode changed to " << (vehicle_mode == SEGMENT_TYPE_ITO ? "MANUAL" : "AUTO"));
    // Get local position data
    bool valid_local_pose = false;
    double local_x;
    double local_y;
    if (use_tf)
    {
      geometry_msgs::TransformStamped transformStamped;
      try
      {
        transformStamped = tf2_buffer.lookupTransform(map_frame, robot_frame, ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_STREAM("Posting new segment without local position data.");
      }
      local_x = transformStamped.transform.translation.x;
      local_y = transformStamped.transform.translation.y;
      valid_local_pose = true;
    }
    else
    {
      if (last_pose_stamped.header.stamp.isValid() && (ros::Time::now() - last_pose_stamped.header.stamp).toSec() < send_pose_period)
      {
        local_x = last_pose_stamped.pose.position.x;
        local_y = last_pose_stamped.pose.position.y;
        valid_local_pose = true;
      }
      else
      {
        ROS_WARN_STREAM("Posting new segment without local position data.");
      }
    }
    // Check gps position data
    if (last_lat == 0 && last_lng == 0)
    {
      ROS_WARN_STREAM("Posting new segment without gps position data.");
    }
    // Build the body of the HTTP request
    json data;
    data["leg_id"] = "Current Leg";
    data["segment_type_id"] = vehicle_mode;
    data["ito_reason_id"] = ITO_REASON_UNASSIGNED;
    if (last_lng != 0 && last_lat != 0)
    {
      data["lng"] = last_lng;
      data["lat"] = last_lat;
    }
    else
    {
      data["lng"] = nullptr;
      data["lat"] = nullptr;
    }
    if (valid_local_pose)
    {
      data["local_x"] = local_x;
      data["local_y"] = local_y;
    }
    else
    {
      data["local_x"] = nullptr;
      data["local_y"] = nullptr;
    }
    data["orig_starttime_secs"] = ros::Time::now().toSec();
    // Post new segment
    cpr::Response r = cpr::Post(
      cpr::Url{std::string("http://") + server_address + std::string("/segment")},
      cpr::Body{data.dump()},
      cpr::Header{{"Content-Type", "application/json"}}
    );
    if (r.status_code == 200)
    {
      last_robot_mode = msg.val;
      printRespJson(r.text);
    }
    else if (r.status_code == 400)
    {
      printRespJson(r.text);
    }
    else
    {
      ROS_ERROR_STREAM("Server unavailable!");
    }
  }
}

void Ros2api::navSatFixCallback(const sensor_msgs::NavSatFix &msg)
{
  last_ts = msg.header.stamp.toSec();
  last_lat = msg.latitude;
  last_lng = msg.longitude;
}

void Ros2api::poseCallback(const geometry_msgs::PoseStamped &msg)
{
  last_pose_stamped = msg;
}

void Ros2api::mapCallback(const nav_msgs::OccupancyGrid &msg)
{
  // Copy the message
  last_map = msg;
}

void Ros2api::sendPoseTimerCb(const ros::TimerEvent& event)
{
  if (last_robot_mode != -1)
  {
    sendLastGpsPose();
    sendLastLocalPose();
  }
}

void Ros2api::sendMapTimerCb(const ros::TimerEvent& event)
{
  if (last_map.header.stamp.isValid())
  {
    if (map_sent)
    {
      updateMap(last_map);
    }
    else
    {
      sendNewMap(last_map);
    }
  }
}

void Ros2api::sendLastGpsPose()
{
  if (last_lng != 0 && last_lat != 0)
  {
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
      cpr::Body{data.dump()},
      cpr::Header{{"Content-Type", "application/json"}}
    );
    if (r.status_code == 200)
    {
      last_lng = 0;
      last_lat = 0;
      printRespJson(r.text);
    }
    else if (r.status_code == 400)
    {
      printRespJson(r.text);
    }
    else
    {
      ROS_ERROR_STREAM("Server unavailable!");
    }
  }
}

void Ros2api::sendLastLocalPose()
{
  if (map_sent)
  {
    bool valid_local_pose = false;
    double local_x;
    double local_y;
    ros::Time pose_ts;
    if (use_tf)
    {
      geometry_msgs::TransformStamped transformStamped;
      try
      {
        transformStamped = tf2_buffer.lookupTransform(map_frame, robot_frame, ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_STREAM("Unable to get transfrom from " << map_frame << " to " << robot_frame);
      }
      local_x = transformStamped.transform.translation.x;
      local_y = transformStamped.transform.translation.y;
      pose_ts = ros::Time::now();
      valid_local_pose = true;
    }
    else if (last_pose_stamped.header.stamp.isValid() && (ros::Time::now() - last_pose_stamped.header.stamp).toSec() < send_pose_period)
    {
      local_x = last_pose_stamped.pose.position.x;
      local_y = last_pose_stamped.pose.position.y;
      pose_ts = last_pose_stamped.header.stamp;
      valid_local_pose = true;
    }
    if (valid_local_pose)
    {
      // Build the body of the HTTP request
      json data;
      data["segment_id"] = "Current Segment";
      data["frame_id"] = map_frame;
      data["x"] = local_x;
      data["y"] = local_y;
      data["orig_secs"] = pose_ts.toSec();
      // Post new local pose
      cpr::Response r = cpr::Post(
        cpr::Url{std::string("http://") + server_address + std::string("/local_pose")},
        cpr::Body{data.dump()},
        cpr::Header{{"Content-Type", "application/json"}}
      );
      if (r.status_code == 200)
      {
        last_pose_stamped = geometry_msgs::PoseStamped();
        printRespJson(r.text);
      }
      else if (r.status_code == 400)
      {
        printRespJson(r.text);
      }
      else
      {
        ROS_ERROR_STREAM("Server unavailable!");
      }
    }
  }
}

std::string Ros2api::encodeMap(const nav_msgs::OccupancyGrid &msg)
{
  // Convert Occupancy Grid to cv::Mat
  int idx = 0;
  cv::Mat image(msg.info.width, msg.info.height, CV_8UC1);
  for(std::vector<int8_t>::const_iterator it = msg.data.begin(); it != msg.data.end(); ++it, ++idx) {
    // Default value (Unkown)
    int value = 127;
    // Rescale 0-100 to 0-255
    if ((int)*it >= 0) {
      value = 255 - (int)*it*255/100;
    } 
    image.at<uchar>(idx/image.cols, idx%image.cols) = (uchar)value;
  }
  // Flip image
  cv::Mat flipped_image;
  cv::flip(image, flipped_image, 0);
  // Transform to jpeg
  std::vector<uchar> buf;
  cv::imencode(".jpg", flipped_image, buf);
  // Encode in base64
  auto *enc_msg = reinterpret_cast<unsigned char*>(buf.data());
  std::string encoded = base64_encode(enc_msg, buf.size());
  return encoded;
}

void Ros2api::sendNewMap(const nav_msgs::OccupancyGrid &msg)
{
  std::string map_img_encoded = encodeMap(msg);
  // Build the body of the HTTP request
  json data;
  data["shift_id"] = "Current Shift";
  data["frame_id"] = msg.header.frame_id;
  data["width"] = msg.info.width;
  data["height"] = msg.info.height;
  data["resolution"] = msg.info.resolution;
  data["origin_x"] = msg.info.origin.position.x;
  data["origin_y"] = msg.info.origin.position.y;
  data["image_data"] = map_img_encoded;
  data["orig_secs"] = msg.header.stamp.toSec();
  // Post new map
  cpr::Response r = cpr::Post(
    cpr::Url{std::string("http://") + server_address + std::string("/map_image")},
    cpr::Body{data.dump()},
    cpr::Header{{"Content-Type", "application/json"}}
  );
  if (r.status_code == 200)
  {
    map_sent = true;
    last_map = nav_msgs::OccupancyGrid();
    printRespJson(r.text);
  }
  else if (r.status_code == 400)
  {
    printRespJson(r.text);
    updateMap(msg);
  }
  else
  {
    ROS_ERROR_STREAM("Server unavailable!");
  }
}

void Ros2api::updateMap(const nav_msgs::OccupancyGrid &msg)
{
  std::string map_img_encoded = encodeMap(msg);
  // Build the body of the HTTP request
  json data;
  data["shift_id"] = "Current Shift";
  data["frame_id"] = msg.header.frame_id;
  data["width"] = msg.info.width;
  data["height"] = msg.info.height;
  data["resolution"] = msg.info.resolution;
  data["origin_x"] = msg.info.origin.position.x;
  data["origin_y"] = msg.info.origin.position.y;
  data["image_data"] = map_img_encoded;
  data["orig_secs"] = msg.header.stamp.toSec();
  // Update map
  cpr::Response r = cpr::Put(
    cpr::Url{std::string("http://") + server_address + std::string("/map_image")},
    cpr::Body{data.dump()},
    cpr::Header{{"Content-Type", "application/json"}}
  );
  if (r.status_code == 200)
  {
    map_sent = true;
    last_map = nav_msgs::OccupancyGrid();
    printRespJson(r.text);
  }
  else if (r.status_code == 400)
  {
    printRespJson(r.text);
  }
  else
  {
    ROS_ERROR_STREAM("Server unavailable!");
  }
}

int main( int argc, char** argv ) {
  ros::init( argc, argv, "ros2api" );
  Ros2api ros2api;
  ros::spin();
  return 0;
}
