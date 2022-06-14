#include <ftt_ros_interface/ros2api.h>
#include <ftt_ros_interface/base64.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>


Ros2api::Ros2api()
{
  ROS_INFO_STREAM("Initiating Ros2api.");
  nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
  priv_nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle("~"));

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
  last_position = geometry_msgs::Pose();
  last_map = nav_msgs::OccupancyGrid();
  map_sent = false;
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

bool Ros2api::closeCurrentSegment()
{}

bool Ros2api::setLogging(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{}

bool Ros2api::getLogging(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{}

bool Ros2api::saveParams(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{}

void Ros2api::robotModeCallback(const industrial_msgs::RobotMode &msg)
{}

void Ros2api::navSatFixCallback(const sensor_msgs::NavSatFix &msg)
{}

void Ros2api::poseCallback(const geometry_msgs::Pose &msg)
{}

void Ros2api::mapCallback(const nav_msgs::OccupancyGrid &msg)
{
  // Copy the message
  last_map = msg;
}

void Ros2api::sendPoseTimerCb(const ros::TimerEvent& event)
{}

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
{}

void Ros2api::sendLastLocalPose()
{}

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
{}

void Ros2api::updateMap(const nav_msgs::OccupancyGrid &msg)
{}

int main( int argc, char** argv ) {
  ros::init( argc, argv, "ros2api" );
  Ros2api ros2api;
  ros::spin();
  return 0;
}
