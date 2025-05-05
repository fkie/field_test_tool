#ifndef ROS2API_H
#define ROS2API_H

#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class Ros2api : public rclcpp::Node
{
public:
  Ros2api();
  virtual ~Ros2api();

protected:
  void resetVariables();
  std::string printRespJson(std::string resp);
  void getParams();
  void subscribeData();
  void unsubscribeData();
  void createPostTimers();
  void stopPostTimers();
  std::pair<bool, std::string> closeCurrentSegment();
  void setLogging(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  void getLogging(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void saveParams(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void autonomousModeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void sendPoseTimerCb();
  void sendMapTimerCb();
  void sendLastGpsPose();
  void sendLastLocalPose();
  std::string encodeMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void sendNewMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void sendImageBuffer();
  std::string encodeImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void sendImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
  //Internal variables
  const unsigned int SEGMENT_TYPE_ITO = 1;
  const unsigned int SEGMENT_TYPE_AUTO = 2;
  const unsigned int SEGMENT_TYPE_TELEOP = 3;
  const unsigned int ITO_REASON_UNASSIGNED = 11;

  bool start_logging;
  bool robot_mode_received;
  bool last_robot_mode;
  double last_ts;
  double last_lat;
  double last_lng;
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_stamped;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map;
  bool map_sent;
  rclcpp::Time last_image_time;

  //ROS node pointer
  std::shared_ptr<rclcpp::Node> node;

  //ROS params
  bool use_tf;
  std::string map_frame;
  std::string robot_frame;
  std::string server_address;
  double send_pose_period;
  double send_map_period;
  int image_count;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_buffer;
  int image_buffer_size;
  double image_buffer_step;

  std::string robot_mode_topic;
  std::string gps_fix_topic;
  std::string local_pose_topic;
  std::string map_topic;
  std::string image_topic;

  //ROS subscribers, publishers and services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_logging_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_logging_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_params_service;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> data_subscribers;
  std::vector<rclcpp::TimerBase::SharedPtr> post_timers;
  image_transport::Subscriber image_sub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  std::shared_ptr<image_transport::ImageTransport> it;
};

#endif
