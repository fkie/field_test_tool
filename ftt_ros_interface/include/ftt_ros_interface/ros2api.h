#ifndef ROS2API_H
#define ROS2API_H

#include <ros/ros.h>

#include <industrial_msgs/RobotMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>

#include <string>
#include <utility>
#include <vector>
#include <deque>

class Ros2api {
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
  std::pair<bool,std::string> closeCurrentSegment();
  bool setLogging(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool getLogging(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool saveParams(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void robotModeCallback(const industrial_msgs::RobotMode &msg);
  void navSatFixCallback(const sensor_msgs::NavSatFix &msg);
  void poseCallback(const geometry_msgs::PoseStamped &msg);
  void mapCallback(const nav_msgs::OccupancyGrid &msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void sendPoseTimerCb(const ros::TimerEvent& event);
  void sendMapTimerCb(const ros::TimerEvent& event);
  void sendLastGpsPose();
  void sendLastLocalPose();
  std::string encodeMap(const nav_msgs::OccupancyGrid &msg);
  void sendNewMap(const nav_msgs::OccupancyGrid &msg);
  void updateMap(const nav_msgs::OccupancyGrid &msg);
  void sendImageBuffer();
  std::string encodeImage(const sensor_msgs::ImageConstPtr& msg);
  void sendImage(const sensor_msgs::ImageConstPtr& msg);
private:
  //Internal variables
  const unsigned int SEGMENT_TYPE_ITO = 1;
  const unsigned int SEGMENT_TYPE_AUTO = 2;
  const unsigned int SEGMENT_TYPE_TELEOP = 3;
  const unsigned int ITO_REASON_UNASSIGNED = 11;
  
  bool start_logging;
  int last_robot_mode;
  double last_ts;
  double last_lat;
  double last_lng;
  geometry_msgs::PoseStamped last_pose_stamped;
  nav_msgs::OccupancyGrid last_map;
  bool map_sent;
  ros::Time last_image_time;

  //ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  //ROS params
  std::string set_logging_service_name;
  std::string get_logging_service_name;
  std::string save_params_service_name;
  bool use_tf;
  std::string map_frame;
  std::string robot_frame;
  std::string server_address;
  double send_pose_period;
  double send_map_period;
  int image_count;
  std::deque<sensor_msgs::ImageConstPtr> image_buffer;
  int image_buffer_size;
  double image_buffer_step;
  
  std::string robot_mode_topic;
  std::string gps_fix_topic;
  std::string local_pose_topic;
  std::string map_topic;
  std::string image_topic;

  //ROS subscribers, publishers and services
  ros::ServiceServer set_logging_service;
  ros::ServiceServer get_logging_service;
  ros::ServiceServer save_params_service;
  std::vector<ros::Subscriber> data_subscribers;
  std::vector<ros::Timer> post_timers;
  image_transport::Subscriber image_sub;

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener *tf2_listener;

  image_transport::ImageTransport it;
};

#endif
