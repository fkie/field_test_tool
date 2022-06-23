#ifndef ROBOT_POSE_PUBLISHER_H
#define ROBOT_POSE_PUBLISHER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class RobotPosePublisher {
public:
  RobotPosePublisher();
	virtual ~RobotPosePublisher();
protected:
	void publishTimerCb(const ros::TimerEvent& event);
private:
	double publish_rate;
  std::string robot_frame;
  std::string global_frame;

  ros::Publisher pub;

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener *tf2_listener;

  ros::Timer timer;
};

#endif
