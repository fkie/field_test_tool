#ifndef ROBOT_POSE_PUBLISHER_H
#define ROBOT_POSE_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class RobotPosePublisher : public rclcpp::Node {
public:
  RobotPosePublisher();
	virtual ~RobotPosePublisher();
protected:
	void publishTimerCb();
private:
	double publish_rate;
  std::string robot_frame;
  std::string global_frame;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  rclcpp::TimerBase::SharedPtr timer;
};

#endif
