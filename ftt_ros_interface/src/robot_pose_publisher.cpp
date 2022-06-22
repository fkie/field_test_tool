#include <ftt_ros_interface/robot_pose_publisher.h>


RobotPosePublisher::RobotPosePublisher()
{
  ros::NodeHandle nh, priv_nh("~");

  // Get parameters
  priv_nh.param<double>("publish_rate", publish_rate , 10);
  priv_nh.param<std::string>("robot_frame", robot_frame, "base_link");
  priv_nh.param<std::string>("global_frame", global_frame, "map");
  

  // Publisher
  pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);

  // TF listener
  tf2_listener = new tf2_ros::TransformListener(tf2_buffer);

  // Publish timer
  timer = nh.createTimer(ros::Duration(1.0/publish_rate), &RobotPosePublisher::publishTimerCb, this);
}

RobotPosePublisher::~RobotPosePublisher() {}

void RobotPosePublisher::publishTimerCb(const ros::TimerEvent& event)
{
  // Get the transformation between robot and global frame
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf2_buffer.lookupTransform(global_frame, robot_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }
  
  // Make robot pose message with the transformation data
  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = transformStamped.transform.translation.x;
  robot_pose.position.y = transformStamped.transform.translation.y;
  robot_pose.position.z = transformStamped.transform.translation.z;
  robot_pose.orientation = transformStamped.transform.rotation;

  // Publish robot pose
  pub.publish(robot_pose);
}

int main( int argc, char** argv ) {
  ros::init( argc, argv, "robot_pose_publisher" );
  RobotPosePublisher robot_pose_pub;
  ros::spin();
  return 0;
}
