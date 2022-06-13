#ifndef MAP_TO_JPEG_H
#define MAP_TO_JPEG_H

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>

class MapToJpeg {
public:
  MapToJpeg();
	virtual ~MapToJpeg();
protected:
  void mapCallback(const nav_msgs::OccupancyGrid &msg);
	void publishTimerCb(const ros::TimerEvent& event);
private:
	double publish_rate;
  bool new_map;
  nav_msgs::OccupancyGrid map;
  
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Timer timer;
};

#endif
