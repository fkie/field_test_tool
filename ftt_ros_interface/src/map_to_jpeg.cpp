#include <ftt_ros_interface/map_to_jpeg.h>

#include <opencv2/core.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>

MapToJpeg::MapToJpeg()
{
  ros::NodeHandle nh, priv_nh("~");

  // Get parameters
  priv_nh.param<double>("publish_rate", publish_rate , 1.0);
  new_map = false;
  
  // Subscriber
  sub = nh.subscribe("map", 1, &MapToJpeg::mapCallback, this);

  // Publisher
  pub = nh.advertise<sensor_msgs::CompressedImage>("map_jpeg", 1);

  // Publish timer
  timer = nh.createTimer(ros::Duration(1.0/publish_rate), &MapToJpeg::publishTimerCb, this);
}

MapToJpeg::~MapToJpeg() {}

void MapToJpeg::mapCallback(const nav_msgs::OccupancyGrid &msg)
{
  // Copy the message
  map = msg;
  new_map = true;
}

void MapToJpeg::publishTimerCb(const ros::TimerEvent& event)
{
  static int seq = 0;
  if (new_map)
  {
    // Convert Occupancy Grid to cv::Mat
    int idx = 0;
    cv::Mat image(map.info.height, map.info.width, CV_8UC1);
    for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it, ++idx) {
      // Default value (Unkown)
      int value = 127;
      // Rescale 0-100 to 0-255
      if (*it >= 0) {
        value = 255 - static_cast<int>(*it) * 255 / 100;
      }
      image.at<uchar>(idx/image.cols, idx%image.cols) = static_cast<uchar>(value);
    }
    // Flip image
    cv::Mat flipped_image;
    cv::flip(image, flipped_image, 0);
    // Transform to CompressedImage message
    std_msgs::Header header;
    header.seq = seq++;
    header.stamp = ros::Time::now();
    cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::MONO8, flipped_image);
    sensor_msgs::CompressedImage jpeg_msg;
    img_bridge.toCompressedImageMsg(jpeg_msg, cv_bridge::Format::JPEG);
    // Publish CompressedImage
    pub.publish(jpeg_msg);
    new_map = false;
  }
}

int main( int argc, char** argv ) {
  ros::init( argc, argv, "map_to_jpeg" );
  MapToJpeg map_to_jpeg;
  ros::spin();
  return 0;
}
