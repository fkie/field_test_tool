/* eslint-disable no-undef */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { RosParamsInterface } from "./Parameters.js";
import { getRosTopicsForType } from "./roslibExtension.js";

//ROS topics interaction class.
//Interaction with ROS topics is done here through parameters, but their edition is conditioned to available topics.
export class RosTopicsInterface extends RosParamsInterface {
  constructor(ros, prefix = "/ftt_ros/topics/") {
    super(ros, prefix);
    this.supportedTopics = {
      robot_mode: "industrial_msgs/RobotMode",
      gps_fix: "sensor_msgs/NavSatFix",
      local_pose: "geometry_msgs/Pose",
      map: "nav_msgs/OccupancyGrid",
      image: "sensor_msgs/Image",
      image_compressed: "sensor_msgs/CompressedImage",
    };
  }

  async getOptions(paramName) {
    //Get topic type.
    const topicType = this.supportedTopics[paramName];
    //Get all topics of type.
    const topicsForType = await getRosTopicsForType(this.ros, topicType);
    return topicsForType.map((topicName) => '"' + topicName + '"');
  }
}
