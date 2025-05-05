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
  constructor(ros, prefix = "topics") {
    super(ros, prefix);
    this.supportedTopics = {
      robot_mode: "std_msgs/msg/Bool",
      gps_fix: "sensor_msgs/msg/NavSatFix",
      local_pose: "geometry_msgs/msg/Pose",
      map: "nav_msgs/msg/OccupancyGrid",
      image: "sensor_msgs/msg/Image",
      image_compressed: "sensor_msgs/msg/CompressedImage",
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
