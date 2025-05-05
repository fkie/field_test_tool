/* eslint-disable no-undef */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Wrap some roslib's service calls in promises. Services:
//  * /ftt_ros/list_parameters (previously /rosapi/get_param_names, but now it's too slow)
//  * /rosapi/get_param
//  * /rosapi/set_param
//  * /rosapi/topic_type
//  * /rosapi/topics_for_type

export function getRosParamNames(ros, prefix) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros: ros,
      name: "/ftt_ros/list_parameters",
      serviceType: "rcl_interfaces/srv/ListParameters",
    });
    const request = {prefixes: [prefix]};
    client.callService(
      request,
      function (result) {
        console.log("Retrieved all ROS parameter names.");
        resolve(result.result.names.map((name) => "/ftt_ros:"+name));
      },
      function (message) {
        reject(new Error(message));
      }
    );
  });
}

export function getRosParam(ros, paramName) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros: ros,
      name: "/rosapi/get_param",
      serviceType: "rosapi_msgs/srv/GetParam",
    });
    const request = {
      name: paramName,
      default_value: "",
    };
    client.callService(
      request,
      function (result) {
        console.log(`Got ROS parameter -> ${paramName}: ${result.value}`);
        resolve(result.value);
      },
      function (message) {
        reject(new Error(message));
      }
    );
  });
}

export function setRosParam(ros, paramName, paramValue) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros: ros,
      name: "/rosapi/set_param",
      serviceType: "rosapi_msgs/srv/SetParam",
    });
    const request = {
      name: paramName,
      value: paramValue,
    };
    client.callService(
      request,
      function (result) {
        console.log(`Set ROS parameter -> ${paramName}: ${paramValue}`);
        resolve(result.value);
      },
      function (message) {
        reject(new Error(message));
      }
    );
  });
}

export function getRosTopicType(ros, topic) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros: ros,
      name: "/rosapi/topic_type",
      serviceType: "rosapi_msgs/srv/TopicType",
    });
    const request = {
      topic: topic,
    };
    client.callService(
      request,
      function (result) {
        console.log(`Got ROS topic type -> ${topic}: ${result.type}`);
        resolve(result.type);
      },
      function (message) {
        reject(new Error(message));
      }
    );
  });
}

export function getRosTopicsForType(ros, topicType) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros: ros,
      name: "/rosapi/topics_for_type",
      serviceType: "rosapi_msgs/srv/TopicsForType",
    });
    const request = {
      type: topicType,
    };
    client.callService(
      request,
      function (result) {
        console.log(`Got all ROS topics of type ${topicType}`);
        resolve(result.topics);
      },
      function (message) {
        reject(new Error(message));
      }
    );
  });
}
