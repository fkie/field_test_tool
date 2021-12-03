/* eslint-disable no-undef */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Wrap roslib's service calls in promises. Services:
//  * get_param_names
//  * get_param
//  * set_param
//  * topic_type
//  * topics_for_type

export function getRosParamNames(ros) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros: ros,
      name: "/rosapi/get_param_names",
      serviceType: "rosapi/GetParamNames",
    });
    const request = new ROSLIB.ServiceRequest();
    client.callService(
      request,
      function (result) {
        console.log("Retrieved all ROS parameter names.");
        resolve(result.names);
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
      serviceType: "rosapi/GetParam",
    });
    const request = new ROSLIB.ServiceRequest({
      name: paramName,
      default: "",
    });
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
      serviceType: "rosapi/SetParam",
    });
    const request = new ROSLIB.ServiceRequest({
      name: paramName,
      value: paramValue,
    });
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
      serviceType: "rosapi/TopicType",
    });
    const request = new ROSLIB.ServiceRequest({
      topic: topic,
    });
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
      serviceType: "rosapi/TopicsForType",
    });
    const request = new ROSLIB.ServiceRequest({
      type: topicType,
    });
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
