/* eslint-disable no-undef */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import {
  getRosParamNames,
  getRosParam,
  setRosParam,
} from "./roslibExtension.js";

//ROS parameters interaction class.
export class RosParamsInterface {
  constructor(ros, prefix = "params") {
    //Set arguments as properties.
    this.ros = ros;
    //Initialize variables.
    this.paramNames = [
      "name",
      "value"
    ];
    this.fttRosParamNamePrefix = prefix;
    this.fttRosParamNames = null;
  }

  async getFttRosParamNames() {
    if (!this.fttRosParamNames) {
      //Get parameters from ROS.
      this.fttRosParamNames = await getRosParamNames(this.ros, this.fttRosParamNamePrefix);
    }
  }

  async get() {
    //Ensure the fttRosParamNames property is initialized.
    await this.getFttRosParamNames()
    //Get the value of each parameter and construct a list of objects.
    const fttParams = [];
    for (const name of this.fttRosParamNames) {
      fttParams.push({
        name: name.split(/[/:.]/).pop(),
        value: await getRosParam(this.ros, name),
      });
    }
    //Return the object with the parameters.
    return fttParams;
  }

  async put(paramName, paramValue) {
    //Ensure the fttRosParamNames property is initialized.
    await this.getFttRosParamNames()
    //Find complete name of the requested parameter.
    const rosParamName = this.fttRosParamNames.find(name => name.includes(paramName))
    //Update param in ros parameter server.
    await setRosParam(this.ros, rosParamName, paramValue);
  }
}
