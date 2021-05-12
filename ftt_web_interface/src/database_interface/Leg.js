/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Leg data container class.
export class Leg {
  constructor(
    id,
    shiftId,
    startTime,
    endTime,
    weatherId,
    defaultPoseSourceId,
    note,
    number
  ) {
    this.id = id;
    this.shiftId = shiftId;
    this.startTime = startTime;
    this.endTime = endTime;
    this.weatherId = weatherId;
    this.defaultPoseSourceId = defaultPoseSourceId;
    this.note = note;
    this.number = number;
  }
}

//Leg database interaction class.
export class LegInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "id",
      "shift_id",
      "starttime_secs",
      "endtime_secs",
      "weather_id",
      "default_pose_source_id",
      "note",
      "number",
    ];
  }

  async get(shiftId) {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest(
      "leg",
      "shift_id=" + shiftId
    );
    return data.map(
      (entry) =>
        new Leg(
          entry[0],
          entry[1],
          entry[2],
          entry[3],
          entry[4],
          entry[5],
          entry[6],
          entry[7]
        )
    );
  }

  async post(shiftId, weatherId, defaultPoseSourceId, note) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: shiftId,
      [this.paramNames[4]]: weatherId,
      [this.paramNames[5]]: defaultPoseSourceId,
      [this.paramNames[6]]: note,
    };
    await this.serverInterface.sendPostRequest("leg", data);
  }

  async put(
    id,
    action,
    weatherId = null,
    defaultPoseSourceId = null,
    note = null
  ) {
    //Update entry in server table.
    //The action parameter can be either "edit" or "close".
    //Mandatory parameters:
    const data = {
      [this.paramNames[0]]: id,
      ["action"]: action,
    };
    //Optional parameters:
    Object.assign(
      data,
      weatherId && { [this.paramNames[4]]: weatherId },
      defaultPoseSourceId && { [this.paramNames[5]]: defaultPoseSourceId },
      note && { [this.paramNames[6]]: note }
    );
    await this.serverInterface.sendPutRequest("leg", data);
  }
}
