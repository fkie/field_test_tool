/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Shift data container class.
export class Shift {
  constructor(
    id,
    testEventId,
    startTime,
    endTime,
    testAdministratorId,
    testDirectorId,
    safetyOfficerId,
    robotOperatorId,
    performerId,
    testIntent,
    workspace,
    vehicleId,
    note,
    number
  ) {
    this.id = id;
    this.testEventId = testEventId;
    this.startTime = startTime;
    this.endTime = endTime;
    this.testAdministratorId = testAdministratorId;
    this.testDirectorId = testDirectorId;
    this.safetyOfficerId = safetyOfficerId;
    this.robotOperatorId = robotOperatorId;
    this.performerId = performerId;
    this.testIntent = testIntent;
    this.workspace = workspace;
    this.vehicleId = vehicleId;
    this.note = note;
    this.number = number;
  }
}

//Shift database interaction class.
export class ShiftInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "id",
      "test_event_id",
      "starttime_secs",
      "endtime_secs",
      "test_administrator_id",
      "test_director_id",
      "safety_officer_id",
      "robot_operator_id",
      "performer_id",
      "test_intent",
      "workspace",
      "vehicle_id",
      "note",
      "number",
    ];
  }

  async get(testEventId) {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest(
      "shift",
      "test_event_id=" + testEventId
    );
    return data.map(
      (entry) =>
        new Shift(
          entry[0],
          entry[1],
          entry[2],
          entry[3],
          entry[4],
          entry[5],
          entry[6],
          entry[7],
          entry[8],
          entry[9] && entry[9].trim(),
          entry[10] && entry[10].trim(),
          entry[11],
          entry[12],
          entry[13]
        )
    );
  }

  async post(
    testEventId,
    testAdministratorId,
    testDirectorId,
    safetyOfficerId,
    robotOperatorId,
    performerId,
    testIntent,
    workspace,
    vehicleId,
    note
  ) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: testEventId,
      [this.paramNames[4]]: testAdministratorId,
      [this.paramNames[5]]: testDirectorId,
      [this.paramNames[6]]: safetyOfficerId,
      [this.paramNames[7]]: robotOperatorId,
      [this.paramNames[8]]: performerId,
      [this.paramNames[9]]: testIntent,
      [this.paramNames[10]]: workspace,
      [this.paramNames[11]]: vehicleId,
      [this.paramNames[12]]: note,
    };
    await this.serverInterface.sendPostRequest("shift", data);
  }

  async put(
    id,
    action,
    testAdministratorId = null,
    testDirectorId = null,
    safetyOfficerId = null,
    robotOperatorId = null,
    performerId = null,
    testEventId = null,
    workspace = null,
    vehicleId = null,
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
      testAdministratorId && { [this.paramNames[4]]: testAdministratorId },
      testDirectorId && { [this.paramNames[5]]: testDirectorId },
      safetyOfficerId && { [this.paramNames[6]]: safetyOfficerId },
      robotOperatorId && { [this.paramNames[7]]: robotOperatorId },
      performerId && { [this.paramNames[8]]: performerId },
      testEventId && { [this.paramNames[9]]: testEventId },
      workspace && { [this.paramNames[10]]: workspace },
      vehicleId && { [this.paramNames[11]]: vehicleId },
      note && { [this.paramNames[12]]: note }
    );
    await this.serverInterface.sendPutRequest("shift", data);
  }
}
