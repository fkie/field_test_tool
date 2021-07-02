/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//TestEvent data container class.
export class TestEvent {
  constructor(id, startTime, endTime, location, version, timeZone, note) {
    this.id = id;
    this.startTime = startTime;
    this.endTime = endTime;
    this.location = location;
    this.version = version;
    this.timeZone = timeZone;
    this.note = note;
  }
}

//TestEvent database interaction class.
export class TestEventInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "id",
      "starttime_secs",
      "endtime_secs",
      "location",
      "version",
      "time_zone",
      "note",
    ];
  }

  async get() {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest("test_event");
    return data.map(
      (entry) =>
        new TestEvent(
          entry[0],
          entry[1],
          entry[2],
          entry[3],
          entry[4],
          entry[5],
          entry[6]
        )
    );
  }

  async post(location, version, timeZone, note) {
    //Create entry in server table.
    const data = {
      [this.paramNames[3]]: location,
      [this.paramNames[4]]: version,
      [this.paramNames[5]]: timeZone,
      [this.paramNames[6]]: note,
    };
    await this.serverInterface.sendPostRequest("test_event", data);
  }

  async put(
    id,
    action,
    location = null,
    version = null,
    timeZone = null,
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
      location !== null && { [this.paramNames[3]]: location },
      version !== null && { [this.paramNames[4]]: version },
      timeZone !== null && { [this.paramNames[5]]: timeZone },
      note !== null && { [this.paramNames[6]]: note }
    );
    await this.serverInterface.sendPutRequest("test_event", data);
  }
}
