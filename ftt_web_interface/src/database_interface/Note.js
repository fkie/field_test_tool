/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Note data container class.
export class Note {
  constructor(id, segmentId, secs, personnelId, note) {
    this.id = id;
    this.segmentId = segmentId;
    this.secs = secs;
    this.personnelId = personnelId;
    this.note = note;
  }
}

//Note database interaction class.
export class NoteInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = ["id", "segment_id", "secs", "personnel_id", "note"];
  }

  async get(segmentId) {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest(
      "note",
      "segment_id=" + segmentId
    );
    return data.map(
      (entry) => new Note(entry[0], entry[1], entry[2], entry[3], entry[4])
    );
  }

  async post(segmentId, personnelId, note) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: segmentId,
      [this.paramNames[3]]: personnelId,
      [this.paramNames[4]]: note,
    };
    await this.serverInterface.sendPostRequest("note", data);
  }

  async put(id, personnelId = null, note = null) {
    //Update entry in server table.
    //Mandatory parameters:
    const data = { [this.paramNames[0]]: id };
    //Optional parameters:
    Object.assign(
      data,
      personnelId !== null && { [this.paramNames[3]]: personnelId },
      note !== null && { [this.paramNames[4]]: note }
    );
    await this.serverInterface.sendPutRequest("note", data);
  }

  async delete(id) {
    //Delete entry in server table.
    await this.serverInterface.sendDeleteRequest("note", "id=" + id);
  }
}
