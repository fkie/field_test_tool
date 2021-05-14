/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Performer data container class.
export class Performer {
  constructor(id, institution) {
    this.id = id;
    this.institution = institution;
  }
}

//Performer database interaction class.
export class PerformerInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = ["id", "institution"];
  }

  async get() {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest("performer");
    return data.map((entry) => new Performer(entry[0], entry[1]));
  }

  async post(institution) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: institution,
    };
    await this.serverInterface.sendPostRequest("performer", data);
  }

  async put(id, institution) {
    //Update entry in server table.
    //Mandatory parameters:
    const data = { [this.paramNames[0]]: id };
    //Optional parameters:
    Object.assign(data, institution && { [this.paramNames[1]]: institution });
    await this.serverInterface.sendPutRequest("performer", data);
  }

  async delete(id) {
    //Delete entry in server table.
    await this.serverInterface.sendDeleteRequest("performer", "id=" + id);
  }
}
