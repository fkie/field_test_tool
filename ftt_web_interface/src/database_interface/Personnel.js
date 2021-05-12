/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Personnel data container class.
export class Personnel {
  constructor(id, name, institution) {
    this.id = id;
    this.name = name;
    this.institution = institution;
  }
}

//Personnel database interaction class.
export class PersonnelInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = ["id", "name", "institution"];
  }

  async get() {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest("personnel");
    return data.map((entry) => new Personnel(entry[0], entry[1], entry[2]));
  }

  async post(name, institution) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: name,
      [this.paramNames[2]]: institution,
    };
    await this.serverInterface.sendPostRequest("personnel", data);
  }

  async put(id, name, institution) {
    //Update entry in server table.
    //Mandatory parameters:
    const data = { [this.paramNames[0]]: id };
    //Optional parameters:
    Object.assign(
      data,
      name && { [this.paramNames[1]]: name },
      institution && { [this.paramNames[2]]: institution }
    );
    await this.serverInterface.sendPutRequest("personnel", data);
  }

  async delete(id) {
    //Delete entry in server table.
    await this.serverInterface.sendDeleteRequest("personnel", "id=" + id);
  }
}
