/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//PoseSource data container class.
export class PoseSource {
  constructor(id, key, shortDescription, longDescription) {
    this.id = id;
    this.key = key;
    this.shortDescription = shortDescription;
    this.longDescription = longDescription;
  }
}

//PoseSource database interaction class.
export class PoseSourceInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = ["id", "key", "short_description", "long_description"];
  }

  async get() {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest("pose_source");
    return data.map(
      (entry) => new PoseSource(entry[0], entry[1], entry[2], entry[3])
    );
  }

  async post(key, shortDescription, longDescription) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: key,
      [this.paramNames[2]]: shortDescription,
      [this.paramNames[3]]: longDescription,
    };
    await this.serverInterface.sendPostRequest("pose_source", data);
  }

  async put(id, key, shortDescription, longDescription) {
    //Update entry in server table.
    //Mandatory parameters:
    const data = { [this.paramNames[0]]: id };
    //Optional parameters:
    Object.assign(
      data,
      key !== null && { [this.paramNames[1]]: key },
      shortDescription !== null && { [this.paramNames[2]]: shortDescription },
      longDescription !== null && { [this.paramNames[3]]: longDescription }
    );
    await this.serverInterface.sendPutRequest("pose_source", data);
  }

  async delete(id) {
    //Delete entry in server table.
    await this.serverInterface.sendDeleteRequest("pose_source", "id=" + id);
  }
}
