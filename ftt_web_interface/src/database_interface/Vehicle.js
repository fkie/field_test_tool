/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Vehicle data container class.
export class Vehicle {
  constructor(
    id,
    key,
    shortDescription,
    longDescription,
    institution,
    configuration
  ) {
    this.id = id;
    this.key = key;
    this.shortDescription = shortDescription;
    this.longDescription = longDescription;
    this.institution = institution;
    this.configuration = configuration;
  }
}

//Vehicle database interaction class.
export class VehicleInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "id",
      "key",
      "short_description",
      "long_description",
      "institution",
      "configuration",
    ];
  }

  async get() {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest("vehicle");
    return data.map(
      (entry) =>
        new Vehicle(
          entry[0],
          entry[1] && entry[1].trim(),
          entry[2] && entry[2].trim(),
          entry[3],
          entry[4] && entry[4].trim(),
          entry[5]
        )
    );
  }

  async post(
    key,
    shortDescription,
    longDescription,
    institution,
    configuration
  ) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: key,
      [this.paramNames[2]]: shortDescription,
      [this.paramNames[3]]: longDescription,
      [this.paramNames[4]]: institution,
      [this.paramNames[5]]: configuration,
    };
    await this.serverInterface.sendPostRequest("vehicle", data);
  }

  async put(
    id,
    key,
    shortDescription,
    longDescription,
    institution,
    configuration
  ) {
    //Update entry in server table.
    //Mandatory parameters:
    const data = { [this.paramNames[0]]: id };
    //Optional parameters:
    Object.assign(
      data,
      key && { [this.paramNames[1]]: key },
      shortDescription && { [this.paramNames[2]]: shortDescription },
      longDescription && { [this.paramNames[3]]: longDescription },
      institution && { [this.paramNames[4]]: institution },
      configuration && { [this.paramNames[5]]: configuration }
    );
    await this.serverInterface.sendPutRequest("vehicle", data);
  }

  async delete(id) {
    //Delete entry in server table.
    await this.serverInterface.sendDeleteRequest("vehicle", "id=" + id);
  }
}
