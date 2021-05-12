/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//ItoReason data container class.
export class ItoReason {
  constructor(id, shortDescription) {
    this.id = id;
    this.shortDescription = shortDescription;
  }
}

//ItoReason database interaction class.
export class ItoReasonInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = ["id", "short_description"];
  }

  async get() {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest("ito_reason");
    return data.map((entry) => new ItoReason(entry[0], entry[1]));
  }
}
