/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Weather data container class.
export class Weather {
  constructor(id, shortDescription) {
    this.id = id;
    this.shortDescription = shortDescription;
  }
}

//Weather database interaction class.
export class WeatherInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = ["id", "short_description"];
  }

  async get() {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest("weather");
    return data.map(
      (entry) => new Weather(entry[0], entry[1] && entry[1].trim())
    );
  }
}
