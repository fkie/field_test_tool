/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Image data container class.
export class Image {
  constructor(
    id,
    segmentId,
    secs,
    imageFilename,
    imageData,
    description,
    origSecs
  ) {
    this.id = id;
    this.segmentId = segmentId;
    this.secs = secs;
    this.imageFilename = imageFilename;
    this.imageData = imageData;
    this.description = description;
    this.origSecs = origSecs;
  }
}

//Image database interaction class.
export class ImageInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "id",
      "segment_id",
      "secs",
      "image_filename",
      "image_data",
      "description",
      "orig_secs",
    ];
  }

  async get(segmentId) {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest(
      "image",
      "segment_id=" + segmentId
    );
    return data.map(
      (entry) =>
        new Image(
          entry[0],
          entry[1],
          entry[2],
          entry[3],
          "data:image/jpeg;base64, " + atob(entry[4]),
          entry[5],
          entry[6]
        )
    );
  }

  async post(segmentId, imageFilename, imageData, description) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: segmentId,
      [this.paramNames[3]]: imageFilename,
      [this.paramNames[4]]: imageData,
      [this.paramNames[5]]: description,
      [this.paramNames[6]]: new Date().getTime() / 1000,
    };
    await this.serverInterface.sendPostRequest("image", data);
  }

  async delete(id) {
    //Delete entry in server table.
    await this.serverInterface.sendDeleteRequest("image", "id=" + id);
  }
}
