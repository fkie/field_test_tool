/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//MapImage data container class.
export class MapImage {
  constructor(
    id,
    shiftId,
    secs,
    frameId,
    width,
    height,
    resolution,
    originX,
    originY,
    imageData,
    origSecs
  ) {
    this.id = id;
    this.shiftId = shiftId;
    this.secs = secs;
    this.frameId = frameId;
    this.width = width;
    this.height = height;
    this.resolution = resolution;
    this.originX = originX;
    this.originY = originY;
    this.imageData = imageData;
    this.origSecs = origSecs;
  }
}

//MapImage database interaction class.
export class MapImageInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "id",
      "shift_id",
      "secs",
      "frame_id",
      "width",
      "height",
      "resolution",
      "origin_x",
      "origin_y",
      "image_data",
      "orig_secs",
    ];
  }

  async get(testEventId) {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest(
      "map_image",
      "shift_id=" + testEventId
    );
    return new MapImage(
      data[0],
      data[1],
      data[2],
      data[3],
      data[4],
      data[5],
      data[6],
      data[7],
      data[8],
      "data:image/jpeg;base64, " + atob(data[9]),
      data[10]
    );
  }
}
