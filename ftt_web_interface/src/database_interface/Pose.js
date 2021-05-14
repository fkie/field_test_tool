/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Pose database interaction class.
export class PoseInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "segment_id",
      "lat",
      "lng",
      "pose_source_id",
      "orig_secs",
    ];
  }

  async get(segmentId) {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest(
      "pose",
      "segment_id=" + segmentId
    );
    //Server returns a GeoJSON object in the first position of the response array.
    return data[0];
  }

  async post(segmentId, lat, lng, poseSourceId) {
    //Create entry in server table.
    const data = {
      [this.paramNames[0]]: segmentId,
      [this.paramNames[1]]: lat,
      [this.paramNames[2]]: lng,
      [this.paramNames[3]]: poseSourceId,
      [this.paramNames[4]]: new Date().getTime() / 1000,
    };
    await this.serverInterface.sendPostRequest("pose", data);
  }
}
