/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//LocalPose data container class.
export class LocalPose {
  constructor(id, segmentId, secs, frameId, x, y, type, origSecs) {
    this.id = id;
    this.segmentId = segmentId;
    this.secs = secs;
    this.frameId = frameId;
    this.x = x;
    this.y = y;
    this.type = type;
    this.origSecs = origSecs;
  }
}

//LocalPose database interaction class.
export class LocalPoseInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "id",
      "segment_id",
      "secs",
      "frame_id",
      "x",
      "y",
      "type",
      "orig_secs",
    ];
  }

  async get(segmentId) {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest(
      "local_pose",
      "segment_id=" + segmentId
    );
    return data.map(
      (entry) =>
        new LocalPose(
          entry[0],
          entry[1],
          entry[2],
          entry[3],
          entry[4],
          entry[5],
          entry[6],
          entry[7]
        )
    );
  }
}
