/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Segment data container class.
export class Segment {
  constructor(
    id,
    legId,
    parentId,
    startTime,
    endTime,
    itoReasonId,
    itoReason,
    segmentTypeId,
    segmentType,
    obstacle,
    lighting,
    slope,
    lng,
    lat,
    local_x,
    local_y,
    origStartTime
  ) {
    this.id = id;
    this.legId = legId;
    this.parentId = parentId;
    this.startTime = startTime;
    this.endTime = endTime;
    this.itoReasonId = itoReasonId;
    this.itoReason = itoReason;
    this.segmentTypeId = segmentTypeId;
    this.segmentType = segmentType;
    this.obstacle = obstacle;
    this.lighting = lighting;
    this.slope = slope;
    this.lng = lng;
    this.lat = lat;
    this.local_x = local_x;
    this.local_y = local_y;
    this.origStartTime = origStartTime;
  }
}

//Segment database interaction class.
export class SegmentInterface {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.paramNames = [
      "id",
      "leg_id",
      "parent_id",
      "starttime_secs",
      "endtime_secs",
      "ito_reason_id",
      "ito_reason",
      "segment_type_id",
      "segment_type",
      "obstacle",
      "lighting",
      "slope",
      "lng",
      "lat",
      "local_x",
      "local_y",
      "orig_starttime_secs",
    ];
  }

  async get(legId) {
    //Get table data from server.
    const data = await this.serverInterface.sendGetRequest(
      "segment",
      "leg_id=" + legId
    );
    return data.map(
      (entry) =>
        new Segment(
          entry[0],
          entry[1],
          entry[2],
          entry[3],
          entry[4],
          entry[5],
          entry[6],
          entry[7],
          entry[8],
          entry[9],
          entry[10],
          entry[11],
          entry[12],
          entry[13],
          entry[14],
          entry[15],
          entry[16]
        )
    );
  }

  async post(legId, itoReasonId, segmentTypeId, lng, lat, local_x, local_y) {
    //Create entry in server table.
    const data = {
      [this.paramNames[1]]: legId,
      [this.paramNames[5]]: itoReasonId,
      [this.paramNames[7]]: segmentTypeId,
      [this.paramNames[12]]: lng,
      [this.paramNames[13]]: lat,
      [this.paramNames[14]]: local_x,
      [this.paramNames[15]]: local_y,
      [this.paramNames[16]]: new Date().getTime() / 1000,
    };
    await this.serverInterface.sendPostRequest("segment", data);
  }

  async put(
    id,
    action,
    itoReasonId = null,
    obstacle = null,
    lighting = null,
    slope = null
  ) {
    //Update entry in server table.
    //The action parameter can be either "edit" or "close".
    //Mandatory parameters:
    const data = {
      [this.paramNames[0]]: id,
      ["action"]: action,
    };
    //Optional parameters:
    Object.assign(
      data,
      itoReasonId && { [this.paramNames[5]]: itoReasonId },
      obstacle && { [this.paramNames[9]]: obstacle },
      lighting && { [this.paramNames[10]]: lighting },
      slope && { [this.paramNames[11]]: slope }
    );
    //Update entry in server table.
    await this.serverInterface.sendPutRequest("segment", data);
  }
}
