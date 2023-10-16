/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2023 - Fraunhofer FKIE
 */

import { SegmentInterface } from "../database_interface/Segment.js";
import { LegInterface } from "../database_interface/Leg.js";
import { LeafletMap } from "../utility/LeafletMap.js";
import { distErp, distDtw } from "../utility/TrajectoryDistance.js";

const markerColorList = ["blue", "gold", "violet", "orange", "green", "red"];

//Structure and logic of the map config UI.
export class ShiftCompare {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor(serverInterface, compareIds) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.compareIds = compareIds;
    //Import and build map config node.
    const templateEl = document.getElementById("compare-template");
    this.element = document.importNode(templateEl.content, true);
    //Initialize variables.
    this.legInterface = new LegInterface(serverInterface);
    this.segmentInterface = new SegmentInterface(serverInterface);
    this.mapInterface = null;
    this.compareShifts();
  }

  async compareShifts() {
    let markerColorIdx = 0;
    const trajectories = [];
    //Iterate through shifts.
    for (const shiftId of this.compareIds) {
      //Select the marker color.
      const markerColor = markerColorList[markerColorIdx++];
      //Create an empty array to hold the lng-lat coordinates.
      const trajectory = [];
      //Get legs of each shift.
      const legList = await this.legInterface.get(shiftId);
      const legIds = legList.map((entry) => entry.id);
      for (const legId of legIds) {
        //Get segments of each leg.
        const segmentList = await this.segmentInterface.get(legId);
        for (const entry of segmentList.reverse()) {
          if (!entry.parentId) {
            //Initialize leaflet map.
            if (!this.mapInterface) {
              this.mapInterface = new LeafletMap(
                this.serverInterface,
                "compare-map"
              );
              this.mapInterface.mapElement.style.display = "block";
              const firstLocEntry = segmentList.find(
                (entry) => entry.lat && entry.lng
              );
              this.mapInterface.leafletMap.setView(
                [firstLocEntry.lat, firstLocEntry.lng],
                17
              );
            }
            //Create a new points layer.
            const geoJsonData = await this.mapInterface.getAndDrawMapPoses(
              entry.id,
              markerColor,
              markerColor,
              1.0
            );
            //Get the lng-lat coordinates (at most every 0.5 meter).
            geoJsonData.features.forEach((feature) => {
              trajectory.push(feature.geometry.coordinates);
            });
          }
        }
      }
      trajectories.push(trajectory);
    }
    //Get DTW distance between trajectories
    const distancesDtw = Array.from({ length: trajectories.length }).map(() =>
      Array.from({ length: trajectories.length }).fill(0)
    );
    for (let i = 0; i < trajectories.length - 1; i++) {
      for (let j = i + 1; j < trajectories.length; j++) {
        distancesDtw[i][j] = distDtw(
          trajectories[i],
          trajectories[j],
          trajectories[0][0]
        );
        distancesDtw[j][i] = distancesDtw[i][j];
      }
    }
    //Get ERP distance between trajectories
    const distancesErp = Array.from({ length: trajectories.length }).map(() =>
      Array.from({ length: trajectories.length }).fill(0)
    );
    for (let i = 0; i < trajectories.length - 1; i++) {
      for (let j = i + 1; j < trajectories.length; j++) {
        distancesErp[i][j] = distErp(
          trajectories[i],
          trajectories[j],
          trajectories[0][0]
        );
        distancesErp[j][i] = distancesErp[i][j];
      }
    }
    //Show distances
    const headerEl = document.querySelector(".modal__title");
    let distCompEl;
    if (trajectories.length < 3) {
      distCompEl = this.createComparisonParagraph(distancesDtw[0][1], "DTW");
      headerEl.appendChild(distCompEl);
      distCompEl = this.createComparisonParagraph(distancesErp[0][1], "ERP");
      headerEl.appendChild(distCompEl);
    } else {
      distCompEl = this.createComparisonTable(distancesDtw);
      headerEl.appendChild(distCompEl);
    }
  }

  createComparisonParagraph(distance, type) {
    const p = document.createElement("p");
    p.style.marginTop = "0.5rem";
    p.innerHTML = `
    <span style="font-weight:bold; color:${markerColorList[0]}">Shift ${
      this.compareIds[0]
    } (${markerColorList[0]})</span>
    to
    <span style="font-weight:bold; color:${markerColorList[1]}">Shift ${
      this.compareIds[1]
    } (${markerColorList[1]})</span>
    ${type} distance:
    <span style="font-weight:bold">${distance.toFixed(2)}m</span>
    `;
    return p;
  }

  createComparisonTable(distances) {
    const table = document.createElement("table");
    table.style.marginTop = "0.5rem";
    const tHead = document.createElement("thead");
    const tBody = document.createElement("tbody");
    //Header row
    const tHeaderRow = document.createElement("tr");
    tHeaderRow.appendChild(document.createElement("th"));
    for (const [idx, shiftId] of this.compareIds.entries()) {
      const tHeadData = document.createElement("th");
      tHeadData.textContent = `Shift ${shiftId} (${markerColorList[idx]})`;
      tHeaderRow.appendChild(tHeadData);
    }
    tHead.appendChild(tHeaderRow);
    table.appendChild(tHead);
    //Distance value rows
    this.compareIds.forEach((shiftId1, idx1) => {
      const tRow = document.createElement("tr");
      const tDataLabel = document.createElement("th");
      tDataLabel.textContent = `Shift ${shiftId1}`;
      tRow.appendChild(tDataLabel);
      this.compareIds.forEach((shiftId2, idx2) => {
        const tData = document.createElement("td");
        tData.textContent = distances[idx1][idx2].toFixed(2);
        tRow.appendChild(tData);
      });
      tBody.appendChild(tRow);
    });
    table.appendChild(tBody);
    return table;
  }
}
