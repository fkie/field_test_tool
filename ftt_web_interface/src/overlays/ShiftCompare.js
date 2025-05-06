/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2023 - Fraunhofer FKIE
 */

import { SegmentInterface } from "../database_interface/Segment.js";
import { LegInterface } from "../database_interface/Leg.js";
import { LeafletMap } from "../utility/LeafletMap.js";
import {
  // distL2FromLngLat,
  distErp,
  distDtw,
  minimumDistance,
} from "../utility/TrajectoryDistance.js";

const markerColorList = [
  "Navy",
  "Crimson",
  "Green",
  "Violet",
  "Orange",
  "Gold",
];

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
            geoJsonData.features && geoJsonData.features.forEach((feature) => {
              // const coords = feature.geometry.coordinates;
              // for (const p of trajectory) {
              //   if (distL2FromLngLat(p, coords) < 0.5) {
              //     return;
              //   }
              // }
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
    //Get Tracking distances between trajectories
    const distancesMin = Array.from({ length: trajectories.length }).map(() =>
      Array.from({ length: trajectories.length }).fill(0)
    );
    for (let i = 0; i < trajectories.length; i++) {
      for (let j = 0; j < trajectories.length; j++) {
        if (i == j) {
          continue;
        }
        distancesMin[i][j] = minimumDistance(
          trajectories[i],
          trajectories[j]
        );
      }
    }
    //Show distances
    const headerEl = document.querySelector(".modal__title");
    let distCompEl;
    if (trajectories.length < 3) {
      distCompEl = this.createComparisonParagraph(0, 1, distancesMin[0][1], "Minimum");
      headerEl.appendChild(distCompEl);
      distCompEl = this.createComparisonParagraph(1, 0, distancesMin[1][0], "Minimum");
      headerEl.appendChild(distCompEl);
      distCompEl = this.createComparisonParagraph(0, 1, distancesDtw[0][1], "DTW");
      headerEl.appendChild(distCompEl);
      distCompEl = this.createComparisonParagraph(0, 1, distancesErp[0][1], "ERP");
      headerEl.appendChild(distCompEl);
    } else {
      distCompEl = this.createComparisonTable(distancesMin, "Min");
      headerEl.appendChild(distCompEl);
      distCompEl = this.createComparisonTable(distancesDtw, "DTW");
      headerEl.appendChild(distCompEl);
      distCompEl = this.createComparisonTable(distancesErp, "ERP");
      headerEl.appendChild(distCompEl);
    }
  }

  createComparisonParagraph(sourceId, targetId, distance, type) {
    const p = document.createElement("p");
    p.style.marginTop = "0.5rem";
    p.innerHTML = `
    <span style="font-weight:bold">Shift ${this.compareIds[sourceId]} </span>
    <span style="color:${markerColorList[sourceId]}">(${markerColorList[sourceId]})</span>
    to
    <span style="font-weight:bold">Shift ${this.compareIds[targetId]} </span>
    <span style="color:${markerColorList[targetId]}">(${markerColorList[targetId]})</span>
    <span style="font-weight:bold">${type} </span>
    distance:
    <span style="font-weight:bold">${distance.toFixed(2)}m</span>
    `;
    return p;
  }

  createComparisonTable(distances, type) {
    const table = document.createElement("table");
    table.style.marginTop = "0.5rem";
    const tHead = document.createElement("thead");
    const tBody = document.createElement("tbody");
    //Header row
    const tHeaderRow = document.createElement("tr");
    const tHeadTitle = document.createElement("th");
    tHeadTitle.textContent = `${type}`;
    tHeaderRow.appendChild(tHeadTitle);
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
