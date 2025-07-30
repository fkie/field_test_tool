/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

/* eslint-disable no-undef */

// Leaflet JS Library:
// https://github.com/Leaflet/Leaflet/blob/master/LICENSE

// © OpenStreetMap contributors:
// https://www.openstreetmap.org/copyright.
// Base map and data from OpenStreetMap and OpenStreetMap Foundation.

import { PoseInterface } from "../database_interface/Pose.js";
import { LocalPoseInterface } from "../database_interface/LocalPose.js";
import { MapImageInterface } from "../database_interface/MapImage.js";

//LeafletMap class to wrap map related variables and functions.
export class LeafletMap {
  constructor(serverInterface, mapElementName, mapContainerName) {
    //Set arguments as properties.
    this.poseInterface = new PoseInterface(serverInterface);
    this.localPoseInterface = new LocalPoseInterface(serverInterface);
    this.mapImageInterface = new MapImageInterface(serverInterface);
    //Initialize variables and objects.
    this.mapPointsLayers = [];
    this.activeMarker = null;
    this.activePoses = null;
    this.leafletMap = L.map(mapElementName);
    this.changeTileLayer();
    this.localMapOverlay = null;
    this.localPosesPolyline = null;
    this.layerControl = null;
    this.mapImage = null;
    this.lnglatCoords = [];
    //Reach to DOM elements.
    this.mapElementContainer = document.getElementById(mapContainerName);
    this.mapElement = document.getElementById(mapElementName);
  }

  changeTileLayer() {
    //Get the stored config data.
    let mapData = JSON.parse(localStorage.getItem("fttTileServerData"));
    //Set default if not found.
    if (!mapData) {
      mapData = {
        url: "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
        minZoom: 0,
        maxZoom: 19,
      };
      //Store data.
      localStorage.setItem("fttTileServerData", JSON.stringify(mapData));
    }
    //Create the tile layer.
    L.tileLayer(mapData.url, {
      minZoom: mapData.minZoom,
      maxZoom: mapData.maxZoom,
    }).addTo(this.leafletMap);
    //Set attribution if using OpenStreetMap
    if (mapData.url.indexOf("openstreetmap") > -1) {
      this.leafletMap.attributionControl.addAttribution(
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      );
    } else {
      this.leafletMap.attributionControl.removeAttribution(
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      );
    }
  }

  matchPosePairs(utmPoses, localPoses, maxTolerance = 0.1) {
    //Find the utm, local pose pairs with the closest timestamp that does not exceed tolerance
    const pairs = [];
    let localIdx = 0;
    for (const utm of utmPoses) {
      let minDiff = Infinity;
      let bestLocal = null;

      while (
        localIdx < localPoses.length - 1 &&
        localPoses[localIdx + 1].timestamp <= utm.timestamp
      ) {
        localIdx++;
      }

      for (let k = localIdx; k <= localIdx + 1 && k < localPoses.length; k++) {
        const local = localPoses[k];
        const diff = Math.abs(utm.timestamp - local.timestamp);
        if (diff < minDiff) {
          minDiff = diff;
          bestLocal = local;
        } else if (diff > minDiff) {
          break;
        }
      }

      if (minDiff <= maxTolerance && bestLocal) {
        pairs.push({ utm, local: bestLocal });
      }
    }
    return pairs;
  }

  // Least squares transformation (Kabsch-Algorithm 2D)
  computeLeastSquaresTransform(pairs) {
    if (pairs.length === 0) return null;
    const n = pairs.length;
    // Compute mean
    let meanUtm = { x: 0, y: 0 },
      meanLocal = { x: 0, y: 0 };
    for (const { utm, local } of pairs) {
      meanUtm.x += utm.x;
      meanUtm.y += utm.y;
      meanLocal.x += local.x;
      meanLocal.y += local.y;
    }
    meanUtm.x /= n;
    meanUtm.y /= n;
    meanLocal.x /= n;
    meanLocal.y /= n;
    // Optimal rotation
    let sx = 0,
      sy = 0;
    for (const { utm, local } of pairs) {
      const ux = utm.x - meanUtm.x,
        uy = utm.y - meanUtm.y;
      const lx = local.x - meanLocal.x,
        ly = local.y - meanLocal.y;
      sx += lx * uy - ly * ux;
      sy += lx * ux + ly * uy;
    }
    const dtheta = Math.atan2(sx, sy);
    // Use optimal rotation to compute the translation
    const cos = Math.cos(dtheta),
      sin = Math.sin(dtheta);
    const tx = meanUtm.x - (cos * meanLocal.x - sin * meanLocal.y);
    const ty = meanUtm.y - (sin * meanLocal.x + cos * meanLocal.y);
    return { dx: tx, dy: ty, dtheta };
  }

  // Apply transform to a point
  applyTransform(pt, transform) {
    const { dx, dy, dtheta } = transform;
    const c = Math.cos(dtheta);
    const s = Math.sin(dtheta);
    return {
      x: c * pt.x - s * pt.y + dx,
      y: s * pt.x + c * pt.y + dy,
    };
  }

  // Convert a point (x,y) to (lat, lng)
  toLatLng(pt, latMid, lngMid, mPerLat, mPerLng) {
    return {
      lat: latMid + pt.y / mPerLat,
      lng: lngMid + pt.x / mPerLng,
    };
  }

  // Utility: Compute squared distance
  pointDist(a, b) {
    return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
  }

  // RANSAC robust estimation
  robustTransform(pairs, threshold = 2.0) {
    if (pairs.length < 2) {
      throw new Error("At least two pairs required");
    }
    let bestInliers = [];
    let bestTransform = null;

    for (let t = 0; t < pairs.length; t++) {
      // Randomly pick pairs
      const idx = [];
      while (idx.length < Math.min(pairs.length / 10, 2)) {
        const r = Math.floor(Math.random() * pairs.length);
        if (!idx.includes(r)) idx.push(r);
      }
      const selectedPairs = idx.map((i) => pairs[i]);
      const tf = this.computeLeastSquaresTransform(selectedPairs);

      // Count inliers
      const inliers = [];
      for (let i = 0; i < pairs.length; i++) {
        const pred = this.applyTransform(pairs[i].local, tf);
        if (this.pointDist(pred, pairs[i].utm) < threshold) {
          inliers.push(i);
        }
      }
      if (inliers.length > bestInliers.length) {
        bestInliers = inliers;
        bestTransform = tf;
      }
    }

    // Re-estimate transform using all inliers
    if (bestInliers.length >= 2) {
      const inlierPairs = bestInliers.map((i) => pairs[i]);
      bestTransform = this.computeLeastSquaresTransform(inlierPairs);
    }
    return bestTransform;
  }

  async addLocalMap(mapImage, fSegs) {
    // //If there was a local map with the same parameters, update the image and return
    // if (
    //   this.localMapOverlay &&
    //   this.mapImage.width === mapImage.width &&
    //   this.mapImage.height === mapImage.height &&
    //   this.mapImage.resolution === mapImage.resolution
    // ) {
    //   this.localMapOverlay._rawImage.src = mapImage.imageData;
    //   return;
    // }
    // //Check the filtered segment list (segments with both gps and local positions) has enough values
    // if (fSegs.length < 2) {
    //   return;
    // }

    //Check there is enough lnglat coordinates
    if (this.lnglatCoords.length < 2) {
      return;
    }
    //Assign mapImage data to html image.
    const image = new Image();
    image.src = mapImage.imageData;
    //Get local coordinates
    const localPoses = [];
    for (const segment of fSegs) {
      const poses = await this.localPoseInterface.get(segment.id);
      for (const pose of poses) {
        localPoses.push({ x: pose.x, y: pose.y, timestamp: pose.origSecs });
      }
    }
    //Check there is enough local coordinates
    if (localPoses.length < 2) {
      return;
    }
    //Get the mean lng and lat coordinates for the currently used poses
    // const latMid = ((fSegs[1].lat + fSegs[0].lat) / 2) * (Math.PI / 180);
    const sum = this.lnglatCoords.reduce(
      (acc, obj) => {
        acc.lat += obj.lat;
        acc.lng += obj.lng;
        return acc;
      },
      { lat: 0, lng: 0 }
    );
    const latMid = sum.lat / this.lnglatCoords.length;
    const lngMid = sum.lng / this.lnglatCoords.length;
    //Estimate differences of lat and lng degrees in meters.
    //https://en.wikipedia.org/wiki/Geographic_coordinate_system
    const mPerLat =
      111132.92 -
      559.82 * Math.cos(2 * latMid * (Math.PI / 180)) +
      1.175 * Math.cos(4 * latMid * (Math.PI / 180)) -
      0.0023 * Math.cos(6 * latMid * (Math.PI / 180));
    const mPerLng =
      111412.84 * Math.cos(latMid * (Math.PI / 180)) -
      93.5 * Math.cos(3 * latMid * (Math.PI / 180)) +
      0.118 * Math.cos(5 * latMid * (Math.PI / 180));
    //Calculate cartesian coordinates from lnglat
    const utmPoses = this.lnglatCoords.map((coords) => ({
      x: (coords.lng - lngMid) * mPerLng,
      y: (coords.lat - latMid) * mPerLat,
      timestamp: coords.timestamp,
    }));
    //Match utm-like and local pose coordinates
    const matchedCoordPairs = this.matchPosePairs(utmPoses, localPoses);
    //Check there is enough matched coordinates
    if (matchedCoordPairs.length < 2) {
      return;
    }
    //Calculate the best transform between the pairs
    const tf = this.robustTransform(matchedCoordPairs);
    // //Estimate image rotation with respect to cardinal orientation.
    // const rotation =
    //   Math.atan2(
    //     (fSegs[1].lat - fSegs[0].lat) * mPerLat,
    //     (fSegs[1].lng - fSegs[0].lng) * mPerLng
    //   ) -
    //   Math.atan2(
    //     fSegs[1].local_y - fSegs[0].local_y,
    //     fSegs[1].local_x - fSegs[0].local_x
    //   );
    //Calculate the local coordinates of the top and bottom-left image corners
    const bottomLeft = {
      x: mapImage.originX,
      y: mapImage.originY,
    };
    const topLeft = {
      x: mapImage.originX,
      y: mapImage.originY + mapImage.height * mapImage.resolution,
    };
    const topRight = {
      x: mapImage.originX + mapImage.width * mapImage.resolution,
      y: mapImage.originY + mapImage.height * mapImage.resolution,
    };
    //Calculate the transformed image corners
    const bottomLeftRotD = this.applyTransform(bottomLeft, tf);
    const topLeftRotD = this.applyTransform(topLeft, tf);
    const topRightRotD = this.applyTransform(topRight, tf);
    //Calculate the transformed image corners in lat-lng.
    const bottomLeftLatLng = this.toLatLng(
      bottomLeftRotD,
      latMid,
      lngMid,
      mPerLat,
      mPerLng
    );
    const topLeftLatLng = this.toLatLng(
      topLeftRotD,
      latMid,
      lngMid,
      mPerLat,
      mPerLng
    );
    const topRightLatLng = this.toLatLng(
      topRightRotD,
      latMid,
      lngMid,
      mPerLat,
      mPerLng
    );
    //If a local map and poses were already added, remember they were displayed and delete them.
    const displayMap = !!this.localMapOverlay && this.leafletMap.hasLayer(this.localMapOverlay);
    const displayPoses = !!this.localPosesPolyline && this.leafletMap.hasLayer(this.localPosesPolyline);
    this.removeLocalMap();
    //Create the overlay for the rotated image.
    this.localMapOverlay = L.imageOverlay.rotated(
      image,
      topLeftLatLng,
      topRightLatLng,
      bottomLeftLatLng,
      {
        opacity: 0.4,
        interactive: true,
      }
    );
    //Create a polyline of the local poses
    this.localPosesPolyline = L.polyline(
      localPoses.map((pose) =>
        this.toLatLng(
          this.applyTransform(pose, tf),
          latMid,
          lngMid,
          mPerLat,
          mPerLng
        )
      ),
      {
        color: "orange",
        opacity: 0.4,
      }
    );
    //If a map was previously displayed, add the new one.
    if (displayMap) {
      this.localMapOverlay.addTo(this.leafletMap);
    }
    //If local poses were previously displayed, add the new ones.
    if (displayPoses) {
      this.localPosesPolyline.addTo(this.leafletMap);
    }
    //Create the layer control if there is none.
    if (!this.layerControl) {
      this.layerControl = L.control.layers().addTo(this.leafletMap);
    }
    //Add the overlays to the layer control
    this.layerControl.addOverlay(
      this.localMapOverlay,
      "Local map"
    );
    //Add the overlays to the layer control
    this.layerControl.addOverlay(
      this.localPosesPolyline,
      "Local poses (orange)"
    );
    //Save the last mapImage
    this.mapImage = mapImage;
  }

  removeLocalMap() {
    if (this.localMapOverlay) {
      this.leafletMap.removeLayer(this.localMapOverlay);
      this.layerControl.removeLayer(this.localMapOverlay);
      this.localMapOverlay = null;
    }
    if (this.localPosesPolyline) {
      this.localPosesPolyline.remove();
      this.layerControl.removeLayer(this.localPosesPolyline);
      this.localPosesPolyline = null;
    }
    this.mapImage = null;
  }

  removeLayerControl() {
    if (this.layerControl) {
      this.leafletMap.removeControl(this.layerControl);
      this.layerControl = null;
    }
  }

  removePoses(segmentId) {
    //Find the map layer with the poses of the specified segment id.
    const layerIndex = this.mapPointsLayers.findIndex(
      (entry) => entry.segmentId == segmentId
    );
    if (layerIndex > -1) {
      const layerPosesPtr = this.mapPointsLayers[layerIndex].mapPosesPtr;
      //Remove the held variable if the found poses were active (selected).
      if (layerPosesPtr == this.activePoses) {
        this.activePoses = null;
      }
      //Remove the found poses from the map.
      this.leafletMap.removeLayer(layerPosesPtr);
      this.mapPointsLayers.splice(layerIndex, 1);
    }
  }

  async getAndDrawMapPoses(
    segmentId,
    markerColorAuto,
    markerColorIto,
    markerAlpha
  ) {
    let geoJsonData = null;
    try {
      //Get geoJSON data from server.
      geoJsonData = await this.poseInterface.get(segmentId);
      //Remove any data from the specified segment that was already on the map.
      this.removePoses(segmentId);
      if (geoJsonData.features) {
        //Show map and force tile fetch.
        if (this.mapElementContainer) {
          this.mapElementContainer.style.display = "block";
        }
        this.leafletMap.invalidateSize(true);
        //Set poses map display options.
        let markerOptions = {
          radius: 2,
          weight: 1,
          opacity: 1 * markerAlpha,
          fillOpacity: 0.8 * markerAlpha,
        };
        //Sort poses
        geoJsonData.features.sort((a, b) => a.properties.id - b.properties.id);
        //Add poses to map, while setting special options and pop-up with data from geoJSON data features.
        const points = L.geoJSON(geoJsonData, {
          pointToLayer: (feature, latlng) => {
            //Save the lnglat coordinates
            this.lnglatCoords.push({
              lat: latlng.lat,
              lng: latlng.lng,
              timestamp: feature.properties.timestamp,
            });
            //Generate the markers for the map
            if (feature.properties.type == "AUTO") {
              markerOptions.color = markerColorAuto || "green";
            } else {
              markerOptions.color = markerColorIto || "red";
            }
            return L.circleMarker(latlng, markerOptions);
          },
          onEachFeature: function (feature, layer) {
            if (feature.properties && feature.properties.segmentId) {
              layer.bindPopup(
                "<span>Segment " + feature.properties.segmentId + "</span>"
              );
            }
          },
        }).addTo(this.leafletMap);
        //Save segment id and pointer to map layer in a list.
        this.mapPointsLayers.push({
          segmentId: segmentId,
          mapPosesPtr: points,
        });
      } else {
        console.log("No position data for segment " + segmentId);
      }
    } catch (error) {
      console.log(error.message);
    }
    return geoJsonData;
  }

  removeActiveMarker() {
    //Remove location marker from map.
    if (this.activeMarker) {
      this.leafletMap.removeLayer(this.activeMarker);
      this.activeMarker = null;
    }
  }

  removeActivePoses() {
    //Restore point coloring.
    if (this.activePoses) {
      this.activePoses.eachLayer((layer) => {
        if (layer.feature.properties.type == "AUTO") {
          layer.setStyle({ color: "green" });
        } else {
          layer.setStyle({ color: "red" });
        }
      });
      this.activePoses = null;
    }
  }

  addActiveMarker(lat, lng) {
    //Add location marker to map.
    this.activeMarker = L.marker([lat, lng]).addTo(this.leafletMap);
    if (this.leafletMap.getZoom()) {
      this.leafletMap.panTo([lat, lng]);
    } else {
      console.log("Cannot pan Leaflet map before loading it.");
    }
  }

  addActivePoses(segmentId) {
    //Change point coloring to blue for poses of the associated segment.
    const selectedPoses = this.mapPointsLayers.find(
      (poses) => poses.segmentId == segmentId
    );
    if (selectedPoses) {
      this.activePoses = selectedPoses.mapPosesPtr;
      this.activePoses.setStyle({ color: "blue" });
    }
  }
}
