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
import { MapImageInterface } from "../database_interface/MapImage.js";

//LeafletMap class to wrap map related variables and functions.
export class LeafletMap {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.poseInterface = new PoseInterface(serverInterface);
    this.mapImageInterface = new MapImageInterface(serverInterface);
    //Initialize variables and objects.
    this.mapPointsLayers = [];
    this.activeMarker = null;
    this.activePoses = null;
    this.leafletMap = L.map("gps-map");
    this.changeTileLayer();
    this.localMapOverlay = null;
    this.layerControl = null;
    this.mapImage = null;
    //Reach to DOM elements.
    this.mapElementContainer = document.getElementById("map-viewer");
    this.mapElement = document.getElementById("gps-map");
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

  async addLocalMap(mapImage, fSegs) {
    //If there was a local map with the same parameters, update the image and return
    if (
      this.localMapOverlay &&
      this.mapImage.width === mapImage.width &&
      this.mapImage.height === mapImage.height &&
      this.mapImage.resolution === mapImage.resolution
    ) {
      this.localMapOverlay._rawImage.src = mapImage.imageData;
      return;
    }
    //Check the filtered segment list (segments with both gps and local positions) has enough values
    if (fSegs.length < 2) {
      return;
    }
    //Assign mapImage data to html image.
    const image = new Image();
    image.src = mapImage.imageData;
    //Estimate differences of lat and lng degrees in meters.
    //https://en.wikipedia.org/wiki/Geographic_coordinate_system
    const latMid = ((fSegs[1].lat + fSegs[0].lat) / 2) * (Math.PI / 180);
    const mPerLat =
      111132.92 -
      559.82 * Math.cos(2 * latMid) +
      1.175 * Math.cos(4 * latMid) -
      0.0023 * Math.cos(6 * latMid);
    const mPerLng =
      111412.84 * Math.cos(latMid) -
      93.5 * Math.cos(3 * latMid) +
      0.118 * Math.cos(5 * latMid);
    //Estimate image rotation with respect to cardinal orientation.
    const rotation =
      Math.atan2(
        (fSegs[1].lat - fSegs[0].lat) * mPerLat,
        (fSegs[1].lng - fSegs[0].lng) * mPerLng
      ) -
      Math.atan2(
        fSegs[1].local_y - fSegs[0].local_y,
        fSegs[1].local_x - fSegs[0].local_x
      );
    //Get the local coordinates of the top and bottom-left image corners
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
    //Get the rotated distance vectors (in cardinal orientation) between the corners and the first segment's position
    const bottomLeftRotD = {
      x:
        (bottomLeft.x - fSegs[0].local_x) * Math.cos(rotation) -
        (bottomLeft.y - fSegs[0].local_y) * Math.sin(rotation),
      y:
        (bottomLeft.x - fSegs[0].local_x) * Math.sin(rotation) +
        (bottomLeft.y - fSegs[0].local_y) * Math.cos(rotation),
    };
    const topLeftRotD = {
      x:
        (topLeft.x - fSegs[0].local_x) * Math.cos(rotation) -
        (topLeft.y - fSegs[0].local_y) * Math.sin(rotation),
      y:
        (topLeft.x - fSegs[0].local_x) * Math.sin(rotation) +
        (topLeft.y - fSegs[0].local_y) * Math.cos(rotation),
    };
    const topRightRotD = {
      x:
        (topRight.x - fSegs[0].local_x) * Math.cos(rotation) -
        (topRight.y - fSegs[0].local_y) * Math.sin(rotation),
      y:
        (topRight.x - fSegs[0].local_x) * Math.sin(rotation) +
        (topRight.y - fSegs[0].local_y) * Math.cos(rotation),
    };
    //Get the rotated image corners in lat-lng.
    const bottomLeftLatLng = L.latLng(
      fSegs[0].lat + bottomLeftRotD.y / mPerLat,
      fSegs[0].lng + bottomLeftRotD.x / mPerLng
    );
    const topLeftLatLng = L.latLng(
      fSegs[0].lat + topLeftRotD.y / mPerLat,
      fSegs[0].lng + topLeftRotD.x / mPerLng
    );
    const topRightLatLng = L.latLng(
      fSegs[0].lat + topRightRotD.y / mPerLat,
      fSegs[0].lng + topRightRotD.x / mPerLng
    );
    //If a different local map was already added, remember if was displayed and then delete it.
    let displayMap = false;
    if (this.localMapOverlay) {
      displayMap = this.leafletMap.hasLayer(this.localMapOverlay);
      this.removeLocalMap();
    }
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
    //If a previous map was displayed, add the overlay to the map.
    if (displayMap) {
      this.localMapOverlay.addTo(this.leafletMap);
    }
    //Create the layer control if there is none.
    if (!this.layerControl) {
      this.layerControl = L.control.layers().addTo(this.leafletMap);
    }
    //Add the overlay to the layer control
    this.layerControl.addOverlay(this.localMapOverlay, "Local Map");
    //Save the last mapImage
    this.mapImage = mapImage;
  }

  removeLocalMap() {
    if (this.localMapOverlay) {
      this.leafletMap.removeLayer(this.localMapOverlay);
      this.layerControl.removeLayer(this.localMapOverlay);
      this.localMapOverlay = null;
      this.mapImage = null;
    }
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

  async getAndDrawMapPoses(segmentId) {
    try {
      //Get geoJSON data from server.
      const geoJsonData = await this.poseInterface.get(segmentId);
      //Remove any data from the specified segment that was already on the map.
      this.removePoses(segmentId);
      if (geoJsonData.features) {
        //Show map and force tile fetch.
        this.mapElementContainer.style.display = "block";
        this.leafletMap.invalidateSize(true);
        //Set poses map display options.
        let markerOptions = {
          radius: 2,
          weight: 1,
          opacity: 1,
          fillOpacity: 0.8,
        };
        //Add poses to map, while setting special options and pop-up with data from geoJSON data features.
        const points = L.geoJSON(geoJsonData, {
          pointToLayer: function (feature, latlng) {
            if (feature.properties.type == "AUTO") {
              markerOptions.color = "green";
            } else {
              markerOptions.color = "red";
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
