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

//LeafletMap class to wrap map related variables and functions.
export class LeafletMap {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.poseInterface = new PoseInterface(serverInterface);
    //Initialize variables and objects.
    this.mapPointsLayers = [];
    this.activeMarker = null;
    this.activePoses = null;
    this.leafletMap = L.map("gps-map");
    this.changeTileLayer();
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
      this.leafletMap.attributionControl.addAttribution('&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors');
    } else {
      this.leafletMap.attributionControl.removeAttribution('&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors');
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
