/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

/* eslint-disable no-undef */

import { MapImageInterface } from "../database_interface/MapImage.js";
import { LocalPoseInterface } from "../database_interface/LocalPose.js";

//LocalMap class to wrap ROS local occupancy grid map related variables and functions.
export class LocalMap {
  constructor(serverInterface) {
    //Create api interface.
    this.mapImageInterface = new MapImageInterface(serverInterface);
    this.localPoseInterface = new LocalPoseInterface(serverInterface);
    //Reach to DOM elements.
    this.mapElement = document.getElementById("local-map");
    this.zoomInBtn = document.getElementById("local-map-zoom-in-btn");
    this.zoomOutBtn = document.getElementById("local-map-zoom-out-btn");
    //Initialize variables and objects.
    this.mapPointsLayers = [];
    this.activeMarker = null;
    this.activePoses = null;
    this.mapBitmap = null;
    //Create the main viewer.
    this.viewer = new ROS2D.Viewer({
      divID: "local-map",
      width: 560,
      height: 560,
    });
    //Reserve first index of viewer scene for map.
    this.viewer.scene.addChild(new createjs.Shape());
    //Add zoom to the viewer.
    this.zoomView = new ROS2D.ZoomView({
      rootObject: this.viewer.scene,
    });
    //Add panning to the viewer.
    this.panView = new ROS2D.PanView({
      rootObject: this.viewer.scene,
    });
    //Enable touch events to the viewer.
    createjs.Touch.enable(this.viewer.scene);
    //Add event listeners for map moving.
    this.mouseDown = false;
    this.viewer.scene.addEventListener("stagemousedown", (event) => {
      this.panView.startPan(event.stageX, event.stageY);
      this.mouseDown = true;
    });
    this.viewer.scene.addEventListener("stagemousemove", (event) => {
      if (this.mouseDown === true) {
        this.panView.pan(event.stageX, event.stageY);
      }
    });
    this.viewer.scene.addEventListener("stagemouseup", () => {
      if (this.mouseDown === true) {
        this.mouseDown = false;
      }
    });
    //Add event listeners for map zooming.
    this.viewer.scene.canvas.addEventListener(
      "wheel",
      this.zoomMapHandler.bind(this, 0)
    );
    this.zoomInBtn.addEventListener(
      "click",
      this.zoomMapHandler.bind(this, 50)
    );
    this.zoomOutBtn.addEventListener(
      "click",
      this.zoomMapHandler.bind(this, -50)
    );
    //Add viewer resizing.
    //Remove previous ticker function.
    createjs.Ticker.addEventListener("tick", this.viewer.scene);
    //Add new ticker function.
    createjs.Ticker.on("tick", (event) => {
      //Only update if map is visible.
      if (this.mapElement.style.display == "block") {
        //Get current dimensions.
        const oldWidth = this.viewer.scene.canvas.width;
        const oldHeight = this.viewer.scene.canvas.height;
        //Get new dimensions.
        const newWidth = parseInt(getComputedStyle(this.mapElement).width);
        const newHeight = parseInt(getComputedStyle(this.mapElement).height);
        //Update dimensions.
        this.viewer.scene.canvas.width = newWidth;
        this.viewer.scene.canvas.height = newHeight;
        //Maintain viewer center
        if (this.mapBitmap) {
          this.viewer.shift(
            (oldWidth - newWidth) / 2 / this.viewer.scene.scaleX,
            (newHeight - oldHeight) / 2 / this.viewer.scene.scaleY
          );
        }
        //Update graphics.
        this.viewer.scene.update(event);
      }
    });
  }

  zoomMapHandler(deltaZoom, event) {
    event.preventDefault();
    event.stopPropagation();
    let startZoomX;
    let startZoomY;
    if (event.type === "wheel") {
      const boundingRect = event.target.getBoundingClientRect();
      startZoomX = event.clientX - boundingRect.left;
      startZoomY = event.clientY - boundingRect.top;
      deltaZoom = -event.deltaY;
    } else if (event.type === "click") {
      startZoomX = this.viewer.scene.canvas.width / 2;
      startZoomY = this.viewer.scene.canvas.height / 2;
    } else {
      console.log("zoomMapHandler triggered by unexpected event. Aborting.");
      return;
    }
    this.zoomView.startZoom(startZoomX, startZoomY);
    const zoom = 1 + deltaZoom / this.viewer.scene.canvas.clientHeight;
    this.zoomView.zoom(zoom);
  }

  resetViewer() {
    //Delete al graphics from viewer.
    this.viewer.scene.removeAllChildren();
    //Reserve index for map.
    this.viewer.scene.addChild(new createjs.Shape())
    //Delete map.
    if (this.mapBitmap) {
      this.mapBitmap = null;
    }
    //Reset viewer defaults.
    this.viewer.scene.x = 0;
    this.viewer.scene.x_prev_shift = this.viewer.scene.x;
    this.viewer.scene.y = this.viewer.height;
    this.viewer.scene.y_prev_shift = this.viewer.scene.y;
  }

  async getAndDrawMap(shiftId) {
    try {
      //Get map data from database.
      const mapImage = await this.mapImageInterface.get(shiftId);
      //Record absence of previous map.
      const firstMap = !this.mapBitmap;
      //Store new map origin.
      this.mapOrigin = { x: mapImage.originX, y: mapImage.originY };
      //Create drawable element for map image.
      this.mapBitmap = new createjs.Bitmap(mapImage.imageData);
      //Set drawable origin and scale.
      this.mapBitmap.x = mapImage.originX;
      this.mapBitmap.y =
        -mapImage.height * mapImage.resolution - mapImage.originY;
      this.mapBitmap.scaleX = mapImage.resolution;
      this.mapBitmap.scaleY = mapImage.resolution;
      //Clear scene at map index.
      this.viewer.scene.removeChildAt(0);
      //Add map to viewer.
      this.viewer.scene.addChildAt(this.mapBitmap, 0);
      //Scale and shift viewer if this is the first map.
      if (firstMap) {
        this.viewer.scene.addChildAt(this.mapBitmap, 0);
        //Adjust viewer visible area to display map.
        const viewerRatio = this.viewer.width / this.viewer.height;
        //Fit scale for largest map dimension.
        if (mapImage.width > mapImage.height) {
          this.viewer.scaleToDimensions(
            mapImage.width * mapImage.resolution,
            (mapImage.width * mapImage.resolution) / viewerRatio
          );
        } else {
          this.viewer.scaleToDimensions(
            mapImage.height * mapImage.resolution * viewerRatio,
            mapImage.height * mapImage.resolution
          );
        }
        //Shift view to center map.
        this.viewer.shift(
          mapImage.originX -
            (this.viewer.scene.canvas.width - this.viewer.width) /
              2 /
              this.viewer.scene.scaleX,
          mapImage.originY +
            (this.viewer.scene.canvas.height - this.viewer.height) /
              2 /
              this.viewer.scene.scaleY
        );
      }
      return mapImage;
    } catch (error) {
      //Log response.
      console.log(error.message);
      return null;
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
      this.viewer.scene.removeChild(layerPosesPtr.shape);
      this.mapPointsLayers.splice(layerIndex, 1);
    }
  }

  async getAndDrawMapPoses(segmentId) {
    try {
      //Get local poses from database.
      const localPoseList = await this.localPoseInterface.get(segmentId);
      //Remove any data from the specified segment that was already on the map.
      this.removePoses(segmentId);
      if (localPoseList.length > 0) {
        //Create drawable element for local poses.
        const traceShape = new createjs.Shape();
        //Define color of the stroke.
        let strokeColor;
        if (localPoseList[0].type == "AUTO") {
          strokeColor = "green";
        } else {
          strokeColor = "red";
        }
        //Set drawable style and initial position.
        traceShape.graphics.setStrokeStyle(0.2);
        const traceCommand =
          traceShape.graphics.beginStroke(strokeColor).command;
        traceShape.graphics.moveTo(localPoseList[0].x, -localPoseList[0].y);
        //Add local poses to drawable.
        localPoseList.forEach((localPose) => {
          traceShape.graphics.lineTo(localPose.x, -localPose.y);
        });
        //Add drawable to viewer.
        //If there is an active marker, redraw it on top of the drawable.
        if (this.activeMarker) {
          this.viewer.scene.removeChild(this.activeMarker);
        }
        this.viewer.scene.addChild(traceShape);
        if (this.activeMarker) {
          this.viewer.scene.addChild(this.activeMarker);
        }
        //Save segment id and pointer to map layer in a list.
        this.mapPointsLayers.push({
          segmentId: segmentId,
          mapPosesPtr: {
            shape: traceShape,
            command: traceCommand,
            type: localPoseList[0].type,
          },
        });
      } else {
        console.log("No local position data for segment " + segmentId);
      }
    } catch (error) {
      console.log(error.message);
    }
  }

  removeActiveMarker() {
    //Remove location marker from map.
    if (this.activeMarker) {
      this.viewer.scene.removeChild(this.activeMarker);
      this.activeMarker = null;
    }
  }

  removeActivePoses() {
    //Restore point coloring.
    //Unfortunately, it seems like the only way to do this for easeljs graphics is to re-draw the shape.
    if (this.activePoses) {
      this.activePoses.command.style =
        this.activePoses.type == "AUTO" ? "green" : "red";
      this.activePoses = null;
    }
  }

  addActiveMarker(x, y) {
    //Add location marker to map.
    this.activeMarker = new createjs.Shape();
    this.activeMarker.graphics.beginFill("blue").drawCircle(x, -y, 0.5);
    this.viewer.scene.addChild(this.activeMarker);
  }

  addActivePoses(segmentId) {
    //Change point coloring to blue for poses of the associated segment.
    const selectedPoses = this.mapPointsLayers.find(
      (poses) => poses.segmentId == segmentId
    );
    if (selectedPoses) {
      this.activePoses = selectedPoses.mapPosesPtr;
      this.activePoses.command.style = "blue";
    }
  }
}
