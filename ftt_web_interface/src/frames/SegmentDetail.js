/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { LeafletMap } from "../utility/LeafletMap.js";
import { LocalMap } from "../utility/LocalMap.js";
import { DOMGeneric } from "../utility/DOMGeneric.js";
import { Modal } from "../utility/Modal.js";
import { SegmentInterface } from "../database_interface/Segment.js";
import { ItoReasonInterface } from "../database_interface/ItoReason.js";
import { NoteInterface } from "../database_interface/Note.js";
import { ImageInterface } from "../database_interface/Image.js";
import { SegmentEdit } from "../overlays/SegmentEdit.js";
import { MapConfig } from "../overlays/MapConfig.js";

//Global variables for fixed and common database parameter values.
const SEGMENT_TYPE_ITO = 1;
const ITO_REASON_UNEXPECTED = 12;
const ITO_REASON_PLANNER = 3;
const ITO_REASON_SAFETY = 1;
const ITO_REASON_TRANSIT = 9;
// const ITO_REASON_UNASSIGNED = 11;

export class SegmentDetail {
  //Container class for the segment info and map display sections of the web page.
  constructor(serverInterface, currentUser) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    this.currentUser = currentUser;
    //Reach to DOM elements.
    this.legSelectHook = document.getElementById("leg-id");
    this.unexpectedBtn = document.getElementById("unexpected-btn");
    this.plannerBtn = document.getElementById("planner-btn");
    this.safetyBtn = document.getElementById("safety-btn");
    this.transitBtn = document.getElementById("transit-btn");
    this.autoRefreshBox = document.getElementById("auto-refresh");
    this.segmentTable = document.getElementById("segment-table-body");
    this.segmentNewBtn = document.getElementById("new-segment-btn");
    this.segmentEndBtn = document.getElementById("end-segment-btn");
    this.segmentEditBtn = document.getElementById("edit-segment-btn");
    this.refreshBtn = document.getElementById("refresh-btn");
    this.gpsMapConfig = document.getElementById("gps-map-config-icon");
    this.gpsMapBox = document.getElementById("gps-map-box");
    this.localMapBox = document.getElementById("local-map-box");
    //Initialize variables.
    this.mapInterface = new LeafletMap(serverInterface);
    this.localMapInterface = new LocalMap(serverInterface);
    this.segmentInterface = new SegmentInterface(serverInterface);
    this.itoReasonInterface = new ItoReasonInterface(serverInterface);
    this.noteInterface = new NoteInterface(serverInterface);
    this.imageInterface = new ImageInterface(serverInterface);
    this.segmentList = [];
    this.itoReasonList = null;
    this.autoRefreshIntervalId = null;
    this.selectedRow = null;
    this.selectedRowId = null;
    this.selectedRowParentId = null;
    //Add event listeners.
    this.segmentTable.addEventListener(
      "click",
      this.segmentTableClickHandler.bind(this)
    );
    this.unexpectedBtn.addEventListener(
      "click",
      this.quickReasonEditBtnHandler.bind(this, ITO_REASON_UNEXPECTED)
    );
    this.plannerBtn.addEventListener(
      "click",
      this.quickReasonEditBtnHandler.bind(this, ITO_REASON_PLANNER)
    );
    this.safetyBtn.addEventListener(
      "click",
      this.quickReasonEditBtnHandler.bind(this, ITO_REASON_SAFETY)
    );
    this.transitBtn.addEventListener(
      "click",
      this.quickReasonEditBtnHandler.bind(this, ITO_REASON_TRANSIT)
    );
    this.segmentNewBtn.addEventListener(
      "click",
      this.newSegmentHandler.bind(this)
    );
    this.segmentEndBtn.addEventListener(
      "click",
      this.endSegmentHandler.bind(this)
    );
    this.segmentEditBtn.addEventListener(
      "click",
      this.editSegmentHandler.bind(this)
    );
    this.refreshBtn.addEventListener("click", this.updateSegments.bind(this));
    this.autoRefreshBox.addEventListener(
      "change",
      this.autoRefreshBoxHandler.bind(this)
    );
    this.gpsMapConfig.addEventListener(
      "click",
      this.gpsMapConfigHandler.bind(this)
    );
    this.gpsMapBox.addEventListener(
      "click",
      this.toggleGpsMapHandler.bind(this)
    );
    this.localMapBox.addEventListener(
      "click",
      this.toggleLocalMapHandler.bind(this)
    );
  }

  async updateSegments() {
    //Update segment data table and map display.
    try {
      //Get segment data from DB.
      this.segmentList = await this.segmentInterface.get(
        this.legSelectHook.value
      );
      //Remove old data from table.
      const tableRows = Array.from(this.segmentTable.querySelectorAll("tr"));
      for (const row of tableRows.reverse()) {
        const match = this.segmentList.find(
          (entry) => entry.id == row.firstChild.textContent
        );
        if (!match) {
          //If the row was selected, delete selection data.
          if (this.selectedRowId == row.firstChild.textContent) {
            this.selectedRow = null;
            this.selectedRowId = null;
            this.selectedRowParentId = null;
            this.mapInterface.removeActiveMarker();
            this.mapInterface.removeActivePoses();
            this.localMapInterface.removeActiveMarker();
            this.localMapInterface.removeActivePoses();
          }
          //Delete row.
          row.remove();
        }
      }
      const mapLayers = this.mapInterface.mapPointsLayers;
      const localMapLayers = this.localMapInterface.mapPointsLayers;
      //If GPS map is active:
      if (this.gpsMapBox.checked) {
        //Remove old data from map.
        for (let idx = mapLayers.length - 1; idx >= 0; idx--) {
          const match = this.segmentList.find(
            (entry) => entry.id == mapLayers[idx].segmentId
          );
          if (!match) {
            this.mapInterface.leafletMap.removeLayer(
              mapLayers[idx].mapPosesPtr
            );
            mapLayers.splice(idx, 1);
          }
        }
        //Reset display if map is empty.
        if (mapLayers.length == 0) {
          const firstLocEntry = this.segmentList.find(
            (entry) => entry.lat && entry.lng
          );
          if (firstLocEntry) {
            console.log(
              "Setting map display in " +
                firstLocEntry.lat +
                ", " +
                firstLocEntry.lng
            );
            this.mapInterface.leafletMap.setView(
              [firstLocEntry.lat, firstLocEntry.lng],
              17
            );
          }
        }
      }
      //If local map is active:
      if (this.localMapBox.checked) {
        //Remove old data from map.
        for (let idx = localMapLayers.length - 1; idx >= 0; idx--) {
          const match = this.segmentList.find(
            (entry) => entry.id == localMapLayers[idx].segmentId
          );
          if (!match) {
            this.localMapInterface.removePoses(localMapLayers[idx].segmentId);
          }
        }
        //Reset display if map is empty.
        if (localMapLayers.length == 0) {
          this.localMapInterface.resetViewer();
        }
        // Update local map:
        const shiftSelectValue = document.getElementById("shift-id").value;
        if (shiftSelectValue) {
          await this.localMapInterface.getAndDrawMap(shiftSelectValue);
        } else {
          alert("No shift selected!");
        }
      }
      //Add or edit data from table and map:
      for (const entry of this.segmentList.reverse()) {
        //Update table rows:
        if (
          entry.segmentType == "AUTO" ||
          (entry.segmentType == "ITO" && entry.parentId)
        ) {
          const matchedRow = tableRows.find(
            (row) => row.firstChild.textContent == entry.id
          );
          const entryState = entry.endTime == null ? "OPEN" : "CLOSED";
          const entryReason = entry.itoReason || "";
          if (matchedRow) {
            //Update row info with new data.
            if (matchedRow.children[2].textContent != entryReason) {
              console.log("Updating row " + entry.id + " reason");
              matchedRow.children[2].textContent = entryReason;
            }
            if (matchedRow.children[3].textContent != entryState) {
              console.log("Updating row " + entry.id + " state");
              matchedRow.children[3].textContent = entryState;
            }
          } else {
            //Create new table row.
            console.log("Creating row " + entry.id);
            DOMGeneric.addRow(this.segmentTable, [
              entry.id,
              entry.segmentType,
              entryReason,
              entryState,
            ]);
          }
        }
        //If GPS map is active:
        if (this.gpsMapBox.checked) {
          //Update map poses:
          if (!entry.parentId) {
            const matchedMapLayer = mapLayers.find(
              (layer) => layer.segmentId == entry.id
            );
            if (!matchedMapLayer || entry.endTime == null) {
              //Create new or update points layer.
              await this.mapInterface.getAndDrawMapPoses(entry.id);
              if (
                (entry.segmentType == "AUTO" &&
                  entry.id == this.selectedRowId) ||
                (entry.segmentType == "ITO" &&
                  entry.id == this.selectedRowParentId)
              ) {
                this.mapInterface.addActivePoses(entry.id);
              }
            }
          }
        }
        //If local map is active:
        if (this.localMapBox.checked) {
          //Update map poses:
          if (!entry.parentId) {
            const matchedMapLayer = localMapLayers.find(
              (layer) => layer.segmentId == entry.id
            );
            if (!matchedMapLayer || entry.endTime == null) {
              //Create new or update points layer.
              await this.localMapInterface.getAndDrawMapPoses(entry.id);
              if (
                (entry.segmentType == "AUTO" &&
                  entry.id == this.selectedRowId) ||
                (entry.segmentType == "ITO" &&
                  entry.id == this.selectedRowParentId)
              ) {
                this.localMapInterface.addActivePoses(entry.id);
              }
            }
          }
        }
      }
    } catch (error) {
      alert(error.message);
    }
  }

  autoRefreshBoxHandler() {
    //Call updateSegments in regular intervals if the box is checked.
    if (this.autoRefreshBox.checked) {
      if (this.legSelectHook.value) {
        this.autoRefreshIntervalId = setInterval(
          this.updateSegments.bind(this),
          2000
        );
      } else {
        this.autoRefreshBox.checked = false;
        alert("No leg selected!");
      }
    } else {
      clearInterval(this.autoRefreshIntervalId);
    }
  }

  segmentTableClickHandler(event) {
    //Find the closest row to the click event.
    const closestRow = event.target.closest("tr");
    if (this.selectedRow) {
      //Remove any previous selection data
      this.selectedRow.classList.remove("active-row");
      this.mapInterface.removeActiveMarker();
      this.mapInterface.removeActivePoses();
      this.localMapInterface.removeActiveMarker();
      this.localMapInterface.removeActivePoses();
      if (this.selectedRow == closestRow) {
        //Reset selection variables for a selected row being deselected.
        this.selectedRow = null;
        this.selectedRowId = null;
        this.selectedRowParentId = null;
        return;
      }
    }
    //Add selection data to reference the clicked row.
    this.selectedRow = closestRow;
    this.selectedRowId = closestRow.firstChild.textContent;
    this.selectedRow.classList.add("active-row");
    //Find associated segment from the list.
    const selectedEntry = this.segmentList.find(
      (entry) => entry.id == this.selectedRowId
    );
    //Mark start location of the selected segment/parent segment.
    this.selectedRowParentId = selectedEntry.parentId;
    if (selectedEntry.lat && selectedEntry.lng) {
      this.mapInterface.addActiveMarker(selectedEntry.lat, selectedEntry.lng);
    } else if (this.selectedRowParentId) {
      const selectedEntryParent = this.segmentList.find(
        (entry) => entry.id == this.selectedRowParentId
      );
      if (selectedEntryParent.lat && selectedEntryParent.lng) {
        this.mapInterface.addActiveMarker(
          selectedEntryParent.lat,
          selectedEntryParent.lng
        );
      }
    }
    //Mark local start location of the selected segment/parent segment.
    if (selectedEntry.local_x && selectedEntry.local_y) {
      this.localMapInterface.addActiveMarker(
        selectedEntry.local_x,
        selectedEntry.local_y
      );
    } else if (this.selectedRowParentId) {
      const selectedEntryParent = this.segmentList.find(
        (entry) => entry.id == this.selectedRowParentId
      );
      if (selectedEntryParent.local_x && selectedEntryParent.local_y) {
        this.localMapInterface.addActiveMarker(
          selectedEntryParent.local_x,
          selectedEntryParent.local_y
        );
      }
    }
    //Highlight map poses of the selected parent segment.
    this.mapInterface.addActivePoses(
      this.selectedRowParentId || this.selectedRowId
    );
    //Highlight local map poses of the selected parent segment.
    this.localMapInterface.addActivePoses(
      this.selectedRowParentId || this.selectedRowId
    );
  }

  async quickReasonEditBtnHandler(itoReasonId) {
    //Check that a segment is selected.
    if (!this.selectedRowId) {
      alert("No segment selected for edition!");
      return;
    }
    const selectedEntry = this.segmentList.find(
      (entry) => entry.id == this.selectedRowId
    );
    if (selectedEntry.segmentType == "AUTO") {
      alert("Cannot assign ITO reason to AUTO segment!");
      return;
    }
    try {
      //Send put request to edit the ito reason.
      await this.segmentInterface.put(this.selectedRowId, "edit", itoReasonId);
      //Update table.
      this.updateSegments();
    } catch (error) {
      alert(error.message);
    }
  }

  async newSegmentHandler() {
    //Fetch ito reason data from server if needed.
    try {
      if (!this.itoReasonList || this.itoReasonList.length == 0) {
        this.itoReasonList = await this.itoReasonInterface.get();
        //If there are no ito reasons, alert the user.
        if (this.itoReasonList.length == 0) {
          alert(
            "ITO reasons list empty. Your database was not properly initialized. Please fix this before continuing."
          );
          return;
        }
      }
    } catch (error) {
      alert(error.message);
      return;
    }
    //Select the unassigned ito reason.
    const matchedItoReason = this.itoReasonList.find(
      (entry) => entry.shortDescription == "Unassigned"
    );
    //Check there is a match and keep the id of the unassigned ito reason.
    let unassignedItoReasonId;
    if (matchedItoReason) {
      unassignedItoReasonId = matchedItoReason.id;
    } else {
      alert(
        "Unable to find the 'Unassigned' ITO reason. Your database was not properly initialized. Please fix this before continuing."
      );
      return;
    }
    try {
      //Send post request to create a new ITO segment with unassigned reason.
      await this.segmentInterface.post(
        this.legSelectHook.value,
        unassignedItoReasonId,
        SEGMENT_TYPE_ITO,
        null,
        null,
        null,
        null
      );
      //Update table.
      this.updateSegments();
    } catch (error) {
      alert(error.message);
    }
  }

  async endSegmentHandler() {
    //Check that a segment is selected.
    if (!this.selectedRowId) {
      alert("No segment selected to end!");
      return;
    }
    try {
      //Send put request to close the segment.
      await this.segmentInterface.put(this.selectedRowId, "close");
      //Update table.
      this.updateSegments();
    } catch (error) {
      alert(error.message);
    }
  }

  async editSegmentHandler() {
    //Check that a segment is selected.
    if (!this.selectedRowId) {
      alert("Please select a segment to edit.");
      return;
    }
    try {
      //Fetch ito reason data from server if needed.
      if (!this.itoReasonList || this.itoReasonList.length == 0) {
        this.itoReasonList = await this.itoReasonInterface.get();
      }
      //Find the selected segment entry.
      const selectedSegment = this.segmentList.find(
        (segment) => segment.id == this.selectedRowId
      );
      //Build the segment editing overlay.
      const segmentEdit = new SegmentEdit(
        selectedSegment,
        this.segmentInterface,
        this.noteInterface,
        this.imageInterface,
        this.itoReasonList,
        this.currentUser
      );
      //Display the overlay.
      const segmentModal = new Modal(
        segmentEdit,
        "Your browser does't support this feature! - Please change to a more modern one.",
        this.updateSegments.bind(this)
      );
      segmentModal.show();
    } catch (error) {
      alert(error.messaje);
    }
  }

  gpsMapConfigHandler() {
    //Build a map configuration overlay.
    const mapConfig = new MapConfig();
    //Display the overlay.
    const userModal = new Modal(
      mapConfig,
      "Your browser doesn't support this feature! - Please change to a more modern one.",
      () => {
        this.mapInterface.changeTileLayer();
      }
    );
    userModal.show();
  }

  toggleGpsMapHandler(event) {
    if (event.target.checked) {
      this.mapInterface.mapElement.style.display = "block";
      this.mapInterface.leafletMap.invalidateSize(true);
      this.updateSegments();
    } else {
      //Hide map container.
      this.mapInterface.mapElement.style.display = "none";
      //Delete map poses.
      const mapLayers = this.mapInterface.mapPointsLayers;
      for (let idx = mapLayers.length - 1; idx >= 0; idx--) {
        this.mapInterface.leafletMap.removeLayer(mapLayers[idx].mapPosesPtr);
        mapLayers.splice(idx, 1);
      }
    }
  }

  toggleLocalMapHandler(event) {
    if (event.target.checked) {
      this.localMapInterface.mapElement.style.display = "block";
      this.updateSegments();
    } else {
      //Hide local map container.
      this.localMapInterface.mapElement.style.display = "none";
      //Delete local map poses.
      const localMapLayers = this.localMapInterface.mapPointsLayers;
      for (let idx = localMapLayers.length - 1; idx >= 0; idx--) {
        this.localMapInterface.removePoses(localMapLayers[idx].segmentId);
      }
    }
  }
}
