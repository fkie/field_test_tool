/* eslint-disable no-undef */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { SegmentDetail } from "./SegmentDetail.js";
import { LogSelection } from "./LogSelection.js";
import {
  Personnel,
  PersonnelInterface,
} from "../database_interface/Personnel.js";
import { UserSelect } from "../overlays/UserSelect.js";
import { TestEventInterface } from "../database_interface/TestEvent.js";
import { ReportDownload } from "../overlays/ReportDownload.js";
import { ShiftCompare } from "../overlays/ShiftCompare.js";
import { Modal } from "../utility/Modal.js";
import { RosConfig } from "../overlays/RosConfig.js";

export class MainFrame {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    //Reach to DOM elements.
    this.userIcon = document.getElementById("user-icon");
    this.userName = document.getElementById("user-name");
    this.reportIcon = document.getElementById("report-icon");
    this.downloadIcon = document.getElementById("download-icon");
    this.rosConnectIcon = document.getElementById("ros-status-light");
    this.selectForCompreBox = document.getElementById("select-for-compare");
    this.compareBtn = document.getElementById("compare-btn");
    //Initialize variables.
    this.currentUser = new Personnel();
    this.personnelInterface = new PersonnelInterface(this.serverInterface);
    this.testEventInterface = new TestEventInterface(this.serverInterface);
    this.openLogsSelected = false;
    this.ros = new ROSLIB.Ros();
    this.rosConnectIntervalId = null;
    this.forceRosConnect = false;
    this.ros_server_adr = `ws://${location.hostname}:9090`;
    this.compareIds = [];
    //Check stored configuration.
    let rosData = JSON.parse(localStorage.getItem("fttRosData"));
    if (rosData) {
      //Update variable.
      this.ros_server_adr = rosData.url;
    }
    //Contruct page sections.
    this.segmentDetail = new SegmentDetail(serverInterface, this.currentUser);
    this.logSelection = new LogSelection(
      serverInterface,
      this.ros,
      this.currentUser,
      this.logSelectionDoneCallback.bind(this)
    );
    //Add event listeners.
    this.userIcon.addEventListener(
      "click",
      this.userIconClickHandler.bind(this)
    );
    this.reportIcon.addEventListener(
      "click",
      this.reportIconClickHandler.bind(this)
    );
    this.downloadIcon.addEventListener(
      "click",
      this.downloadIconClickHandler.bind(this)
    );
    this.rosConnectIcon.addEventListener(
      "click",
      this.rosConnectIconClickHandler.bind(this)
    );
    this.selectForCompreBox.addEventListener(
      "change",
      this.selectForCompreBoxHandler.bind(this)
    );
    this.compareBtn.addEventListener("click", () => {
      this.compareLogsHandler();
    });
    this.ros.on("error", this.rosErrorHandler.bind(this));
    this.ros.on("connection", this.rosConnectionHandler.bind(this));
    this.ros.on("close", this.rosCloseHandler.bind(this));
    //Try to connect to ROS.
    this.rosConnect();
  }

  async logSelectionDoneCallback() {
    //Display segment and map sections.
    document.getElementById("segment-detail").style.display = "block";
    document.getElementById("map-viewer").style.display = "block";
    //Get new segment data from DB.
    const segmentList = await this.segmentDetail.segmentInterface.get(
      this.segmentDetail.legSelectHook.value
    );
    //If any segment has GPS poses, activate the GPS map
    for (const entry of segmentList) {
      if (entry.lng && entry.lat && !this.segmentDetail.gpsMapBox.checked) {
        this.segmentDetail.gpsMapBox.checked = true;
        this.segmentDetail.mapInterface.mapElement.style.display = "block";
        this.segmentDetail.mapInterface.leafletMap.invalidateSize(true);
        break;
      }
    }
    //If any segment has local poses, activate the local map
    for (const entry of segmentList) {
      if (
        entry.local_x &&
        entry.local_y &&
        !this.segmentDetail.localMapBox.checked
      ) {
        this.segmentDetail.localMapBox.checked = true;
        this.segmentDetail.localMapInterface.mapElement.style.display = "block";
        break;
      }
    }
    //Update segments table.
    this.segmentDetail.updateSegments(segmentList);
    //Check if this shift id is selected for compare and update slider accodingly.
    const shiftSelectValue = document.getElementById("shift-id").value;
    if (this.compareIds.includes(shiftSelectValue)) {
      this.selectForCompreBox.checked = true;
    } else {
      this.selectForCompreBox.checked = false;
    }
  }

  async userIconClickHandler() {
    try {
      //Get the latest personnel data from the server.
      const personnelList = await this.personnelInterface.get();
      //Build a user selection overlay.
      const userSelect = new UserSelect(this.currentUser, personnelList);
      //Display the overlay.
      const userModal = new Modal(
        userSelect,
        "Your browser doesn't support this feature! - Please change to a more modern one.",
        () => {
          this.userName.textContent = this.currentUser.name || "Select User";
        }
      );
      userModal.show();
    } catch (error) {
      alert(error.message);
    }
  }

  async reportIconClickHandler() {
    try {
      //Get the latest test event data from the server.
      const testEventList = await this.testEventInterface.get();
      //Build a report generator overlay.
      const reportDownload = new ReportDownload(
        this.serverInterface,
        testEventList
      );
      //Display the overlay.
      const userModal = new Modal(
        reportDownload,
        "Your browser doesn't support this feature! - Please change to a more modern one."
      );
      userModal.show();
    } catch (error) {
      alert(error.message);
    }
  }

  async downloadIconClickHandler() {
    try {
      // Get blob for the database dump file
      const blob = await this.serverInterface.sendGetRequest("download_data");
      // Create a URL link for the file and download it
      const url = window.URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = 'ftt_dump.sql';
      document.body.appendChild(link);
      link.click();
      // Remove the URL link
      document.body.removeChild(link);
      window.URL.revokeObjectURL(url);
    } catch (error) {
      alert(error.message);
    }
  }

  rosConnectIconClickHandler() {
    //Build a map configuration overlay.
    const rosConfig = new RosConfig();
    //Display the overlay.
    const userModal = new Modal(
      rosConfig,
      "Your browser doesn't support this feature! - Please change to a more modern one.",
      () => {
        this.forceRosConnect = true;
        this.rosConnect();
      }
    );
    userModal.show();
  }

  selectForCompreBoxHandler() {
    //Store or delete this shift id for compare
    const shiftSelectValue = document.getElementById("shift-id").value;
    if (this.selectForCompreBox.checked) {
      this.compareIds.push(shiftSelectValue);
    } else {
      this.compareIds = this.compareIds.filter(
        (value) => value != shiftSelectValue
      );
    }
    this.compareBtn.disabled = this.compareIds.length < 2;
  }

  compareLogsHandler() {
    try {
      //Build a shift compare overlay.
      const shiftCompare = new ShiftCompare(
        this.serverInterface,
        this.compareIds
      );
      //Display the overlay.
      const userModal = new Modal(
        shiftCompare,
        "Your browser doesn't support this feature! - Please change to a more modern one.",
        () => {},
        true
      );
      userModal.show();
    } catch (error) {
      alert(error.message);
    }
  }

  rosConnect() {
    this.ros.connect(this.ros_server_adr);
  }

  rosErrorHandler() {
    this.logSelection.updateLogging();
    const errorMsg =
      "Error connecting to ROS. Make sure rosbridge_server is running.";
    console.log(errorMsg);
    if (this.forceRosConnect) {
      alert(errorMsg);
      this.forceRosConnect = false;
    }
  }

  rosConnectionHandler() {
    this.logSelection.updateLogging();
    this.rosConnectIcon.classList.remove("disconnected");
    this.rosConnectIcon.classList.add("connected");
    this.rosConnectIcon.textContent = "sensors";
    console.log("Connection to ROS established.");
    // if(this.rosConnectIntervalId) {
    //   clearInterval(this.rosConnectIntervalId);
    //   this.rosConnectIntervalId = null;
    // }
  }

  rosCloseHandler() {
    this.logSelection.updateLogging();
    this.rosConnectIcon.classList.remove("connected");
    this.rosConnectIcon.classList.add("disconnected");
    this.rosConnectIcon.textContent = "sensors_off";
    console.log("Connection to ROS closed.");
    // if(!this.rosConnectIntervalId) {
    //   this.rosConnectIntervalId = setInterval(
    //     this.rosConnect.bind(this),
    //     2000
    //   );
    // }
  }
}
