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
import { Modal } from "../utility/Modal.js";

export class MainFrame {
  constructor(serverInterface) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    //Reach to DOM elements.
    this.userIcon = document.getElementById("user-icon");
    this.userName = document.getElementById("user-name");
    this.downloadIcon = document.getElementById("download-icon");
    this.rosConnectIcon = document.getElementById("ros-status-light");
    //Initialize variables.
    this.currentUser = new Personnel();
    this.personnelInterface = new PersonnelInterface(this.serverInterface);
    this.testEventInterface = new TestEventInterface(this.serverInterface);
    this.openLogsSelected = false;
    this.ros = new ROSLIB.Ros();
    this.rosConnectIntervalId = null;
    this.forceRosConnect = false;
    this.ros_server_adr = `ws://${location.hostname}:9090`;
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
    this.downloadIcon.addEventListener(
      "click",
      this.downloadIconClickHandler.bind(this)
    );
    this.rosConnectIcon.addEventListener(
      "click",
      this.rosConnectIconClickHandler.bind(this)
    );
    this.ros.on("error", this.rosErrorHandler.bind(this));
    this.ros.on("connection", this.rosConnectionHandler.bind(this));
    this.ros.on("close", this.rosCloseHandler.bind(this));
    //Try to connect to ROS.
    this.rosConnect();
  }

  logSelectionDoneCallback() {
    //Display segment and map sections.
    document.getElementById("segment-detail").style.display = "block";
    document.getElementById("map-viewer").style.display = "block";
    //Update segments table.
    this.segmentDetail.updateSegments();
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

  async downloadIconClickHandler() {
    try {
      //Get the latest test event data from the server.
      const testEventList = await this.testEventInterface.get();
      //Build a user selection overlay.
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

  rosConnectIconClickHandler() {
    this.forceRosConnect = true;
    this.rosConnect();
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
