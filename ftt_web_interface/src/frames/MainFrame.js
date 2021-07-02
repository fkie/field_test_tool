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
    //Initialize variables.
    this.currentUser = new Personnel();
    this.personnelInterface = new PersonnelInterface(this.serverInterface);
    this.testEventInterface = new TestEventInterface(this.serverInterface);
    //Contruct page sections.
    this.segmentDetail = new SegmentDetail(serverInterface, this.currentUser);
    this.trailSelection = new LogSelection(serverInterface, this.currentUser);
    //Add event listeners.
    this.userIcon.addEventListener(
      "click",
      this.userIconClickHandler.bind(this)
    );
    this.downloadIcon.addEventListener(
      "click",
      this.downloadIconClickHandler.bind(this)
    );
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
        "Your browser does't support this feature! - Please change to a more modern one.",
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
      const reportDownload = new ReportDownload(this.serverInterface, testEventList);
      //Display the overlay.
      const userModal = new Modal(
        reportDownload,
        "Your browser does't support this feature! - Please change to a more modern one."
      );
      userModal.show();
    } catch (error) {
      alert(error.message);
    }
  }
}
