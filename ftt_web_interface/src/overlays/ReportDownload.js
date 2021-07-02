/* eslint-disable no-control-regex */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";

//Structure and logic of the report download UI.
export class ReportDownload {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor(serverInterface, testEventList) {
    //Set arguments as properties.
    this.serverInterface = serverInterface;
    //Import and build leg edition node.
    const templateEl = document.getElementById("report-download-template");
    this.element = document.importNode(templateEl.content, true);

    //============================ DATA SECTION ============================
    //Reach to report test event id select.
    const testEventSelect = this.element.getElementById("report-test-event-id");
    //Populate select.
    DOMGeneric.populateSelect(
      testEventSelect,
      testEventList.filter((entry) => !!entry.endTime).map((entry) => entry.id).reverse()
    );
    //Register generate report button handler.
    this.element
      .querySelector("form button")
      .addEventListener("click", this.generateReportBtnHandler.bind(this));
  }

  async generateReportBtnHandler(event) {
    event.preventDefault();
    //Get data to update.
    const tileServer = document.getElementById("report-tile-server").value;
    const zoomLevel = document.getElementById("report-zoom-level").value.replace(/[^0-9]/g, "");
    const name = document.getElementById("report-name").value.replace(/[^\x00-\x7F]/g, "");
    const version = document.getElementById("report-version").value.replace(/[^\x00-\x7F]/g, "");
    const testEventId = document.getElementById("report-test-event-id").value;
    const minDuration = document.getElementById("report-min-duration").value.replace(/[^0-9.]/g, "");
    const useLocal = document.getElementById("report-use-local-poses").value;
    const recipientName = document.getElementById("report-recipient-name").value.replace(/[^\x00-\x7F]/g, "");
    const recipientAdr1 = document.getElementById("report-recipient-address-1").value.replace(/[^\x00-\x7F]/g, "");
    const recipientAdr2 = document.getElementById("report-recipient-address-2").value.replace(/[^\x00-\x7F]/g, "");
    const creatorName = document.getElementById("report-creator-name").value.replace(/[^\x00-\x7F]/g, "");
    const creatorAdr1 = document.getElementById("report-creator-address-1").value.replace(/[^\x00-\x7F]/g, "");
    const creatorAdr2 = document.getElementById("report-creator-address-2").value.replace(/[^\x00-\x7F]/g, "");
    // Send post request.
    try {
      const data = {
        "tile_server": tileServer, 
        "zoom_level": zoomLevel, 
        "report_name": name, 
        "report_version": version, 
        "test_event_id": testEventId, 
        "min_duration": minDuration, 
        "use_local_poses": useLocal, 
        "recipient_name": recipientName, 
        "recipient_address_l1": recipientAdr1, 
        "recipient_address_l2": recipientAdr2, 
        "creator_name": creatorName, 
        "creator_address_l1": creatorAdr1, 
        "creator_address_l2": creatorAdr2
      };
      await this.serverInterface.sendPostRequest("generate_report", data);
      this.element.querySelector("form button:last-of-type").disabled = false;
    } catch (error) {
      alert(error.message);
    }
  }
}
