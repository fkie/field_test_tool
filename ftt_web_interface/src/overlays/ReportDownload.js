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
    //Get html hooks.
    const testEventSelect = this.element.getElementById("report-test-event-id");
    const form = this.element.querySelector("form");
    //Populate select.
    DOMGeneric.populateSelect(
      testEventSelect,
      testEventList
        .filter((entry) => !!entry.endTime)
        .map(
          (entry) =>
            entry.id +
            ": " +
            new Date(entry.startTime * 1000).toISOString().split("T")[0] +
            "-" +
            (entry.location || "(no location)")
        )
        .reverse()
    );
    //Register generate report button handler.
    form
      .querySelector("button")
      .addEventListener("click", this.generateReportBtnHandler.bind(this));
    //Register change handler for every select.
    form.querySelectorAll("select").forEach((select) => {
      select.addEventListener("change", this.formChangedHandler.bind(this));
    });
    //Register input handler for the complete form.
    form.addEventListener("input", this.formChangedHandler.bind(this));
    //Register input handler for textareas to highlight when empty.
    form.querySelectorAll("textarea").forEach((textarea) => {
      textarea.addEventListener("input", this.checkTextareas.bind(this));
    });
    //Run the textarea check
    this.checkTextareas();
  }

  checkTextareas() {
    this.element.querySelectorAll("form textarea").forEach((textarea) => {
      if (textarea.value.match(/[^\x00-\x7F]/g)) {
        textarea.classList.add("error");
      } else {
        textarea.classList.remove("error");
      }
    });
  }

  formChangedHandler() {
    // Get the hook to the generate button
    const generateBtn = this.element.querySelector("form button:first-of-type");
    // Check if all fields have values and update generate button status
    if (
      document.getElementById("report-tile-server").value &&
      document
        .getElementById("report-zoom-level")
        .value.replace(/[^0-9]/g, "") &&
      document
        .getElementById("report-name")
        .value.replace(/[^\x00-\x7F]/g, "") &&
      document
        .getElementById("report-version")
        .value.replace(/[^\x00-\x7F]/g, "") &&
      document.getElementById("report-test-event-id").value &&
      document
        .getElementById("report-min-duration")
        .value.replace(/[^0-9.]/g, "") &&
      document.getElementById("report-use-local-poses").value &&
      document
        .getElementById("report-recipient-name")
        .value.replace(/[^\x00-\x7F]/g, "") &&
      document
        .getElementById("report-recipient-address")
        .value.replace(/[^\x00-\x7F]/g, "") &&
      document
        .getElementById("report-creator-name")
        .value.replace(/[^\x00-\x7F]/g, "") &&
      document
        .getElementById("report-creator-address")
        .value.replace(/[^\x00-\x7F]/g, "")
    ) {
      generateBtn.disabled = false;
    } else {
      generateBtn.disabled = true;
    }
  }

  async generateReportBtnHandler(event) {
    event.preventDefault();
    // Get data to update.
    const tileServer = document.getElementById("report-tile-server").value;
    const zoomLevel = document
      .getElementById("report-zoom-level")
      .value.replace(/[^0-9]/g, "");
    const name = document
      .getElementById("report-name")
      .value.replace(/[^\x00-\x7F]/g, "");
    const version = document
      .getElementById("report-version")
      .value.replace(/[^\x00-\x7F]/g, "");
    const testEventId = document
      .getElementById("report-test-event-id")
      .value.split(":")[0];
    const minDuration = document
      .getElementById("report-min-duration")
      .value.replace(/[^0-9.]/g, "");
    const useLocal = document.getElementById("report-use-local-poses").value;
    const recipientName = document
      .getElementById("report-recipient-name")
      .value.replace(/[^\x00-\x7F]/g, "");
    const recipientAddress = document
      .getElementById("report-recipient-address")
      .value.replace(/[^\x00-\x7F]/g, "");
    const creatorName = document
      .getElementById("report-creator-name")
      .value.replace(/[^\x00-\x7F]/g, "");
    const creatorAddress = document
      .getElementById("report-creator-address")
      .value.replace(/[^\x00-\x7F]/g, "");
    // Send post request.
    try {
      const data = {
        tile_server: tileServer,
        zoom_level: zoomLevel,
        report_name: name,
        report_version: version,
        test_event_id: testEventId,
        min_duration: minDuration,
        use_local_poses: useLocal,
        recipient_name: recipientName,
        recipient_address: recipientAddress,
        creator_name: creatorName,
        creator_address: creatorAddress,
      };
      await this.serverInterface.sendPostRequest("generate_report", data);
      this.element.querySelector("form button:last-of-type").disabled = false;
    } catch (error) {
      alert(error.message);
    }
  }
}
