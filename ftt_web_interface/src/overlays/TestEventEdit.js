/* eslint-disable no-control-regex */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";
import { timeZoneUTC } from "../utility/TimeZone.js";

//Structure and logic of the test event edition UI.
export class TestEventEdit {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor(testEvent, testEventInterface) {
    //Set arguments as properties.
    this.testEvent = testEvent;
    this.testEventInterface = testEventInterface;
    //Import and build test event edition node.
    const templateEl = document.getElementById("edit-test-event-template");
    this.element = document.importNode(templateEl.content, true);

    //=============================== HEADER ===============================
    //Set title, start and end time.
    this.element.querySelector("header h2 span:last-of-type").textContent =
      testEvent.id;
    this.element.querySelector(
      "header p:first-of-type span:last-of-type"
    ).textContent = new Date(testEvent.startTime * 1000).toISOString();
    if (testEvent.endTime) {
      this.element.querySelector(
        "header p:last-of-type span:last-of-type"
      ).textContent = new Date(testEvent.endTime * 1000).toISOString();
    }

    //============================ DATA SECTION ============================
    //Reach to test event info data elements.
    const locationInput = this.element.getElementById("location");
    const versionInput = this.element.getElementById("version");
    const timeZoneSelect = this.element.getElementById("time-zone");
    const noteText = this.element.getElementById("test-event-note");
    //Populate time zone select.
    const timeZoneList = timeZoneUTC.map((timeZone) => timeZone.label);
    DOMGeneric.populateSelect(timeZoneSelect, timeZoneList);
    //Focus matching option in the select with the time zone in the test event object.
    const matchedTimeZoneIdx = timeZoneList.findIndex(
      (entry) => entry == testEvent.timeZone
    );
    if (matchedTimeZoneIdx > -1) {
      timeZoneSelect.value = timeZoneList[matchedTimeZoneIdx];
      DOMGeneric.removeFirstEmptyOption(timeZoneSelect);
    }
    //Fill input fields with test event data.
    locationInput.value = testEvent.location;
    versionInput.value = testEvent.version;
    noteText.value = testEvent.note;
    //Set change event listeners.
    locationInput.addEventListener(
      "input",
      this.testEventDataChangedHandler.bind(this)
    );
    versionInput.addEventListener(
      "input",
      this.testEventDataChangedHandler.bind(this)
    );
    timeZoneSelect.addEventListener(
      "change",
      this.testEventDataChangedHandler.bind(this)
    );
    noteText.addEventListener(
      "input",
      this.testEventDataChangedHandler.bind(this)
    );
    //Register update button handler.
    this.element
      .querySelector("form button")
      .addEventListener("click", this.updateDataBtnHandler.bind(this));
  }

  testEventDataChangedHandler() {
    //Activate update button.
    this.element.querySelector("form button").disabled = false;
  }

  async updateDataBtnHandler(event) {
    event.preventDefault();
    //Get data to update.
    const location = document.getElementById("location").value.replace(/[^\x00-\x7F]/g, "");
    const version = document.getElementById("version").value.replace(/[^\x00-\x7F]/g, "");
    const timeZone = document.getElementById("time-zone").value;
    const note = document.getElementById("test-event-note").value.replace(/[^\x00-\x7F]/g, "");
    //Send put request.
    try {
      await this.testEventInterface.put(
        this.testEvent.id,
        "edit",
        location,
        version,
        timeZone,
        note
      );
      this.element.querySelector("form button").disabled = true;
    } catch (error) {
      alert(error.message);
    }
  }
}
