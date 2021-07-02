/* eslint-disable no-control-regex */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";

//Structure and logic of the leg edition UI.
export class LegEdit {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor(leg, legInterface, weatherList, poseSourceList) {
    //Set arguments as properties.
    this.leg = leg;
    this.legInterface = legInterface;
    this.weatherList = weatherList;
    this.poseSourceList = poseSourceList;
    //Import and build leg edition node.
    const templateEl = document.getElementById("edit-leg-template");
    this.element = document.importNode(templateEl.content, true);

    //=============================== HEADER ===============================
    //Set title, start and end time.
    this.element.querySelector("header h2 span:last-of-type").textContent =
      leg.id;
    this.element.querySelector(
      "header p:first-of-type span:last-of-type"
    ).textContent = new Date(leg.startTime * 1000).toISOString();
    if (leg.endTime) {
      this.element.querySelector(
        "header p:last-of-type span:last-of-type"
      ).textContent = new Date(leg.endTime * 1000).toISOString();
    }

    //============================ DATA SECTION ============================
    //Reach to leg info data elements.
    const weatherSelect = this.element.getElementById("weather");
    const defaultPoseSourceSelect = this.element.getElementById(
      "default-pose-source"
    );
    const noteText = this.element.getElementById("leg-note");
    //Populate selects.
    DOMGeneric.populateSelect(
      weatherSelect,
      weatherList.map((entry) => entry.shortDescription)
    );
    DOMGeneric.populateSelect(
      defaultPoseSourceSelect,
      poseSourceList.map((entry) => entry.shortDescription)
    );
    //Focus matching option on the selects with the properties in the leg object.
    const matchedWeather = weatherList.find(
      (entry) => entry.id == leg.weatherId
    );
    if (matchedWeather) {
      weatherSelect.value = matchedWeather.shortDescription;
      DOMGeneric.removeFirstEmptyOption(weatherSelect);
    }
    const matchedPoseSource = poseSourceList.find(
      (entry) => entry.id == leg.defaultPoseSourceId
    );
    if (matchedPoseSource) {
      defaultPoseSourceSelect.value = matchedPoseSource.shortDescription;
      DOMGeneric.removeFirstEmptyOption(defaultPoseSourceSelect);
    }
    //Fill input fields with leg data.
    noteText.value = leg.note;
    //Set change event listeners.
    weatherSelect.addEventListener(
      "change",
      this.legDataChangedHandler.bind(this)
    );
    defaultPoseSourceSelect.addEventListener(
      "change",
      this.legDataChangedHandler.bind(this)
    );
    noteText.addEventListener("change", this.legDataChangedHandler.bind(this));
    //Register update button handler.
    this.element
      .querySelector("form button")
      .addEventListener("click", this.updateDataBtnHandler.bind(this));
  }

  legDataChangedHandler() {
    //Activate update button.
    this.element.querySelector("form button").disabled = false;
  }

  async updateDataBtnHandler(event) {
    event.preventDefault();
    //Get data to update.
    const weatherId = this.weatherList.find(
      (entry) =>
        entry.shortDescription == document.getElementById("weather").value
    ).id;
    const defaultPoseSourceId = this.poseSourceList.find(
      (entry) =>
        entry.shortDescription ==
        document.getElementById("default-pose-source").value
    ).id;
    const note = document.getElementById("leg-note").value.replace(/[^\x00-\x7F]/g, "");
    //Send put request.
    try {
      await this.legInterface.put(
        this.leg.id,
        "edit",
        weatherId,
        defaultPoseSourceId,
        note
      );
      this.element.querySelector("form button").disabled = true;
    } catch (error) {
      alert(error.message);
    }
  }
}
