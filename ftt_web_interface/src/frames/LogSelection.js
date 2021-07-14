/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";
import { Modal } from "../utility/Modal.js";
import { TestEventInterface } from "../database_interface/TestEvent.js";
import { ShiftInterface } from "../database_interface/Shift.js";
import { LegInterface } from "../database_interface/Leg.js";
import { PerformerInterface } from "../database_interface/Performer.js";
import { PersonnelInterface } from "../database_interface/Personnel.js";
import { VehicleInterface } from "../database_interface/Vehicle.js";
import { WeatherInterface } from "../database_interface/Weather.js";
import { PoseSourceInterface } from "../database_interface/PoseSource.js";
import { TestEventEdit } from "../overlays/TestEventEdit.js";
import { ShiftEdit } from "../overlays/ShiftEdit.js";
import { LegEdit } from "../overlays/LegEdit.js";

class Log {
  //Helper base class to contain log related data and functions.
  constructor(
    name,
    dataInterface,
    parentSelect = null,
    childClass = null,
    checkSelectionCallback
  ) {
    //Set arguments as properties.
    this.name = name;
    this.dataInterface = dataInterface;
    this.parentSelect = parentSelect;
    this.childClass = childClass;
    this.checkSelectionCallback = checkSelectionCallback;
    //Get DOM element hooks.
    this.select = document.getElementById(`${name}-id`);
    this.newBtn = document.getElementById(`new-${name}-btn`);
    this.endBtn = document.getElementById(`end-${name}-btn`);
    this.editBtn = document.getElementById(`edit-${name}-btn`);
    //Initialize variables.
    this.postData = null; //To be redefined by child class.
    this.dataList = [];
    //Add event listeners
    this.select.addEventListener("change", this.selectChangeHandler.bind(this));
    this.newBtn.addEventListener("click", this.newBtnClickHandler.bind(this));
    this.endBtn.addEventListener("click", this.endBtnClickHandler.bind(this));
    this.editBtn.addEventListener("click", this.editBtnClickHandler.bind(this));
  }

  clearSelectData() {
    //Clear the child select data.
    if (this.childClass) {
      this.childClass.clearSelectData();
    }
    //Clear own select data.
    DOMGeneric.clearChildren(this.select);
  }

  async updateData() {
    this.dataList = await this.dataInterface.get(
      this.parentSelect && this.parentSelect.value
    );
  }

  async updateSelect() {
    //Check parent select value.
    if (this.parentSelect && !this.parentSelect.value) {
      alert("Please select an ID in the parent select first.");
      return;
    }
    try {
      //Get table data from server.
      await this.updateData();
      //Fill the specified select with the fetched data.
      DOMGeneric.populateSelect(
        this.select,
        this.dataList.map((entry) => entry.id).reverse(),
        this.dataList
          .map((entry) => (entry.endTime ? "closed-log" : "open-log"))
          .reverse()
      );
      //Clear the child select data.
      if (this.childClass) {
        this.childClass.clearSelectData();
      }
    } catch (error) {
      alert(error.message);
    }
  }

  async selectChangeHandler() {
    //Remove top blank option.
    DOMGeneric.removeFirstEmptyOption(this.select);
    //Update the child options.
    if (this.childClass) {
      await this.childClass.updateSelect();
    }
    //Check log IDs, hide segments and map
    this.checkSelectionCallback();
  }

  async newBtnClickHandler() {
    try {
      //Send request for the creation of a new log entry.
      await this.dataInterface.post(...this.postData);
      //Update the select options.
      await this.updateSelect();
      //Remove top blank option.
      DOMGeneric.removeFirstEmptyOption(this.select);
      //Choose the newly created log entry from the select.
      this.select.value = Math.max(...this.dataList.map((entry) => entry.id));
      //Trigger the edit behavior.
      await this.editBtnClickHandler();
      //Check log IDs, hide segments and map
      this.checkSelectionCallback();
    } catch (error) {
      alert(error.message);
    }
  }

  async recursiveUpdate() {
    //Method to update the dataList and select options after a put request.
    //This won't rebuild the selects, but rather update the log status.
    //Child recursiveUpdate.
    if (this.childClass) {
      await this.childClass.recursiveUpdate();
    }
    if (!this.parentSelect || (this.parentSelect && this.parentSelect.value)) {
      //Update this dataList:
      await this.updateData();
      //Change the select option html class for the all closed logs.
      Array.from(this.select.children).forEach((option) => {
        if (option.value) {
          //Find the log entry for the option.
          const matchedEntry = this.dataList.find(
            (entry) => entry.id == option.value
          );
          //Update option html class if the entry is closed.
          if (matchedEntry.endTime) {
            option.className = "closed-log";
          }
        }
      });
    }
  }

  async endBtnClickHandler() {
    //Check select value.
    if (!this.select.value) {
      alert("Please select an ID to close.");
      return;
    }
    try {
      //Close log entry.
      await this.dataInterface.put(this.select.value, "close");
      //Update dataList and select options' status for this and all the children classes.
      await this.recursiveUpdate();
      //Check log IDs, hide segments and map
      this.checkSelectionCallback();
    } catch (error) {
      alert(error.message);
    }
  }

  async editBtnClickHandler() {
    //Display the edit window build in the child function.
    const modal = new Modal(
      this.editContainer,
      "Your browser does't support this feature! - Please change to a more modern one.",
      this.updateData.bind(this)
    );
    modal.show();
  }
}

class TestEventLog extends Log {
  //Specific test event class to deal with log interactions.
  constructor(serverInterface, childClass, checkSelectionCallback) {
    //Parent constructor.
    super(
      "test-event",
      new TestEventInterface(serverInterface),
      null,
      childClass,
      checkSelectionCallback
    );
  }

  async newBtnClickHandler() {
    //Set postData.
    this.postData = [null, null, null, null];
    //Call parent function.
    await super.newBtnClickHandler();
  }

  async editBtnClickHandler() {
    //Check select value.
    if (!this.select.value) {
      alert("Please select an ID to edit.");
      return;
    }
    //Find the selected log entry.
    const selectedLog = this.dataList.find(
      (entry) => entry.id == this.select.value
    );
    //Build the edit window.
    this.editContainer = new TestEventEdit(selectedLog, this.dataInterface);
    //Call parent function.
    await super.editBtnClickHandler();
  }
}

class ShiftLog extends Log {
  //Specific shift class to deal with log interactions.
  constructor(
    serverInterface,
    childClass,
    checkSelectionCallback,
    currentUser
  ) {
    //Parent constructor.
    super(
      "shift",
      new ShiftInterface(serverInterface),
      document.getElementById("test-event-id"),
      childClass,
      checkSelectionCallback
    );
    //Initialize child variables and objects.
    this.currentUser = currentUser;
    this.performerInterface = new PerformerInterface(serverInterface);
    this.personnelInterface = new PersonnelInterface(serverInterface);
    this.vehicleInterface = new VehicleInterface(serverInterface);
    this.performerList = null;
    this.personnelList = null;
    this.vehicleList = null;
  }

  async newBtnClickHandler() {
    //Make sure a user is selected from the interface.
    if (!this.currentUser.id) {
      alert("Please select a user to add a new shift!");
      return;
    }
    //Fetch performer and vehicle data from server.
    try {
      this.performerList = await this.performerInterface.get();
      this.vehicleList = await this.vehicleInterface.get();
    } catch (error) {
      alert(error.message);
      return;
    }
    //If there are no performers or vehicles in the list, alert the user.
    if (this.performerList.length == 0 || this.vehicleList.length == 0) {
      alert(
        "Please configure a performer and a vehicle before trying to add a new shift!"
      );
      return;
    }
    //Find the performer that matches the selected user. If none matches, use the first one.
    let matchedPerformer =
      this.performerList.find(
        (performer) => performer.institution == this.currentUser.institution
      ) || this.performerList[0];
    //Find a vehicle that matches the selected user's institution. If none matches, use the first one.
    let matchedVehicle =
      this.vehicleList.find(
        (vehicle) => vehicle.institution == this.currentUser.institution
      ) || this.vehicleList[0];
    //Set postData.
    this.postData = [
      this.parentSelect.value,
      this.currentUser.id,
      this.currentUser.id,
      this.currentUser.id,
      this.currentUser.id,
      matchedPerformer.id,
      null,
      null,
      matchedVehicle.id,
      null,
    ];
    //Call parent function.
    await super.newBtnClickHandler();
  }

  async editBtnClickHandler() {
    //Check select value.
    if (!this.select.value) {
      alert("Please select an ID to edit.");
      return;
    }
    //Find the selected log entry.
    const selectedLog = this.dataList.find(
      (entry) => entry.id == this.select.value
    );
    //Fetch performer, personnel and vehicle data from server.
    try {
      this.performerList = await this.performerInterface.get();
      this.personnelList = await this.personnelInterface.get();
      this.vehicleList = await this.vehicleInterface.get();
    } catch (error) {
      alert(error.message);
      return;
    }
    //Build the edit window.
    this.editContainer = new ShiftEdit(
      selectedLog,
      this.dataInterface,
      this.performerList,
      this.personnelList,
      this.vehicleList
    );
    //Call parent function.
    await super.editBtnClickHandler();
  }
}

class LegLog extends Log {
  //Specific leg class to deal with log interactions.
  constructor(serverInterface, checkSelectionCallback) {
    //Parent constructor.
    super(
      "leg",
      new LegInterface(serverInterface),
      document.getElementById("shift-id"),
      null,
      checkSelectionCallback
    );
    //Initialize child variables and objects.
    this.weatherInterface = new WeatherInterface(serverInterface);
    this.poseSourceInterface = new PoseSourceInterface(serverInterface);

    this.weatherList = null;
    this.poseSourceList = null;
  }

  async newBtnClickHandler() {
    //Fetch weather and pose source data from server.
    try {
      //Weather data is static. Skip if already fetched.
      if (!this.weatherList || this.weatherList.length == 0) {
        this.weatherList = await this.weatherInterface.get();
      }
      this.poseSourceList = await this.poseSourceInterface.get();
    } catch (error) {
      alert(error.message);
      return;
    }
    //If there are no weather states, alert the user.
    if (this.weatherList.length == 0) {
      alert(
        "Weather list empty. Your database was not properly initialized. Please fix this before continuing."
      );
      return;
    }
    //If the pose source list is empty, alert the user.
    if (this.poseSourceList.length == 0) {
      alert("Please configure a pose source before trying to add a new leg!");
      return;
    }
    //Set postData.
    this.postData = [
      this.parentSelect.value,
      this.weatherList[0].id,
      this.poseSourceList[0].id,
      null,
    ];
    //Call parent function.
    await super.newBtnClickHandler();
  }

  async editBtnClickHandler() {
    //Check select value.
    if (!this.select.value) {
      alert("Please select an ID to edit.");
      return;
    }
    //Find the selected log entry.
    const selectedLog = this.dataList.find(
      (entry) => entry.id == this.select.value
    );
    //Fetch weather and pose source data from server.
    try {
      //Weather data is static. Skip if already fetched.
      if (!this.weatherList || this.weatherList.length == 0) {
        this.weatherList = await this.weatherInterface.get();
      }
      this.poseSourceList = await this.poseSourceInterface.get();
    } catch (error) {
      alert(error.message);
      return;
    }
    //Build the edit window.
    this.editContainer = new LegEdit(
      selectedLog,
      this.dataInterface,
      this.weatherList,
      this.poseSourceList
    );
    //Call parent function.
    await super.editBtnClickHandler();
  }
}

export class LogSelection {
  //Container class for the log selection section of the web page.
  constructor(serverInterface, currentUser) {
    //Initialize log objects.
    this.legLog = new LegLog(
      serverInterface,
      this.checkSelectedLogs.bind(this)
    );
    this.shiftLog = new ShiftLog(
      serverInterface,
      this.legLog,
      this.checkSelectedLogs.bind(this),
      currentUser
    );
    this.testEventLog = new TestEventLog(
      serverInterface,
      this.shiftLog,
      this.checkSelectedLogs.bind(this)
    );
    //Trigger init behavior.
    this.init();
  }

  async init() {
    this.testEventLog.updateSelect();
  }

  checkSelectedLogs() {
    //Hide the segment details and map
    document.getElementById("segment-detail").style.display = "none";
    document.getElementById("map-viewer").style.display = "none";
    //Only enable the get segments button if all log ids have been selected.
    const getSegmentsBtn = document.getElementById("get-segments-btn");
    if (
      this.testEventLog.select.value &&
      this.shiftLog.select.value &&
      this.legLog.select.value
    ) {
      getSegmentsBtn.disabled = false;
    } else {
      getSegmentsBtn.disabled = true;
    }
    //Guarantee the segment auto-refresh isn't active.
    const autoRefreshBox = document.getElementById("auto-refresh");
    if (autoRefreshBox.checked) {
      autoRefreshBox.click();
    }
    //Guarantee the gps map disabled.
    const gpsBox = document.getElementById("gps-map-box");
    if (gpsBox.checked) {
      gpsBox.click();
    }
    //Guarantee the gps map disabled.
    const localBox = document.getElementById("local-map-box");
    if (localBox.checked) {
      localBox.click();
    }
  }
}
