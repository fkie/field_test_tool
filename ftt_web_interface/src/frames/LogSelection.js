/* eslint-disable no-undef */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";
import { Modal } from "../utility/Modal.js";
import { ConfirmationModal } from "../utility/ConfirmationModal.js";
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
    this.checkSelectionFcn = checkSelectionCallback;
    //Get DOM element hooks.
    this.statusDot = document.getElementById(`${name}-status-dot`);
    this.select = document.getElementById(`${name}-id`);
    this.newBtn = document.getElementById(`new-${name}-btn`);
    this.endBtn = document.getElementById(`end-${name}-btn`);
    this.editBtn = document.getElementById(`edit-${name}-btn`);
    this.deleteBtn = document.getElementById(`delete-${name}-btn`);
    //Initialize variables.
    this.postData = null; //To be redefined by child class.
    this.dataList = [];
    this.open = false;
    //Add event listeners.
    this.select.addEventListener("change", this.selectChangeHandler.bind(this));
    this.newBtn.addEventListener("click", this.newBtnClickHandler.bind(this));
    this.endBtn.addEventListener("click", this.endBtnClickHandler.bind(this));
    this.editBtn.addEventListener("click", this.editBtnClickHandler.bind(this));
    this.deleteBtn.addEventListener(
      "click",
      this.deleteBtnClickHandler.bind(this)
    );
  }

  checkSelectionCallback() {
    //Execute callback passed in constructor's arguments.
    this.checkSelectionFcn();
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
    //Update open log status
    this.open = !this.dataList.find((entry) => entry.id == this.select.value)
      .endTime;
    //Update the child options.
    if (this.childClass) {
      await this.childClass.updateSelect();
    }
    if (this.childClass && this.childClass.dataList.length === 1) {
      //If the child only has one option, select it and call recursion.
      this.childClass.select.value = this.childClass.dataList[0].id;
      this.childClass.selectChangeHandler();
    } else {
      //Else finish: Check log IDs, hide segments and map.
      this.checkSelectionCallback();
    }
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
      //Update open log status.
      this.open = true;
      //Trigger the edit behavior.
      await this.editBtnClickHandler();
      //Check log IDs, hide segments and map.
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
      //Update open log status
      if (this.select.value) {
        this.open = !this.dataList.find(
          (entry) => entry.id == this.select.value
        ).endTime;
      }
      //Change the select option html class for the logs.
      Array.from(this.select.children).forEach((option) => {
        if (option.value) {
          //Find the log entry for the option.
          const matchedEntry = this.dataList.find(
            (entry) => entry.id == option.value
          );
          //Update the option's html class.
          option.className = matchedEntry.endTime ? "closed-log" : "open-log";
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
      //Stop logging.
      const startLoggingBtn = document.getElementById("start-logging-btn");
      if (startLoggingBtn.textContent === "Stop Logging") {
        startLoggingBtn.click();
      }
      //Close log entry.
      await this.dataInterface.put(this.select.value, "close");
      //Update dataList and select options' status for this and all the children classes.
      await this.recursiveUpdate();
      //Check log IDs, hide segments and map.
      this.checkSelectionCallback();
    } catch (error) {
      alert(error.message);
    }
  }

  async editBtnClickHandler() {
    //Display the edit window built in the child function.
    const modal = new Modal(
      this.editContainer,
      "Your browser does't support this feature! - Please change to a more modern one.",
      this.updateData.bind(this)
    );
    modal.show();
  }

  async deleteLogEntry() {
    try {
      //Send request for the deletion of the current log entry.
      await this.dataInterface.delete(this.select.value);
      //Update the select options.
      await this.updateSelect();
      //Reset log status.
      this.open = false;
      //Check log IDs, hide segments and map.
      this.checkSelectionCallback();
    } catch (error) {
      alert(error.message);
    }
  }

  async deleteBtnClickHandler() {
    //Make sure an id selected.
    if (!this.select.value) {
      alert("Please select an ID to delete!");
      return;
    }
    //Prompt confirmation.
    const modal = new ConfirmationModal(
      "Warning!",
      `<strong>This operation cannot be undone!</strong>
      Are you sure you want to delete <b>${this.name.replace(/-/g, " ")} ${
        this.select.value
      }</b> and all its children?`,
      "Your browser does't support this feature! - Please change to a more modern one.",
      this.deleteLogEntry.bind(this) //Delete the entry if confirmed.
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
    //Add option to open the test event.
    this.openBtn = document.getElementById("open-test-event-btn");
    this.openBtn.addEventListener("click", this.openBtnClickHandler.bind(this));
  }

  async newBtnClickHandler() {
    //Set postData.
    this.postData = [null, null, null, null];
    //Call parent function.
    await super.newBtnClickHandler();
  }

  async openBtnClickHandler() {
    //Check select value.
    if (!this.select.value) {
      alert("Please select an ID to open.");
      return;
    }
    try {
      //Open log entry.
      await this.dataInterface.put(this.select.value, "open");
      //Update dataList and select options' status for this and all the children classes.
      await this.recursiveUpdate();
      //Check log IDs, hide segments and map.
      this.checkSelectionCallback();
    } catch (error) {
      alert(error.message);
    }
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
    //Make sure a test event is selected.
    if (!this.parentSelect.value) {
      alert("Please select a test event to add a new shift!");
      return;
    }
    //Make sure a user is selected from the interface.
    if (!this.currentUser.id) {
      document.getElementById("user-icon").click();
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
    //Make sure a shift is selected.
    if (!this.parentSelect.value) {
      alert("Please select a shift to add a new leg!");
      return;
    }
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
  constructor(serverInterface, rosInterface, currentUser, doneCallback) {
    //Set arguments as properties.
    this.ros = rosInterface;
    this.doneCallback = doneCallback;
    //Reach to DOM elements.
    this.startLoggingBtn = document.getElementById("start-logging-btn");
    this.autoRefreshBox = document.getElementById("auto-refresh");
    //Initialize variables.
    this.isLogging = false;
    this.setLoggingClient = new ROSLIB.Service({
      ros: this.ros,
      name: "/set_ftt_logging",
      serviceType: "std_srvs/SetBool",
    });
    this.getLoggingClient = new ROSLIB.Service({
      ros: this.ros,
      name: "/get_ftt_logging",
      serviceType: "std_srvs/Trigger",
    });
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
    //Add event listeners.
    this.startLoggingBtn.addEventListener(
      "click",
      this.loggingBtnClickHandler.bind(this)
    );
    //Trigger init behavior.
    this.init();
  }

  async init() {
    await this.testEventLog.updateSelect();
    const personnelList = await this.shiftLog.personnelInterface.get();
    if (personnelList.length > 0) {
      document.getElementById("user-icon").click();
    } else {
      //Prompt configuration notice.
      const modal = new ConfirmationModal(
        "Welcome!",
        `It looks like you haven't configured any users yet. 
        Please input at least one entry for each <strong>Database Configuration</strong> table. 
        Clicking on the top left cogwheel of the main page or on the <i>confirm</i> button below will take you to the configuration page.`,
        "Your browser does't support this feature! - Please change to a more modern one.",
        () => {
          document.getElementById("config-icon").click();
        }
      );
      modal.show();
    }
  }

  toggleLogging() {
    //Toggle logging status.
    this.isLogging = !this.isLogging;
    //Update button display.
    if (!this.isLogging) {
      this.startLoggingBtn.textContent = "Start Logging";
    } else {
      this.startLoggingBtn.textContent = "Stop Logging";
    }
  }

  updateLogging() {
    //If ROS is connected
    if (this.ros.isConnected) {
      //Check current logging status in ros.
      const request = new ROSLIB.ServiceRequest();
      this.getLoggingClient.callService(request, (result) => {
        if (result.success != this.isLogging) {
          //Toggle logging to match status in ros.
          this.toggleLogging();
        }
        //If all log ids have been selected.
        if (
          this.testEventLog.select.value &&
          this.shiftLog.select.value &&
          this.legLog.select.value
        ) {
          //Enable the logging button if all logs are open or if the logging status is active and all the logs are closed.
          let forceActivate = false;
          if (this.isLogging) {
            //Find if all test events are closed.
            const testEventsClosed =
              !this.testEventLog.open &&
              this.testEventLog.dataList.reduce(
                (closed, curr) => closed && !!curr.endTime,
                true
              );
            if (testEventsClosed) {
              forceActivate = true;
            } else if (this.testEventLog.open) {
              //This test event is open. Find if all shifts are closed.
              const shiftsClosed =
                !this.shiftLog.open &&
                this.shiftLog.dataList.reduce(
                  (closed, curr) => closed && !!curr.endTime,
                  true
                );
              if (shiftsClosed) {
                forceActivate = true;
              } else if (this.shiftLog.open) {
                //This shift is open. Find if all legs are closed.
                const legsClosed =
                  !this.legLog.open &&
                  this.legLog.dataList.reduce(
                    (closed, curr) => closed && !!curr.endTime,
                    true
                  );
                if (legsClosed) {
                  forceActivate = true;
                }
              }
            }
          }
          if (
            (this.testEventLog.open &&
              this.shiftLog.open &&
              this.legLog.open) ||
            forceActivate
          ) {
            this.startLoggingBtn.disabled = false;
          } else {
            this.startLoggingBtn.disabled = true;
          }
          //Enable the auto refresh box if all logs are open.
          if (
            this.testEventLog.open &&
            this.shiftLog.open &&
            this.legLog.open
          ) {
            this.autoRefreshBox.disabled = false;
          } else {
            if (this.autoRefreshBox.checked) {
              this.autoRefreshBox.click();
            }
            this.autoRefreshBox.disabled = true;
          }
          //Activate the auto refresh box if logging, deactivate it if not.
          if (this.isLogging) {
            if (!this.autoRefreshBox.checked) {
              this.autoRefreshBox.click();
            }
          } else {
            if (this.autoRefreshBox.checked) {
              this.autoRefreshBox.click();
            }
          }
        }
      });
    } else {
      this.startLoggingBtn.disabled = true;
      if (this.autoRefreshBox.checked) {
        this.autoRefreshBox.click();
      }
      this.autoRefreshBox.disabled = true;
    }
  }

  loggingBtnClickHandler() {
    //Call ros service to start/stop logging.
    const request = new ROSLIB.ServiceRequest({
      data: !this.isLogging,
    });
    this.setLoggingClient.callService(request, (result) => {
      if (result.success) {
        //Update the table.
        this.doneCallback();
        //Update the logging status
        this.updateLogging();
      } else {
        alert(result.message);
      }
    });
  }

  checkSelectedLogs() {
    //Hide the segment details and map.
    document.getElementById("segment-detail").style.display = "none";
    document.getElementById("map-viewer").style.display = "none";
    //Update status dot for all logs
    const logs = [this.testEventLog, this.shiftLog, this.legLog];
    for (const log of logs) {
      if (log.select.value) {
        if (log.open) {
          log.statusDot.classList.remove("status-red");
          log.statusDot.classList.add("status-green");
        } else {
          log.statusDot.classList.remove("status-green");
          log.statusDot.classList.add("status-red");
        }
      } else {
        log.statusDot.classList.remove("status-red");
        log.statusDot.classList.remove("status-green");
      }
    }
    //Update end/open option for test event
    if (this.testEventLog.open) {
      this.testEventLog.endBtn.style.display = "grid";
      this.testEventLog.openBtn.style.display = "none";
    } else {
      this.testEventLog.endBtn.style.display = "none";
      this.testEventLog.openBtn.style.display = "grid";
    }
    //Guarantee the segment auto-refresh isn't active.
    if (this.autoRefreshBox.checked) {
      this.autoRefreshBox.click();
    }
    //Trigger callback if all log ids have been selected.
    if (
      this.testEventLog.select.value &&
      this.shiftLog.select.value &&
      this.legLog.select.value
    ) {
      this.doneCallback();
      //Update the logging status.
      this.updateLogging();
    } else {
      this.startLoggingBtn.disabled = true;
      this.autoRefreshBox.disabled = true;
    }
    //Guarantee the gps map is disabled.
    const gpsBox = document.getElementById("gps-map-box");
    if (gpsBox.checked) {
      gpsBox.click();
    }
    //Guarantee the local map is disabled.
    const localBox = document.getElementById("local-map-box");
    if (localBox.checked) {
      localBox.click();
    }
  }
}
