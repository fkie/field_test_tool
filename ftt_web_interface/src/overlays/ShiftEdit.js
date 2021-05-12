/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";

//Structure and logic of the shift edition UI.
export class ShiftEdit {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor(
    shift,
    shiftInterface,
    performerList,
    personnelList,
    vehicleList
  ) {
    //Set arguments as properties.
    this.shift = shift;
    this.shiftInterface = shiftInterface;
    this.performerList = performerList;
    this.personnelList = personnelList;
    this.vehicleList = vehicleList;
    //Import and build shift edition node.
    const templateEl = document.getElementById("edit-shift-template");
    this.element = document.importNode(templateEl.content, true);

    //=============================== HEADER ===============================
    //Set title, start and end time.
    this.element.querySelector("header h2 span:last-of-type").textContent =
      shift.id;
    this.element.querySelector(
      "header p:first-of-type span:last-of-type"
    ).textContent = new Date(shift.startTime * 1000).toISOString();
    if (shift.endTime) {
      this.element.querySelector(
        "header p:last-of-type span:last-of-type"
      ).textContent = new Date(shift.endTime * 1000).toISOString();
    }

    //============================ DATA SECTION ============================
    //Reach to shift info data elements.
    const performerSelect = this.element.getElementById("performer");
    const testDirectorSelect = this.element.getElementById("test-director");
    const testAdminSelect = this.element.getElementById("test-admin");
    const robotOperatorSelect = this.element.getElementById("robot-operator");
    const safetyOfficerSelect = this.element.getElementById("safety-officer");
    const testIntentInput = this.element.getElementById("test-intent");
    const workspaceInput = this.element.getElementById("workspace");
    const vehicleSelect = this.element.getElementById("vehicle");
    const noteText = this.element.getElementById("shift-note");
    //Populate selects.
    DOMGeneric.populateSelect(
      performerSelect,
      performerList.map((entry) => entry.institution)
    );
    DOMGeneric.populateSelect(
      testDirectorSelect,
      personnelList.map((entry) => entry.name)
    );
    DOMGeneric.populateSelect(
      testAdminSelect,
      personnelList.map((entry) => entry.name)
    );
    DOMGeneric.populateSelect(
      robotOperatorSelect,
      personnelList.map((entry) => entry.name)
    );
    DOMGeneric.populateSelect(
      safetyOfficerSelect,
      personnelList.map((entry) => entry.name)
    );
    DOMGeneric.populateSelect(
      vehicleSelect,
      vehicleList.map((entry) => entry.shortDescription)
    );
    //Focus matching option on the selects with the properties in the shift object.
    const matchedPerformer = performerList.find(
      (entry) => entry.id == shift.performerId
    );
    if (matchedPerformer) {
      performerSelect.value = matchedPerformer.institution;
      DOMGeneric.removeFirstEmptyOption(performerSelect);
    }
    const matchedTestDirector = personnelList.find(
      (entry) => entry.id == shift.testDirectorId
    );
    if (matchedTestDirector) {
      testDirectorSelect.value = matchedTestDirector.name;
      DOMGeneric.removeFirstEmptyOption(testDirectorSelect);
    }
    const matchedTestAdmin = personnelList.find(
      (entry) => entry.id == shift.testAdministratorId
    );
    if (matchedTestAdmin) {
      testAdminSelect.value = matchedTestAdmin.name;
      DOMGeneric.removeFirstEmptyOption(testAdminSelect);
    }
    const matchedRobotOperator = personnelList.find(
      (entry) => entry.id == shift.robotOperatorId
    );
    if (matchedRobotOperator) {
      robotOperatorSelect.value = matchedRobotOperator.name;
      DOMGeneric.removeFirstEmptyOption(robotOperatorSelect);
    }
    const matchedSafetyOfficer = personnelList.find(
      (entry) => entry.id == shift.safetyOfficerId
    );
    if (matchedSafetyOfficer) {
      safetyOfficerSelect.value = matchedSafetyOfficer.name;
      DOMGeneric.removeFirstEmptyOption(safetyOfficerSelect);
    }
    const matchedVehicle = vehicleList.find(
      (entry) => entry.id == shift.vehicleId
    );
    if (matchedVehicle) {
      vehicleSelect.value = matchedVehicle.shortDescription;
      DOMGeneric.removeFirstEmptyOption(vehicleSelect);
    }
    //Fill input fields with shift data.
    testIntentInput.value = shift.testIntent;
    workspaceInput.value = shift.workspace;
    noteText.value = shift.note;
    //Set change event listeners.
    performerSelect.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    testDirectorSelect.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    testAdminSelect.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    robotOperatorSelect.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    safetyOfficerSelect.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    testIntentInput.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    workspaceInput.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    vehicleSelect.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    noteText.addEventListener(
      "change",
      this.shiftDataChangedHandler.bind(this)
    );
    //Register update button handler.
    this.element
      .querySelector("form button")
      .addEventListener("click", this.updateDataBtnHandler.bind(this));
  }

  shiftDataChangedHandler() {
    //Activate update button.
    this.element.querySelector("form button").disabled = false;
  }

  async updateDataBtnHandler(event) {
    event.preventDefault();
    //Get data to update.
    const testAdministratorId = this.personnelList.find(
      (entry) => entry.name == document.getElementById("test-admin").value
    ).id;
    const testDirectorId = this.personnelList.find(
      (entry) => entry.name == document.getElementById("test-director").value
    ).id;
    const safetyOfficerId = this.personnelList.find(
      (entry) => entry.name == document.getElementById("safety-officer").value
    ).id;
    const robotOperatorId = this.personnelList.find(
      (entry) => entry.name == document.getElementById("robot-operator").value
    ).id;
    const performer = this.performerList.find(
      (entry) => entry.institution == document.getElementById("performer").value
    ).id;
    const testIntent = document.getElementById("test-intent").value;
    const workspace = document.getElementById("workspace").value;
    const vehicleId = this.vehicleList.find(
      (entry) =>
        entry.shortDescription == document.getElementById("vehicle").value
    ).id;
    const note = document.getElementById("shift-note").value;
    //Send put request.
    try {
      await this.shiftInterface.put(
        this.shift.id,
        "edit",
        testAdministratorId,
        testDirectorId,
        safetyOfficerId,
        robotOperatorId,
        performer,
        testIntent,
        workspace,
        vehicleId,
        note
      );
      this.element.querySelector("form button").disabled = true;
    } catch (error) {
      alert(error.message);
    }
  }
}
