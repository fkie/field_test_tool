/* eslint-disable no-undef */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";
import { PerformerInterface } from "../database_interface/Performer.js";
import { PersonnelInterface } from "../database_interface/Personnel.js";
import { PoseSourceInterface } from "../database_interface/PoseSource.js";
import { VehicleInterface } from "../database_interface/Vehicle.js";
import { RosParamsInterface } from "../ros_interface/Parameters.js";
import { RosTopicsInterface } from "../ros_interface/Topics.js";
import { Modal } from "../utility/Modal.js";
import { RosConfig } from "../overlays/RosConfig.js";

export class ConfigFrame {
  constructor(serverInterface) {
    //Get html hooks.
    const titleHooks = [
      document.getElementById("performer"),
      document.getElementById("personnel"),
      document.getElementById("pose-source"),
      document.getElementById("vehicle"),
      document.getElementById("parameters"),
      document.getElementById("topics"),
    ];
    const newEntryHooks = [
      document.getElementById("performer-table-body").lastElementChild,
      document.getElementById("personnel-table-body").lastElementChild,
      document.getElementById("pose-source-table-body").lastElementChild,
      document.getElementById("vehicle-table-body").lastElementChild,
    ];
    this.rosConnectIcon = document.getElementById("ros-connect-icon");
    const saveRosParamsHook = document.getElementById(
      "parameters-table-body"
    ).lastElementChild;
    const saveRosTopicsHook =
      document.getElementById("topics-table-body").lastElementChild;

    //Initialize variables.
    this.forceRosConnect = false;
    this.ros = new ROSLIB.Ros();
    this.ros_server_adr = `ws://${location.hostname}:9090`;
    //Build database tables interface.
    this.dataInterfaces = [
      new PerformerInterface(serverInterface),
      new PersonnelInterface(serverInterface),
      new PoseSourceInterface(serverInterface),
      new VehicleInterface(serverInterface),
      new RosParamsInterface(this.ros, "/ftt_ros/params/"),
      new RosTopicsInterface(this.ros, "/ftt_ros/topics/"),
    ];

    //Append event listeners:
    //1.Display table listeners.
    for (let i = 0; i < titleHooks.length; i++) {
      titleHooks[i].addEventListener(
        "click",
        this.titleClickHandler.bind(this, this.dataInterfaces[i])
      );
    }
    //2.Add row listeners.
    for (let i = 0; i < newEntryHooks.length; i++) {
      newEntryHooks[i].addEventListener(
        "click",
        this.inputNewEntryClickHandler.bind(this, this.dataInterfaces[i])
      );
    }
    //3.Reset add row listeners.
    for (let i = 0; i < newEntryHooks.length; i++) {
      newEntryHooks[i].addEventListener(
        "focusout",
        this.newEntryDefocusHandler.bind(this)
      );
    }
    //4.Save ROS config.
    saveRosParamsHook.addEventListener("click", this.saveRosParams.bind(this));
    saveRosTopicsHook.addEventListener("click", this.saveRosParams.bind(this));
    //5.ROS connection.
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

  static resetNewEntryRow(rowHook) {
    //Remove row content other than first cell.
    while (rowHook.children.length > 1) {
      rowHook.lastElementChild.remove();
    }
    //Force row grid style to "none".
    rowHook.style.gridTemplateColumns = "none";
  }

  insertTableIcons(tableBodyRef, dataInterface) {
    Array.from(tableBodyRef.children).forEach((row) => {
      //check available methods in dataInterface.
      const methods = [];
      const callbacks = [];
      if (typeof dataInterface.put === "function") {
        methods.push("edit");
        callbacks.push(this.editEntryClickHandler.bind(this, dataInterface));
      }
      if (typeof dataInterface.delete === "function") {
        methods.push("clear");
        callbacks.push(this.clearEntryClickHandler.bind(this, dataInterface));
      }
      //Create an icon container.
      const container = DOMGeneric.createMaterialIconsContainer(
        "div",
        "table-icon-container",
        methods,
        callbacks
      );
      //Add the container at the end of the row.
      row.appendChild(container);
    });
  }

  async updateTable(dataInterface, tableBodyHook) {
    //Get data.
    const dataList = await dataInterface.get();
    //Cache last table row.
    const lastRow = tableBodyHook.lastElementChild;
    //Fill table.
    DOMGeneric.populateTable(
      tableBodyHook,
      dataList.map((entry) => Object.values(entry))
    );
    this.insertTableIcons(tableBodyHook, dataInterface);
    tableBodyHook.appendChild(lastRow);
  }

  async titleClickHandler(dataInterface, event) {
    //Select the whole title div.
    const target = event.target.closest("div");
    //Select the table div.
    const contentHook = target.nextElementSibling;
    if (contentHook.style.display == "block") {
      //Hide table div and round title edges.
      target.style.borderRadius = "10px";
      target.lastElementChild.textContent = "add";
      contentHook.style.display = "none";
    } else {
      //Get table body hook.
      const tableBodyHook = contentHook.querySelector("tbody");
      try {
        //Update table body
        await this.updateTable(dataInterface, tableBodyHook);
        //Sharpen title lower edges
        target.style.borderRadius = "10px 10px 0 0";
        target.lastElementChild.textContent = "remove";
        //Show table.
        contentHook.style.display = "block";
      } catch (error) {
        alert(error.message);
      }
    }
  }

  async putEntryClickHandler(dataInterface, event) {
    //Select the input row.
    const dataRow = event.target.closest("tr");
    //Gather input data.
    let putData = [dataRow.children[0].textContent];
    let missingData = false;
    for (let i = 1; i < dataInterface.paramNames.length; i++) {
      const colValue = dataRow.children[i].value;
      if (!colValue) {
        dataRow.children[i].required = true;
        missingData = true;
      }
      putData.push(colValue);
    }
    //Return if there are missing data.
    if (missingData) {
      return;
    }
    //Put entry through dataInterface.
    try {
      await dataInterface.put(...putData);
      await this.updateTable(dataInterface, dataRow.closest("tbody"));
    } catch (error) {
      alert(error.message);
    }
  }

  async postEntryClickHandler(dataInterface, event) {
    //Select the input row.
    const dataRow = event.target.closest("tr");
    //Gather input data.
    let postData = [];
    let missingData = false;
    for (let i = 1; i < dataInterface.paramNames.length; i++) {
      const colValue =
        dataRow.children[i].value || dataRow.children[i].placeholder;
      if (!colValue) {
        dataRow.children[i].required = true;
        missingData = true;
      }
      postData.push(colValue);
    }
    //Return if there are missing data.
    if (missingData) {
      return;
    }
    //Post entry through dataInterface.
    try {
      await dataInterface.post(...postData);
      await this.updateTable(dataInterface, dataRow.closest("tbody"));
      ConfigFrame.resetNewEntryRow(dataRow);
    } catch (error) {
      alert(error.message);
    }
  }

  async editEntryDefocusHandler(dataInterface, event) {
    //Select the row in edition.
    const editRow = event.target.closest("tr");
    //Check if the focus was lost outside the row or to a non-focusable element.
    if (!event.relatedTarget || event.relatedTarget.closest("tr") != editRow) {
      //Restore the table.
      try {
        await this.updateTable(dataInterface, editRow.closest("tbody"));
      } catch (error) {
        alert(error.message);
      }
    }
  }

  async editEntryClickHandler(dataInterface, event) {
    //Select the row to edit.
    const editRow = event.target.closest("tr");
    //Append focusout event listener to the row.
    editRow.addEventListener(
      "focusout",
      this.editEntryDefocusHandler.bind(this, dataInterface)
    );
    //Change table data cells' type for edition.
    for (let i = 1; i < dataInterface.paramNames.length; i++) {
      const newInput = document.createElement("input");
      newInput.type = "text";
      newInput.className = "new-entry-input";
      newInput.value = editRow.children[i].textContent;
      editRow.children[i].replaceWith(newInput);
      //Workaround for the topics interface: add a datalist to the input.
      if (dataInterface instanceof RosTopicsInterface) {
        let dataList = document.getElementById("topics-list");
        if (!dataList) {
          dataList = document.createElement("datalist");
          dataList.id = "topics-list";
          newInput.insertAdjacentElement("afterend", dataList);
        }
        const options = await dataInterface.getOptions(
          editRow.children[0].textContent
        );
        DOMGeneric.populateSelect(dataList, options);
        DOMGeneric.removeFirstEmptyOption(dataList);
        newInput.setAttribute("list", "topics-list");
      }
    }
    //Change row icons to done icon.
    const doneCell = DOMGeneric.createMaterialIconsContainer(
      "button",
      "table-icon-container",
      ["done"]
    );
    editRow.lastElementChild.replaceWith(doneCell);
    //Append event listener to the button (not to the icon, so it can be triggered with an "enter").
    doneCell.addEventListener(
      "click",
      this.putEntryClickHandler.bind(this, dataInterface)
    );
    //Focus first input field.
    editRow.children[1].focus();
  }

  async clearEntryClickHandler(dataInterface, event) {
    //Select the row to clear.
    const clearRow = event.target.closest("tr");
    //Get entry id.
    const clearId = clearRow.firstElementChild.textContent;
    try {
      //Remove entry from server.
      await dataInterface.delete(clearId);
      //Update table.
      await this.updateTable(dataInterface, clearRow.closest("tbody"));
    } catch (error) {
      alert(error.message);
    }
  }

  inputNewEntryClickHandler(dataInterface, event) {
    //Select last table row.
    const target = event.target.closest("tr");
    //Check row grid style property.
    if (target.style.gridTemplateColumns) {
      //Remove row grid property (should be "none") to default to table grid style.
      target.style.removeProperty("grid-template-columns");
      //Insert input fields for new entry.
      for (let i = 1; i < dataInterface.paramNames.length; i++) {
        const newInput = document.createElement("input");
        newInput.type = "text";
        newInput.className = "new-entry-input";
        //Insert default value for secondary fields.
        if (
          dataInterface.paramNames[i] == "long_description" ||
          dataInterface.paramNames[i] == "configuration"
        ) {
          newInput.placeholder = "--";
        }
        target.appendChild(newInput);
      }
      //Insert done icon in last cell.
      const doneCell = DOMGeneric.createMaterialIconsContainer(
        "button",
        "table-icon-container",
        ["done"]
      );
      target.appendChild(doneCell);
      //Append event listener to the button (not to the icon, so it can be triggered with an "enter").
      doneCell.addEventListener(
        "click",
        this.postEntryClickHandler.bind(this, dataInterface)
      );
      //Focus first input field.
      target.children[1].focus();
    }
  }

  newEntryDefocusHandler(event) {
    //Select last table row.
    const target = event.target.closest("tr");
    //Check if the focus was lost outside the row or to a non-focusable element.
    if (!event.relatedTarget || event.relatedTarget.closest("tr") != target) {
      //Reset row state.
      ConfigFrame.resetNewEntryRow(target);
    }
  }

  async saveRosParams() {
    //Call ros service to save parameters to file.
    const client = new ROSLIB.Service({
      ros: this.ros,
      name: "/save_ftt_params",
      serviceType: "std_srvs/Trigger",
    });
    const request = new ROSLIB.ServiceRequest();
    client.callService(
      request,
      function (result) {
        if (result.success) {
          alert("Request succeeded! " + result.message);
        } else {
          alert("Request failed! " + result.message);
        }
      },
      function (message) {
        alert(message);
      }
    );
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

  rosConnect() {
    this.ros.connect(this.ros_server_adr);
  }

  rosErrorHandler() {
    console.log("Error connecting to ROS");
    if (this.forceRosConnect) {
      alert("Error connecting to ROS");
      this.forceRosConnect = false;
    }
  }

  rosConnectionHandler() {
    this.rosConnectIcon.classList.remove("disconnected");
    this.rosConnectIcon.classList.add("connected");
    this.rosConnectIcon.textContent = "sensors";
    console.log("Connection to ROS established.");
  }

  rosCloseHandler() {
    this.rosConnectIcon.classList.remove("connected");
    this.rosConnectIcon.classList.add("disconnected");
    this.rosConnectIcon.textContent = "sensors_off";
    console.log("Connection to ROS closed.");
    //Close any open table to prevent parameter modification attempts.
    const paramsDiv = document.getElementById("parameters");
    const topicsDiv = document.getElementById("topics");
    if (paramsDiv.nextElementSibling.style.display == "block") {
      paramsDiv.click();
    }
    if (topicsDiv.nextElementSibling.style.display == "block") {
      topicsDiv.click();
    }
  }
}
