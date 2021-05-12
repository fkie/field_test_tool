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

export class ConfigFrame {
  constructor(serverInterface) {
    //Get html hooks.
    this.titleHooks = [
      document.getElementById("performer"),
      document.getElementById("personnel"),
      document.getElementById("pose-source"),
      document.getElementById("vehicle"),
    ];

    this.newEntryHooks = [
      document.getElementById("performer-table-body").lastElementChild,
      document.getElementById("personnel-table-body").lastElementChild,
      document.getElementById("pose-source-table-body").lastElementChild,
      document.getElementById("vehicle-table-body").lastElementChild,
    ];

    //Build database tables interface.
    this.dataInterfaces = [
      new PerformerInterface(serverInterface),
      new PersonnelInterface(serverInterface),
      new PoseSourceInterface(serverInterface),
      new VehicleInterface(serverInterface),
    ];

    //Append event listeners:
    //1.Display table listeners.
    for (let i = 0; i < this.titleHooks.length; i++) {
      this.titleHooks[i].addEventListener(
        "click",
        this.titleClickHandler.bind(this, this.dataInterfaces[i])
      );
    }
    //2.Add row listeners.
    for (let i = 0; i < this.newEntryHooks.length; i++) {
      this.newEntryHooks[i].addEventListener(
        "click",
        this.inputNewEntryClickHandler.bind(this, this.dataInterfaces[i])
      );
    }
    //3.Reset add row listeners.
    for (let i = 0; i < this.newEntryHooks.length; i++) {
      this.newEntryHooks[i].addEventListener(
        "focusout",
        this.newEntryDefocusHandler.bind(this)
      );
    }
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
      //Create an icon container.
      const container = DOMGeneric.createMaterialIconsContainer(
        "div",
        "edit",
        "clear"
      );
      //Assign an event listener to the edit icon.
      container.firstElementChild.addEventListener(
        "click",
        this.editEntryClickHandler.bind(this, dataInterface)
      );
      //Assign an event listener to the clear icon.
      container.lastElementChild.addEventListener(
        "click",
        this.clearEntryClickHandler.bind(this, dataInterface)
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
    for (let i = 1; i < dataInterface.paramNames.length; i++) {
      putData.push(dataRow.children[i].value);
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
    for (let i = 1; i < dataInterface.paramNames.length; i++) {
      postData.push(dataRow.children[i].value);
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
    //Change table data cells to input fields.
    for (let i = 1; i < dataInterface.paramNames.length; i++) {
      const newInput = document.createElement("input");
      newInput.type = "text";
      newInput.className = "new-entry-input";
      newInput.value = editRow.children[i].textContent;
      editRow.children[i].replaceWith(newInput);
    }
    //Change row icons to done icon.
    const doneCell = DOMGeneric.createMaterialIconsContainer("button", "done");
    editRow.lastElementChild.replaceWith(doneCell);
    //Append event listener to done icon.
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
        target.appendChild(newInput);
      }
      //Insert done icon in last cell.
      const doneCell = DOMGeneric.createMaterialIconsContainer(
        "button",
        "done"
      );
      target.appendChild(doneCell);
      //Append event listener to done icon.
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
}
