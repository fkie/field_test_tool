/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";

//Structure and logic of the user select UI.
export class UserSelect {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor(currentUser, personnelList) {
    //Set arguments as properties.
    this.currentUser = currentUser;
    this.personnelList = personnelList;
    //Import and build leg edition node.
    const templateEl = document.getElementById("select-user-template");
    this.element = document.importNode(templateEl.content, true);

    //============================ DATA SECTION ============================
    //Reach to user select.
    const userSelect = this.element.getElementById("user");
    //Populate select.
    DOMGeneric.populateSelect(
      userSelect,
      personnelList.map(
        (entry) => `${entry.id}.- ${entry.name} - ${entry.institution}`
      )
    );
    //Focus matching option on the select with the current user.
    const matchedUserIdx = personnelList.findIndex(
      (entry) => entry.id == currentUser.id
    );
    if (matchedUserIdx > -1) {
      userSelect.value = userSelect.children[matchedUserIdx].value;
      DOMGeneric.removeFirstEmptyOption(userSelect);
    }
    //Set change event listeners.
    userSelect.addEventListener("change", this.userChangedHandler.bind(this));
  }

  userChangedHandler() {
    //Get select value.
    const selectedUser = document.getElementById("user").value;
    //Find the entry in the personnel list and assign their values to the current user object.
    Object.assign(
      this.currentUser,
      this.personnelList.find((entry) => selectedUser.includes(entry.name))
    );
  }
}
