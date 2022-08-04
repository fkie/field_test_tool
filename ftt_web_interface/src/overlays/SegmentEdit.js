/* eslint-disable no-control-regex */
/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { DOMGeneric } from "../utility/DOMGeneric.js";

//Structure and logic of the segment edition UI.
export class SegmentEdit {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor(
    segment,
    segmentInterface,
    noteInterface,
    imageInterface,
    itoReasonList,
    currentUser
  ) {
    //Set arguments as properties.
    this.segment = segment;
    this.segmentInterface = segmentInterface;
    this.noteInterface = noteInterface;
    this.imageInterface = imageInterface;
    this.itoReasonList = itoReasonList;
    this.currentUser = currentUser;
    //Import and build segment edition node.
    const templateEl = document.getElementById("edit-segment-template");
    this.element = document.importNode(templateEl.content, true);

    //=============================== HEADER ===============================
    //Set title, start and end time.
    this.element.querySelector("header h2 span:last-of-type").textContent =
      segment.id;
    this.element.querySelector(
      "header p:first-of-type span:last-of-type"
    ).textContent = new Date(segment.startTime * 1000).toISOString();
    if (segment.endTime) {
      this.element.querySelector(
        "header p:last-of-type span:last-of-type"
      ).textContent = new Date(segment.endTime * 1000).toISOString();
    }

    //======================== SEGMENT DATA SECTION ========================
    //Reach to segment info data elements.
    const itoReasonsSelect = this.element.getElementById("ito-reason");
    const obstacleInput = this.element.getElementById("obstacle");
    const lightingInput = this.element.getElementById("lighting");
    const slopeInput = this.element.getElementById("slope");
    //Populate ito reason select.
    DOMGeneric.populateSelect(
      itoReasonsSelect,
      itoReasonList.map((entry) => entry.shortDescription)
    );
    //Focus matching option in the select with the current segment ito reason.
    const matchedItoReason = itoReasonList.find(
      (entry) => entry.id == segment.itoReasonId
    );
    if (matchedItoReason) {
      itoReasonsSelect.value = matchedItoReason.shortDescription;
      DOMGeneric.removeFirstEmptyOption(itoReasonsSelect);
    }
    //Fill input fields with segment data.
    obstacleInput.value = segment.obstacle;
    lightingInput.value = segment.lighting;
    slopeInput.value = segment.slope;
    //Block obstacle context input fields for non ITO segments.
    if (segment.segmentType != "ITO") {
      itoReasonsSelect.disabled = true;
      obstacleInput.disabled = true;
      lightingInput.disabled = true;
      slopeInput.disabled = true;
    }
    //Set change event listeners.
    itoReasonsSelect.addEventListener(
      "change",
      this.segmentDataChangedHandler.bind(this)
    );
    obstacleInput.addEventListener(
      "input",
      this.segmentDataChangedHandler.bind(this)
    );
    lightingInput.addEventListener(
      "input",
      this.segmentDataChangedHandler.bind(this)
    );
    slopeInput.addEventListener(
      "input",
      this.segmentDataChangedHandler.bind(this)
    );
    //Register update button handler.
    this.element
      .querySelector("form button")
      .addEventListener("click", this.updateDataBtnHandler.bind(this));

    //=========================== NOTES SECTION ============================
    //Initialize variables.
    this.noteList = null;
    this.selectedNoteIdx = 0;
    //Assign listener to changes in notes.
    this.element
      .getElementById("note-text")
      .addEventListener("keydown", this.noteTextChangeHandler.bind(this));
    //Assign listener to save button.
    this.element
      .getElementById("save-note-btn")
      .addEventListener("click", this.saveNoteButtonHandler.bind(this));
    //Assign listener to remove button.
    this.element
      .getElementById("remove-note-btn")
      .addEventListener("click", this.removeNoteButtonHandler.bind(this));

    //=========================== IMAGES SECTION ===========================
    //Reach to image elements.
    const imagesContainer = this.element.querySelector(".current-images");
    //Initialize variables to hold images.
    this.imageList = null;
    this.newImage = null;
    this.newImageName = null;
    //Assign listeners to current images and upload image tabs.
    this.element
      .getElementById("current-images-tab")
      .addEventListener("click", this.currentImagesTabClickHandler.bind(this));
    this.element
      .getElementById("upload-image-tab")
      .addEventListener("click", this.uploadImageTabClickHandler.bind(this));
    //Assign listener to the current images div.
    imagesContainer.addEventListener(
      "focusin",
      this.imageFocusHandler.bind(this)
    );
    imagesContainer.addEventListener(
      "focusout",
      this.imageDefocusHandler.bind(this)
    );
    imagesContainer.addEventListener(
      "dblclick",
      this.imageDoubleClickHandler.bind(this)
    );
    //Assign listener to the upload image input field.
    this.element
      .getElementById("new-image")
      .addEventListener("change", this.newImageInputHandler.bind(this));
    //Assign listener to the submit image button.
    this.element
      .getElementById("submit-image-btn")
      .addEventListener("click", this.submitImageBtnHandler.bind(this));

    //================================ INIT ================================
    //Call init function to execute async behavior.
    this.init();
  }

  async init() {
    //Get and render notes.
    await this.renderNotes();
    //Get and render images.
    await this.renderImages();
  }

  segmentDataChangedHandler(event) {
    //Prevent selection for AUTO segments.
    if (this.segment.segmentType == "AUTO") {
      event.target.value = "";
      alert("Obstacle context data available only for ITO segments!");
      return;
    }
    //Check validity for all text inputs.
    const invalid =
      this.element.querySelector("#obstacle").validity.patternMismatch ||
      this.element.querySelector("#lighting").validity.patternMismatch ||
      this.element.querySelector("#slope").validity.patternMismatch;
    if (invalid) {
      //Disable update button.
      this.element.querySelector("form button").disabled = true;
      //Show info text.
      this.element.querySelector(".info-text").style.display = "inline";
    } else {
      //Enable update button.
      this.element.querySelector("form button").disabled = false;
      //Hide info text.
      this.element.querySelector(".info-text").style.display = "";
    }
  }

  async updateDataBtnHandler(event) {
    event.preventDefault();
    //Get parameters to update.
    const itoReasonId = this.itoReasonList.find(
      (entry) =>
        entry.shortDescription == document.getElementById("ito-reason").value
    ).id;
    const obstacle = document
      .getElementById("obstacle")
      .value.replace(/[^\x00-\x7F]/g, "");
    const lighting = document
      .getElementById("lighting")
      .value.replace(/[^\x00-\x7F]/g, "");
    const slope = document
      .getElementById("slope")
      .value.replace(/[^\x00-\x7F]/g, "");
    try {
      //Send update request to server.
      await this.segmentInterface.put(
        this.segment.id,
        "edit",
        itoReasonId,
        obstacle,
        lighting,
        slope
      );
      //Disable update button.
      this.element.querySelector("form button").disabled = true;
    } catch (error) {
      alert(error.message);
    }
  }

  async renderNotes() {
    try {
      //Get data from server.
      this.noteList = await this.noteInterface.get(this.segment.id);
      //Clear any previously created tabs.
      DOMGeneric.clearChildren(this.element.querySelector(".btn-tabs"));
      //Create a new tab for each note.
      this.noteList.forEach((entry) => {
        this.createNewNoteTab(
          new Date(entry.secs * 1000).toString().substring(4, 10)
        );
      });
      //Create tab for a potential new note.
      this.createNewNoteTab("New Note", "");
      //Display the selected note (the first one by default).
      this.element
        .querySelector(".btn-tabs")
        .children[this.selectedNoteIdx].click();
    } catch (error) {
      alert(error.message);
    }
  }

  createNewNoteTab(tabName) {
    //Remove the new-note class from all existing tabs.
    const tabs = this.element.querySelector(".btn-tabs");
    Array.from(tabs.children).forEach((tab) => {
      tab.classList.remove("new-note");
    });
    //Create a new tab, assign it a name, and the new-note class
    const newTab = document.createElement("button");
    newTab.textContent = tabName;
    newTab.classList.add("new-note");
    //Attach the new tab to the html page
    tabs.appendChild(newTab);
    //Add event listener for the click on the tab
    newTab.addEventListener("click", this.noteTabClickHandler.bind(this));
  }

  noteTabClickHandler(event) {
    //Remove the selected-tab class and any changes from all tabs.
    const tabs = Array.from(this.element.querySelector(".btn-tabs").children);
    tabs.forEach((tab) => {
      tab.classList.remove("selected-tab");
      if (tab.textContent.slice(-1) == "*") {
        tab.textContent = tab.textContent.slice(0, -1);
      }
      //Disable save button.
      this.element.querySelector("#save-note-btn").disabled = true;
    });
    //Add selected class to clicked tab.
    event.target.classList.add("selected-tab");
    //Find and store index of selected tab.
    this.selectedNoteIdx = tabs.indexOf(event.target);
    //Get text area hook.
    const textArea = this.element.querySelector("#note-text");
    if (this.selectedNoteIdx < this.noteList.length) {
      //For existing notes:
      //Write note in text area.
      textArea.value = this.noteList[this.selectedNoteIdx].note;
      //Enable the remove button.
      this.element.querySelector("#remove-note-btn").disabled = false;
    } else {
      //Clear text area.
      textArea.value = "";
      //Disable remove button for a new note.
      this.element.querySelector("#remove-note-btn").disabled = true;
    }
  }

  noteTextChangeHandler() {
    //If not already included, add * to tab name to reflect unsaved changes.
    let selectedTab = this.element.querySelector(".selected-tab");
    if (selectedTab.textContent.slice(-1) != "*") {
      selectedTab.textContent += "*";
    }
    //Enable save button.
    document.getElementById("save-note-btn").disabled = false;
  }

  async saveNoteButtonHandler(event) {
    event.preventDefault();
    //Check if the current user is set.
    if (!this.currentUser.id) {
      alert("Please select a user to add or modify a note!");
      return;
    }
    //Reach to the text content.
    const noteText = document.getElementById("note-text").value.replace(/[^\x00-\x7F]/g, "");
    try {
      //Check if the note is being modified or created.
      if (this.selectedNoteIdx < this.noteList.length) {
        //Modified: update note.
        await this.noteInterface.put(
          this.noteList[this.selectedNoteIdx].id,
          this.currentUser.id,
          noteText
        );
      } else {
        //Created: post note.
        await this.noteInterface.post(
          this.segment.id,
          this.currentUser.id,
          noteText
        );
      }
      //Disable save button
      event.target.disabled = true;
      //Update UI.
      await this.renderNotes();
    } catch (error) {
      alert(error.message);
    }
  }

  async removeNoteButtonHandler() {
    try {
      //Delete selected note.
      await this.noteInterface.delete(this.noteList[this.selectedNoteIdx].id);
      //Update UI.
      await this.renderNotes();
    } catch (error) {
      alert(error.message);
    }
  }

  async renderImages() {
    //Get images from server.
    try {
      this.imageList = await this.imageInterface.get(
        this.segment.parentId || this.segment.id
      );
      //Select the container area and clear it.
      const imagesContainer = this.element.querySelector(".current-images");
      DOMGeneric.clearChildren(imagesContainer);
      this.imageList.forEach((image) => {
        //Create a focusable div for each image.
        const imgDiv = document.createElement("div");
        imgDiv.tabIndex = 0;
        //Append the image to the div.
        DOMGeneric.addImage(imgDiv, image.imageData, image.imageFilename);
        //Append the div to the image area.
        imagesContainer.appendChild(imgDiv);
      });
    } catch (error) {
      alert(error.message);
    }
  }

  async currentImagesTabClickHandler(event) {
    //Return if the tab is already selected.
    if (event.target.classList.contains("selected-tab")) {
      return;
    }
    //Swap the selected tab.
    event.target.nextElementSibling.classList.remove("selected-tab");
    event.target.classList.add("selected-tab");
    //Swap display of div content.
    this.element.querySelector(".upload-image").style.display = "none";
    this.element.querySelector(".current-images").style.display = "";
    //get and render images
    this.renderImages();
  }

  uploadImageTabClickHandler(event) {
    //Return if the tab is already selected.
    if (event.target.classList.contains("selected-tab")) {
      return;
    }
    //Check if the current user is set.
    if (!this.currentUser.id) {
      alert("Please select a user to add an image!");
      return;
    }
    //Swap the selected tab.
    event.target.previousElementSibling.classList.remove("selected-tab");
    event.target.classList.add("selected-tab");
    //Swap display of div content.
    this.element.querySelector(".current-images").style.display = "none";
    this.element.querySelector(".upload-image").style.display = "block";
  }

  async removeImageBtnClickHandler(event) {
    //Find the selected image index.
    const selectedImageIdx = Array.from(
      this.element.querySelector(".current-images").children
    ).indexOf(event.target.closest("div"));
    //Send the delete request.
    try {
      await this.imageInterface.delete(this.imageList[selectedImageIdx].id);
      //Update images display.
      this.renderImages();
    } catch (error) {
      alert(error.message);
    }
  }

  imageFocusHandler(event) {
    //Create a remove bubble.
    const removeImgBubble = DOMGeneric.createMaterialIconsContainer(
      "span",
      "remove-icon-container",
      ["remove_circle_outline"]
    );
    //Add an event listener to it.
    removeImgBubble.addEventListener(
      "click",
      this.removeImageBtnClickHandler.bind(this)
    );
    //Append it to the image div container (event target).
    event.target.appendChild(removeImgBubble);
  }

  imageDefocusHandler(event) {
    //Delete the remove bubble (last child) from the image div container (event target).
    event.target.lastElementChild.remove();
  }

  imageDoubleClickHandler(event) {
    //Check if double click was on an image.
    if (event.target.tagName.toLowerCase() === "img") {
      //Open new tab with the image.
      const newTabWindow = open("");
      newTabWindow.document.write(event.target.outerHTML);
      newTabWindow.document.head.appendChild(
        document.head.querySelector("meta")
      );
      newTabWindow.document.head.appendChild(document.createElement("title"));
      newTabWindow.document.head.querySelector("title").textContent =
        event.target.alt;
    }
  }

  newImageInputHandler(event) {
    //Get image data
    const imageData = event.target.files[0];
    this.newImageName = imageData.name.replace(/[.][^.]+$/, "");
    if (imageData) {
      //Parse data as base64 with a FileReader instance.
      const fileReader = new FileReader();
      fileReader.readAsDataURL(imageData);
      //Assign the parsed data to the newImage class property.
      fileReader.addEventListener("load", (e) => {
        this.newImage = e.target.result;
        this.newImage = this.newImage.substring(this.newImage.indexOf(",") + 1);
      });
      document.getElementById("submit-image-btn").disabled = false;
    }
  }

  async submitImageBtnHandler(event) {
    event.preventDefault();
    //If a new image is available, send it to the server.
    if (this.newImage) {
      try {
        await this.imageInterface.post(
          this.segment.parentId || this.segment.id,
          this.newImageName,
          this.newImage,
          `Image uploaded by ${this.currentUser.name}`
        );
        //Reset the newImage property and disable the button.
        this.newImage = null;
        event.target.disabled = true;
        //Change to the current images tab.
        document.getElementById("current-images-tab").click();
      } catch (error) {
        alert(error.message);
      }
    }
  }
}
