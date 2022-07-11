/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Common class for displaying overlays.
export class ConfirmationModal {
  constructor(title, htmlMessage, fallbackText, confirmCallbackFunction) {
    this.fallbackText = fallbackText;
    this.visible = false;
    //Import and build modal node.
    const modalTemplateEl = document.getElementById(
      "confirmation-modal-template"
    );
    const modalElements = document.importNode(modalTemplateEl.content, true);
    //Write message to modal.
    this.modalElement = modalElements.querySelector(".modal");
    this.modalElement.querySelector(".modal__title").firstElementChild.innerHTML = title;
    this.modalElement.querySelector("p").innerHTML = htmlMessage;
    //Select modal element and assign transitionend listener to close it
    this.modalElement.addEventListener("transitionend", this.close.bind(this));
    //Select backdrop and assign click listener to close it.
    this.backdropElement = modalElements.querySelector(".backdrop");
    this.backdropElement.addEventListener("click", this.hide.bind(this));
    //Select confirm button and assign click listener to execute the callback (then hide the modal).
    modalElements
      .getElementById("modal-confirm-btn")
      .addEventListener("click", () => {
        confirmCallbackFunction();
        this.hide();
      });
    //Select close button and assign click listener to close it.
    modalElements
      .getElementById("modal-cancel-btn")
      .addEventListener("click", this.hide.bind(this));
  }

  show() {
    //Check template support
    if ("content" in document.createElement("template")) {
      //Render modal and backdrop
      document.body.insertAdjacentElement("afterbegin", this.modalElement);
      document.body.insertAdjacentElement("afterbegin", this.backdropElement);
      //scroll page to align with the top of the modal
      this.modalElement.scrollIntoView({ behavior: "smooth" });
      //Record visibility status
      this.visible = true;
      //Add show class to make visible
      this.modalElement.classList.add("show");
    } else {
      //Alert fallback text
      alert(this.fallbackText);
    }
  }

  hide() {
    //Record visibility status
    this.visible = false;
    //Remove show class to make it disappear
    this.modalElement.classList.remove("show");
  }

  close() {
    //Workaround to prevent "transitionend" trigger when showing
    if (this.visible) {
      return;
    }
    if (this.modalElement) {
      //Delete modal and backdrop
      document.body.removeChild(this.modalElement);
      document.body.removeChild(this.backdropElement);
      this.modalElement = null;
      this.backdropElement = null;
    }
  }
}
