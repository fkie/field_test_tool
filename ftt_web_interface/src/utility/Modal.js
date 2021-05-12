/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Common class for displaying overlays.
export class Modal {
  constructor(contentObject, fallbackText, closeCallbackFunction) {
    this.fallbackText = fallbackText;
    this.closeCallbackFunction = closeCallbackFunction;
    //Import and build modal node.
    const modalTemplateEl = document.getElementById("modal-template");
    const modalElements = document.importNode(modalTemplateEl.content, true);
    //Fill modal with the elements of contentObject.
    this.modalElement = modalElements.querySelector(".modal");
    this.modalElement.insertBefore(
      contentObject.element,
      this.modalElement.children[0]
    );
    //Reassign element property of contentObject, since insertBefore erases it.
    //This is done to keep the content logic inside the content class
    contentObject.element = this.modalElement;
    //Select backdrop and assign click listener to close it.
    this.backdropElement = modalElements.querySelector(".backdrop");
    this.backdropElement.addEventListener("click", this.hide.bind(this));
    //Select close button and assign click listener to close it.
    this.buttonElement = modalElements.getElementById("modal-close-btn");
    this.buttonElement.addEventListener("click", this.hide.bind(this));
  }

  show() {
    //Check template support
    if ("content" in document.createElement("template")) {
      //Render modal and backdrop
      document.body.insertAdjacentElement("afterbegin", this.modalElement);
      document.body.insertAdjacentElement("afterbegin", this.backdropElement);
      //scroll page to align with the top of the modal
      this.modalElement.scrollIntoView({ behavior: "smooth" });
    } else {
      //Alert fallback text
      alert(this.fallbackText);
    }
  }

  hide() {
    if (this.modalElement) {
      //Delete modal and backdrop
      document.body.removeChild(this.modalElement);
      document.body.removeChild(this.backdropElement);
      this.modalElement = null;
      this.backdropElement = null;
    }
    if (this.closeCallbackFunction) {
      this.closeCallbackFunction();
    }
  }
}
