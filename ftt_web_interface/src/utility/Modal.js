/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Common class for displaying overlays.
export class Modal {
  constructor(
    contentObject,
    fallbackText,
    closeCallbackFunction,
    noMaxWidth = false
  ) {
    this.fallbackText = fallbackText;
    this.closeCallbackFunction = closeCallbackFunction;
    this.visible = false;
    //Import and build modal node.
    const modalTemplateEl = document.getElementById("modal-template");
    const modalElements = document.importNode(modalTemplateEl.content, true);
    //Fill modal with the elements of contentObject.
    this.modalElement = modalElements.querySelector(".modal");
    if (noMaxWidth) {
      this.modalElement.classList.add("no-max-width");
    }
    this.modalElement.insertBefore(
      contentObject.element,
      this.modalElement.children[0]
    );
    //Reassign element property of contentObject, since insertBefore erases it.
    //This is done to keep the content logic inside the content class
    contentObject.element = this.modalElement;
    //Select modal element and assign transitionend listener to close it
    this.modalElement.addEventListener("transitionend", this.close.bind(this));
    //Select backdrop and assign click listener to trigger closing it.
    this.backdropElement = modalElements.querySelector(".backdrop");
    this.backdropElement.addEventListener("click", this.hide.bind(this));
    //Select close button and assign click listener to trigger closing it.
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
    if (this.closeCallbackFunction) {
      this.closeCallbackFunction();
    }
  }
}
