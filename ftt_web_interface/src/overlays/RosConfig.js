/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2022 - Fraunhofer FKIE
 */

//Structure and logic of the ros config UI.
export class RosConfig {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor() {
    //Import and build ros config node.
    const templateEl = document.getElementById("ros-config-template");
    this.element = document.importNode(templateEl.content, true);
    //Get html hooks.
    const rosUrl = this.element.getElementById("ros-url");
    //Fill input fields with current data.
    const rosData = JSON.parse(localStorage.getItem("fttRosData"));
    if (rosData) {
      rosUrl.value = rosData.url;
    } else {
      //No config found. Use default
      rosUrl.value = `ws://${location.hostname}:9090`;
    }
    //Set change event listeners.
    rosUrl.addEventListener("input", this.dataChangedHandler.bind(this));
    //Register update button handler.
    this.element
      .querySelector("form button:first-of-type")
      .addEventListener("click", this.updateDataBtnHandler.bind(this));
    //Register reset button handler.
    this.element
      .querySelector("form button:last-of-type")
      .addEventListener("click", this.resetDataBtnHandler.bind(this));
  }

  dataChangedHandler() {
    //Activate update button.
    this.element.querySelector("form button").disabled = false;
  }

  async updateDataBtnHandler(event) {
    event.preventDefault();
    //Get data to update.
    let rosData = {};
    rosData.url = document.getElementById("ros-url").value;
    //Store data.
    localStorage.setItem("fttRosData", JSON.stringify(rosData));
    //Disable update button.
    this.element.querySelector("form button").disabled = true;
  }

  async resetDataBtnHandler(event) {
    event.preventDefault();
    //Fill input fields with default data
    document.getElementById("ros-url").value = `ws://${location.hostname}:9090`;
    //Remove data from storage.
    localStorage.removeItem("fttRosData");
    //Disable update button.
    this.element.querySelector("form button").disabled = true;
  }
}
