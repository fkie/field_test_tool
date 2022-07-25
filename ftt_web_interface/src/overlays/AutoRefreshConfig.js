/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2022 - Fraunhofer FKIE
 */

//Structure and logic of the autoRefresh config UI.
export class AutoRefreshConfig {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor() {
    //Import and build autoRefresh config node.
    const templateEl = document.getElementById("auto-refresh-config-template");
    this.element = document.importNode(templateEl.content, true);
    //Get html hooks.
    const autoRefreshTimer = this.element.getElementById("auto-refresh-timer");
    //Fill input fields with current data.
    const autoRefreshData = JSON.parse(
      localStorage.getItem("fttAutoRefreshData")
    );
    if (autoRefreshData) {
      autoRefreshTimer.value = autoRefreshData.timer;
    } else {
      alert("No autoRefresh configuration data in local storage.");
    }
    //Set change event listeners.
    autoRefreshTimer.addEventListener(
      "input",
      this.dataChangedHandler.bind(this)
    );
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
    let autoRefreshData = {};
    autoRefreshData.timer = parseInt(
      document.getElementById("auto-refresh-timer").value
    );
    //Store data.
    localStorage.setItem("fttAutoRefreshData", JSON.stringify(autoRefreshData));
    //Disable update button.
    this.element.querySelector("form button").disabled = true;
  }

  async resetDataBtnHandler(event) {
    event.preventDefault();
    //Create default data object.
    const autoRefreshData = {
      timer: 2000,
    };
    //Fill input fields with current data
    document.getElementById("auto-refresh-timer").value = autoRefreshData.timer;
    //Store data.
    localStorage.setItem("fttAutoRefreshData", JSON.stringify(autoRefreshData));
    //Disable update button.
    this.element.querySelector("form button").disabled = true;
  }
}
