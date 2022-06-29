/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2022 - Fraunhofer FKIE
 */

//Structure and logic of the map config UI.
export class MapConfig {
  //This class is based on a template that it later going to be added to another DOM Node.
  //Therefore, no DOM references are kept as properties.
  constructor() {
    //Import and build map config node.
    const templateEl = document.getElementById("map-config-template");
    this.element = document.importNode(templateEl.content, true);
    //Get html hooks.
    const mapUrl = this.element.getElementById("map-url");
    const mapMinZoom = this.element.getElementById("map-min-zoom");
    const mapMaxZoom = this.element.getElementById("map-max-zoom");
    //Fill input fields with current data.
    const mapData = JSON.parse(localStorage.getItem("fttTileServerData"));
    if (mapData) {
      mapUrl.value = mapData.url;
      mapMinZoom.value = mapData.minZoom;
      mapMaxZoom.value = mapData.maxZoom;
    } else {
      alert("No map configuration data in local storage.");
    }
    //Set change event listeners.
    mapUrl.addEventListener("input", this.dataChangedHandler.bind(this));
    mapMinZoom.addEventListener("input", this.dataChangedHandler.bind(this));
    mapMaxZoom.addEventListener("input", this.dataChangedHandler.bind(this));
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
    let mapData = {};
    mapData.url = document.getElementById("map-url").value;
    mapData.minZoom = document.getElementById("map-min-zoom").value;
    mapData.maxZoom = document.getElementById("map-max-zoom").value;
    //Store data.
    localStorage.setItem("fttTileServerData", JSON.stringify(mapData));
    //Disable update button.
    this.element.querySelector("form button").disabled = true;
  }

  async resetDataBtnHandler(event) {
    event.preventDefault();
    //Create default data object.
    const mapData = {
      url: "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
      minZoom: 0,
      maxZoom: 19,
    };
    //Fill input fields with current data
    document.getElementById("map-url").value = mapData.url;
    document.getElementById("map-min-zoom").value = mapData.minZoom;
    document.getElementById("map-max-zoom").value = mapData.maxZoom;
    //Store data.
    localStorage.setItem("fttTileServerData", JSON.stringify(mapData));
    //Disable update button.
    this.element.querySelector("form button").disabled = true;
  }
}
