// src/overlays/PlaybackConfig.js
export class PlaybackConfig {
  // This class is based on a template that is later added to another DOM node.
  // Therefore, no DOM references are kept as properties.
  constructor() {
    const templateEl = document.getElementById("playback-config-template");
    this.element = document.importNode(templateEl.content, true);

    const speedInput = this.element.getElementById("playback-speed");

    // Load current config or default
    const playbackData = JSON.parse(
      localStorage.getItem("fttPlaybackData")
    );
    if (playbackData && typeof playbackData.speed === "number") {
      speedInput.value = playbackData.speed;
    } else {
      speedInput.value = 10; // default speed-up factor
    }

    speedInput.addEventListener(
      "input",
      this.dataChangedHandler.bind(this)
    );

    // Update button
    this.element
      .querySelector("form button:first-of-type")
      .addEventListener("click", this.updateDataBtnHandler.bind(this));

    // Reset button
    this.element
      .querySelector("form button:last-of-type")
      .addEventListener("click", this.resetDataBtnHandler.bind(this));
  }

  dataChangedHandler() {
    this.element.querySelector("form button").disabled = false;
  }

  updateDataBtnHandler(event) {
    event.preventDefault();
    const value = parseFloat(
      document.getElementById("playback-speed").value
    );

    if (Number.isNaN(value) || value <= 0) {
      alert("Playback speed must be a positive number.");
      return;
    }

    const playbackData = { speed: value };
    localStorage.setItem("fttPlaybackData", JSON.stringify(playbackData));
    this.element.querySelector("form button").disabled = true;
  }

  resetDataBtnHandler(event) {
    event.preventDefault();
    const playbackData = { speed: 10 };
    document.getElementById("playback-speed").value = playbackData.speed;
    localStorage.setItem("fttPlaybackData", JSON.stringify(playbackData));
    this.element.querySelector("form button").disabled = true;
  }
}