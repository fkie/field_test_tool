/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { ServerInterface } from "./utility/ServerInterface.js";
import { ConfigFrame } from "./frames/ConfigFrame.js";

class App {
  static init() {
    this.serverInterface = new ServerInterface();
    this.configFrame = new ConfigFrame(this.serverInterface);
  }
}

App.init();
