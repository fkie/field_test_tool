/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

import { ServerInterface } from "./utility/ServerInterface.js";
import { MainFrame } from "./frames/MainFrame.js";

class App {
  static init() {
    this.serverInterface = new ServerInterface();
    this.mainFrame = new MainFrame(this.serverInterface);
  }
}

App.init();
