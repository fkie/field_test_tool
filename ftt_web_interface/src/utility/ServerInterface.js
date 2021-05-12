/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//Fetch wrapper for HTTP interfacing with FTT server API.
export class ServerInterface {
  constructor() {
    this.server_adr = location.host;
  }

  async sendHttpRequest(method, url, data) {
    const response = await fetch(url, {
      method: method,
      body: JSON.stringify(data),
      headers: {
        "Content-Type": "application/json",
      },
    });
    if (response.status >= 200 && response.status < 300) {
      return response.json();
    } else {
      const response_data = await response.json();
      console.log(response_data.message);
      throw new Error("Something went wrong!");
    }
  }

  async sendGetRequest(path, params) {
    const responseData = await this.sendHttpRequest(
      "GET",
      `http://${this.server_adr}/${path}${params ? "?" + params : ""}`
    );
    if (responseData) {
      return responseData;
    } else {
      throw new Error("Empty HTTP response!");
    }
  }

  async sendPostRequest(path, data) {
    const responseData = await this.sendHttpRequest(
      "POST",
      `http://${this.server_adr}/${path}`,
      data
    );
    if (responseData) {
      console.log(responseData);
      return responseData;
    } else {
      throw new Error("Invalid HTTP request");
    }
  }

  async sendPutRequest(path, data) {
    const responseData = await this.sendHttpRequest(
      "PUT",
      `http://${this.server_adr}/${path}`,
      data
    );
    if (responseData) {
      console.log(responseData);
    } else {
      throw new Error("Invalid HTTP request");
    }
  }

  async sendDeleteRequest(path, params) {
    const responseData = await this.sendHttpRequest(
      "DELETE",
      `http://${this.server_adr}/${path}${params ? "?" + params : ""}`
    );
    if (responseData) {
      console.log(responseData);
    } else {
      throw new Error("Invalid HTTP request");
    }
  }
}
