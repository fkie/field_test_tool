// This code is an extract of the ros2d javascript library.
// https://github.com/RobotWebTools/ros2djs

// Software License Agreement (BSD License)

// Copyright (c) 2013, Robert Bosch LLC, Willow Garage Inc., Worcester
// Polytechnic Institute, Yujin Robot. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Robert Bosch LLC, Willow Garage Inc., 
//    Worcester Polytechnic Institute, Yujin Robot nor the names of its 
//    contributors may be used to endorse or promote products derived 
//    from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

var ROS2D = ROS2D || {
  REVISION : '0.9.0'
};

// convert the given global Stage coordinates to ROS coordinates
createjs.Stage.prototype.globalToRos = function(x, y) {
  var rosX = (x - this.x) / this.scaleX;
  var rosY = (this.y - y) / this.scaleY;
  return new ROSLIB.Vector3({
    x : rosX,
    y : rosY
  });
};

// convert the given ROS coordinates to global Stage coordinates
createjs.Stage.prototype.rosToGlobal = function(pos) {
  var x = pos.x * this.scaleX + this.x;
  var y = pos.y * this.scaleY + this.y;
  return {
    x : x,
    y : y
  };
};

// convert a ROS quaternion to theta in degrees
createjs.Stage.prototype.rosQuaternionToGlobalTheta = function(orientation) {
  // See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Rotation_matrices
  // here we use [x y z] = R * [1 0 0]
  var q0 = orientation.w;
  var q1 = orientation.x;
  var q2 = orientation.y;
  var q3 = orientation.z;
  // Canvas rotation is clock wise and in degrees
  return -Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 / Math.PI;
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A Viewer can be used to render an interactive 2D scene to a HTML5 canvas.
 *
 * @constructor
 * @param options - object with following keys:
 *   * divID - the ID of the div to place the viewer in
 *   * width - the initial width, in pixels, of the canvas
 *   * height - the initial height, in pixels, of the canvas
 *   * background (optional) - the color to render the background, like '#efefef'
 */
 ROS2D.Viewer = function(options) {
  var that = this;
  options = options || {};
  var divID = options.divID;
  this.width = options.width;
  this.height = options.height;
  var background = options.background || '#111111';

  // create the canvas to render to
  var canvas = document.createElement('canvas');
  canvas.width = this.width;
  canvas.height = this.height;
  canvas.style.background = background;
  document.getElementById(divID).appendChild(canvas);
  // create the easel to use
  this.scene = new createjs.Stage(canvas);

  // change Y axis center
  this.scene.y = this.height;

  // add the renderer to the page
  document.getElementById(divID).appendChild(canvas);

  // update at 30fps
  createjs.Ticker.setFPS(30);
  createjs.Ticker.addEventListener('tick', this.scene);
};

/**
 * Add the given createjs object to the global scene in the viewer.
 *
 * @param object - the object to add
 */
ROS2D.Viewer.prototype.addObject = function(object) {
  this.scene.addChild(object);
};

/**
 * Scale the scene to fit the given width and height into the current canvas.
 *
 * @param width - the width to scale to in meters
 * @param height - the height to scale to in meters
 */
ROS2D.Viewer.prototype.scaleToDimensions = function(width, height) {
  // restore to values before shifting, if ocurred
  this.scene.x = typeof this.scene.x_prev_shift !== 'undefined' ? this.scene.x_prev_shift : this.scene.x;
  this.scene.y = typeof this.scene.y_prev_shift !== 'undefined' ? this.scene.y_prev_shift : this.scene.y;
  
  // save scene scaling
  this.scene.scaleX = this.width / width;
  this.scene.scaleY = this.height / height;
};

/**
 * Shift the main view of the canvas by the given amount. This is based on the
 * ROS coordinate system. That is, Y is opposite that of a traditional canvas.
 *
 * @param x - the amount to shift by in the x direction in meters
 * @param y - the amount to shift by in the y direction in meters
 */
ROS2D.Viewer.prototype.shift = function(x, y) {
  // save current offset
  this.scene.x_prev_shift = this.scene.x;
  this.scene.y_prev_shift = this.scene.y;

  // shift scene by scaling the desired offset
  this.scene.x -= (x * this.scene.scaleX);
  this.scene.y += (y * this.scene.scaleY);
};

/**
 * @author Bart van Vliet - bart@dobots.nl
 */

/**
 * Adds zooming to a view
 *
 * @constructor
 * @param options - object with following keys:
 *   * rootObject (optional) - the root object to apply zoom to
 *   * minScale (optional) - minimum scale to set to preserve precision
 */
 ROS2D.ZoomView = function(options) {
	options = options || {};
	this.rootObject = options.rootObject;
	this.minScale = options.minScale || 0.001;
	
	// get a handle to the stage
	if (this.rootObject instanceof createjs.Stage) {
		this.stage = this.rootObject;
	}
	else {
		this.stage = this.rootObject.getStage();
	}
	
	this.center = new ROSLIB.Vector3();
	this.startShift = new ROSLIB.Vector3();
	this.startScale = new ROSLIB.Vector3();
};


ROS2D.ZoomView.prototype.startZoom = function(centerX, centerY) {
	this.center.x = centerX;
	this.center.y = centerY;
	this.startShift.x = this.stage.x;
	this.startShift.y = this.stage.y;
	this.startScale.x = this.stage.scaleX;
	this.startScale.y = this.stage.scaleY;
};

ROS2D.ZoomView.prototype.zoom = function(zoom) {
	// Make sure scale doesn't become too small
	if (this.startScale.x*zoom < this.minScale) {
		zoom = this.minScale/this.startScale.x;
	}
	if (this.startScale.y*zoom < this.minScale) {
		zoom = this.minScale/this.startScale.y;
	}
	
	this.stage.scaleX = this.startScale.x*zoom;
	this.stage.scaleY = this.startScale.y*zoom;
	
	this.stage.x = this.startShift.x - (this.center.x-this.startShift.x) * (this.stage.scaleX/this.startScale.x - 1);
	this.stage.y = this.startShift.y - (this.center.y-this.startShift.y) * (this.stage.scaleY/this.startScale.y - 1);
};

/**
 * @author Bart van Vliet - bart@dobots.nl
 */

/**
 * Adds panning to a view
 *
 * @constructor
 * @param options - object with following keys:
 *   * rootObject (optional) - the root object to apply panning to
 */
 ROS2D.PanView = function(options) {
	options = options || {};
	this.rootObject = options.rootObject;
	
	// get a handle to the stage
	if (this.rootObject instanceof createjs.Stage) {
		this.stage = this.rootObject;
	}
	else {
		this.stage = this.rootObject.getStage();
	}
	
	this.startPos = new ROSLIB.Vector3();
};


ROS2D.PanView.prototype.startPan = function(startX, startY) {
	this.startPos.x = startX;
	this.startPos.y = startY;
};

ROS2D.PanView.prototype.pan = function(curX, curY) {
	this.stage.x += curX - this.startPos.x;
	this.startPos.x = curX;
	this.stage.y += curY - this.startPos.y;
	this.startPos.y = curY;
};