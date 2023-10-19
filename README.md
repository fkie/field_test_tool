# Field Test Tool (FTT)

The purpose of this tool is to monitor and analyze the switching events between autonomous mode and manual mode on autonomous ground vehicles. This is done by collecting relevant data from the robot's ROS environment, alongside context data provided by users.
The collected data is processed by an automatic report generator, which aims to help the manufacturer, the project manager and the customer to analyze software issues in certain enviroments.

If you use this code in an academic context, please cite the following work:
```
@inproceedings{9784813,
  author={Tampier, Carlos and Tiderko, Alexander and Schneider, Frank E.},
  booktitle={2022 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)}, 
  title={Field Test Tool: automatic reporting and reliability evaluation for autonomous ground vehicles}, 
  year={2022},
  pages={73-78},
  doi={10.1109/ICARSC55462.2022.9784813}}

```

## _1. Description_

The Field Test Tools comprises five software modules:
- A PostgreSQL database with PostGIS extension. 
- An automatic report generator out of the stored data.
- A web server that exposes a JSON API for the database and report generator.
- A ROS interface to the server API and the web user interface.
- A web user interface to the server API and the ROS interface.
<br/><br/>

### **1.1. Database**

A PostgreSQL database with PostGIS extension is used to store the relevant data. The databse schema is shown in Figure 1. The robot's position, operating mode, images and user notes are stored under a hierarchical logging structure representing the trial instance (Test Event), test attempt (Shift), and route section (Leg). Each trajectory under a single operating mode comprises a Segment entry.

<figure>
  <img src="doc/images/database_schema.png" alt="database schema" width="500">
  <figcaption>Figure 1. Database schema.</figcaption>
</figure>
<br/><br/>

In addition to the data tables (Test Event, Shift, Leg, Segment, Map Image, Pose, Local Pose, Image and Note), which are used to store user- or robot-generated field data, a number of configuration tables complete the database schema. This configuration tables are divided in static (Weather, ITO Reason, Segment Type) and user-defined (Personnel, Performer, Vehicle, Pose Source). While the latter are designed to contain use-case specific information, the former are completely defined upon database initialization and contain a standardized set of options, from which the user can select when configuring the experimental setup. These options are shown below.
<br/><br/>

Table 1: Weather configuration table data.
| Key      |	Description                                                               |
| -------- | -------------------------------------------------------------------------- |
| sunny    |	Mainly sunny, less than 30 percent of the sky is covered by clouds.       |
| cloudy   |	Mainly cloudy, between 30 and 95 percent of the sky is covered by clouds. |
| overcast |	More than 95 percent of the sky is covered by clouds.                     |
| foggy    |	Reduced visibility due to fog to less than one kilometer.                 |
| rainy    |	-                                                                         |
| snowy    |	-                                                                         |

<br/><br/>

Table 2: ITO Reason configuration table data.
| Key	           | Description                                                                                           |
| -------------- | ----------------------------------------------------------------------------------------------------- |
| safety         |	Safety stop, initiated by an inhabitant of the vehicle.                                              |
| maintenance    |	Maintenance stop, a stop initiated by the performer to fix code.                                     |
| planner        |	Planner stop, a stop caused by a timeout or error of the planner.                                    |
| apparatus      |	Apparatus stop, a stop that is initiated by the test admin to fix the test apparatus.                |
| non_stop       |	Non-Stop, indicates that no stop was performed. Ignore this ITO.                                     |
| stereo         |	Stereo test stop, a stop initiated to perform a test of the stereo accuracy.                         |
| goal           |	Goal reached, a stop because the vehicle reached an (intermediate) goal.                             |
| end_shift      |	End of shift indicates the stop was caused because the time for the shift was up.                    |
| manual_transit |	Manual transit indicates that the vehicle was stopped to drive it manually through a dangerous area. |
| other          |	Other reasons.                                                                                       |
| unassigned     |	Unassigned.                                                                                          |
| unexpected     |	Unexpected, non safety-critical behavior.                                                            |

<br/><br/>

Table 3: Segment Type configuration table data.
| Key	 | Description                              |
| ---- | ---------------------------------------- |
| ito	 | ITO - inhabited take over (manual mode). |
| auto | Autonomous.                              |

<br/><br/>

### **1.2. Report generator**
A python script that reads and processes the stored data, then generates and compiles a LaTeX report. The report includes an overview of the robot's path and a timeline of the operating modes, as shown in Figures 2 and 3. Details for each individual segment (path section) are also annotated.

<figure>
  <img src="doc/images/ftt_report_overview_map.png" alt="FTT report map overview" width="500">
  <figcaption>Figure 2. FTT report overview (map).</figcaption>
</figure>
<br/><br/>

<figure>
  <img src="doc/images/ftt_report_overview_timeline.png" alt="FTT report timeline overview" width="500">
  <figcaption>Figure 3. FTT report overview (timeline).</figcaption>
</figure>
<br/><br/>

### **1.3. Web server**

A python script implements a JSON API with the common requests (POST, PUT, GET) to interact with the database tables through HTTP messages. It also serves the main HTML pages and provides an interface to call and get the automatically generated report.
<br/><br/>

### **1.4. ROS interface**

A ROS node that fetches the robot data to create database entries. It can be run either as a Python or a C++ node. The following data are subscribed:

| Data                          | Type                                                 |
| ----------------------------- | ---------------------------------------------------- |
| Operation mode                | industrial_msgs/RobotMode                            |
| GPS position                  | sensor_msgs/NavSatFix                                |
| Local position                | TF (map -> robot frame) or geometry_msgs/PoseStamped |
| Environment map               | nav_msgs/OccupancyGrid                               |
| Map image<sup>\*</sup>        | sensor_msgs/CompressedImage                          |
| Camera image                  | sensor_msgs/Image                                    |
| Compressed image<sup>\*</sup> | sensor_msgs/CompressedImage                          |

<span style="font-size:smaller">\* Options added only to the Python implementation to improve efficiency.</span>
<br/><br/>

Additionally, the node advertises the following services to interact with the web user interface:

| Service          | Description                                          |
| ---------------- | ---------------------------------------------------- |
| /set_ftt_logging | Starts/stops sending robot data to the database      |
| /get_ftt_logging | Returns the current logging status (active/inactive) |
| /save_ftt_params | Write the node parameters to the config file         |

<br/><br/>

### **1.5. Web user interface**

A JavaScript web application that implements a graphical user interface to create new logging instances and append valuable context information. Feedback on the stored robot position data and operating mode is also displayed.
Figures 4 and 5 show an overview of the FTT web GUI displaying GPS and local position, respectively. The data was generated using Clearpath's Husky ROS stack.

<figure>
  <img src="doc/images/ftt_web_overview_gps.png" alt="FTT GUI GPS" width="500">
  <figcaption>Figure 4. FTT web GUI overview with GPS data.</figcaption>
</figure>
<br/><br/>

<figure>
  <img src="doc/images/ftt_web_overview_local.png" alt="FTT GUI local" width="500">
  <figcaption>Figure 5. FTT web GUI overview with local data.</figcaption>
</figure>
<br/><br/>

## _2. Acknowledgements_

The following libraries and resources are needed for this project. They are shown alongside their respective license.

| Name                       | License                                 | URL                                                                |
| -------------------------- | --------------------------------------- | ------------------------------------------------------------------ |
| rospy                      | BSD License                             | https://wiki.ros.org/rospy                                         |
| roscpp                     | BSD License                             | https://wiki.ros.org/roscpp                                        |
| common_msgs                | BSD License                             | https://wiki.ros.org/common_msgs                                   |
| industrial_msgs            | BSD License                             | https://wiki.ros.org/industrial_msgs                               |
| cv_bridge                  | BSD License                             | https://wiki.ros.org/cv_bridge                                     |
| tf2_ros                    | BSD License                             | https://wiki.ros.org/tf2_ros                                       |
| Python Standard Library    | PSF License                             | https://docs.python.org/2/license.html                             |
| requests                   | Apache License Version 2.0              | https://github.com/psf/requests/blob/master/LICENSE                |
| ruamel.yaml                | MIT License                             | https://sourceforge.net/p/ruamel-yaml/code/ci/default/tree/LICENSE |
| Pillow                     | HPND License                            | https://github.com/python-pillow/Pillow/blob/master/LICENSE        |
| psycopg2                   | GNU Lesser General Public License       | https://www.psycopg.org/license/                                   |
| Flask                      | BSD-3-Clause License                    | https://github.com/pallets/flask/blob/master/LICENSE.rst           |
| Flask-restful              | BSD-3-Clause License                    | https://github.com/flask-restful/flask-restful/blob/master/LICENSE |
| Flask-cors                 | MIT License                             | https://github.com/corydolphin/flask-cors/blob/master/LICENSE      |
| LXML                       | BSD-3-Clause License                    | https://github.com/lxml/lxml/blob/master/LICENSE.txt               |
| jinja2                     | BSD-3-Clause License                    | https://github.com/pallets/jinja/blob/master/LICENSE.rst           |
| parse                      | MIT License                             | https://github.com/r1chardj0n3s/parse/blob/master/LICENSE          |
| pyproj                     | MIT License                             | https://github.com/pyproj4/pyproj/blob/master/LICENSE              |
| Matplotlib                 | PSF-based License                       | https://github.com/matplotlib/matplotlib/blob/main/LICENSE/LICENSE |
| NumPy                      | BSD License                             | https://numpy.org/doc/stable/license.html                          |
| GNU C++ Standard Library   | GPLv3 License                           | https://gcc.gnu.org/onlinedocs/libstdc++/manual/license.html       |
| OpenCV                     | BSD-3-Clause or Apache 2.0 License      | https://opencv.org/license                                         |
| cpr                        | MIT License                             | https://github.com/libcpr/cpr/blob/master/LICENSE                  |
| nlohmann json              | MIT License                             | https://github.com/nlohmann/json/blob/develop/LICENSE.MIT          |
| yaml-cpp                   | MIT License                             | https://github.com/jbeder/yaml-cpp/blob/master/LICENSE             |
| Leaflet                    | 2-clause BSD License                    | https://github.com/Leaflet/Leaflet/blob/master/LICENSE             |
| OpenStreetMap<sup>\*</sup> | Open Data Commons Open Database License | https://www.openstreetmap.org/copyright                            |
| Google Material Icons      | Apache License Version 2.0              | https://www.apache.org/licenses/LICENSE-2.0.txt                    |
| Webpack                    | MIT License                             | https://github.com/webpack/webpack/blob/master/LICENSE             |
| Webpack CLI                | MIT License                             | https://github.com/webpack/webpack-cli/blob/master/LICENSE         |
| Webpack Dev Server         | MIT License                             | https://github.com/webpack/webpack-dev-server/blob/master/LICENSE  |
| HTML Webpack Plugin        | MIT License                             | https://github.com/jantimon/html-webpack-plugin/blob/main/LICENSE  |
| Easeljs                    | MIT License                             | https://github.com/CreateJS/EaselJS/blob/master/LICENSE.txt        |
| Roslibjs                   | BSD License                             | https://github.com/RobotWebTools/roslibjs/blob/develop/LICENSE     |
| Ros2djs                    | BSD License                             | https://github.com/RobotWebTools/ros2djs/blob/develop/LICENSE      |

<span style="font-size:smaller">\* © OpenStreetMap contributors. Base map and data from OpenStreetMap and OpenStreetMap Foundation.</span>

<br/><br/>

## _3. System requirements_

- Ubuntu 20.04 and ROS Noetic.
- Current version of a web browser, at least<sup>\*</sup>:
  - Chrome 89+
  - Firefox 86+
  - Safari 14+

<span style="font-size:smaller">\* Older destop versions might also work fine, but mobile ones probably won't.</span>
<br/><br/>

**Note**: The FTT currently runs with Python 3. Python 2 support ended with version 2.1.
<br/><br/>

## _4. Installation_

**Note**: See installation with Docker at the end of the section.

### **4.1. Installation of Python libraries**

```bash
sudo apt-get install build-essential python3-pyproj python3-catkin-tools python3-jinja2 python3-parse python3-lxml python3-ruamel.yaml python3-matplotlib python3-numpy python3-tk python3-opencv libopencv-dev libyaml-cpp-dev
sudo python3 -m pip install -U requests Pillow Flask flask-restful flask-cors psycopg2
```

### **4.2. Installation of LaTeX and libraries**

```bash
sudo apt-get install texlive texlive-lang-german texlive-latex-extra texmaker texlive-fonts-extra texlive-xetex
```

### **4.3. Installation of the Field Test Tool**

```bash
cd <ros_workspace>/src
git clone https://github.com/fkie/field_test_tool.git
rosdep install --from-paths field_test_tool/ --ignore-src
catkin build
```

### **4.4. Installation of PostgreSQL server**

```bash
sudo apt-get install postgresql postgresql-client postgis
```

### **4.5. Configuration of PostgreSQL**

Change to PostgreSQL interactive terminal (recognizable by postgres=#):

```bash
sudo -u postgres psql
```

Set password for user postgres:

```psql
\password postgres
postgres
postgres
```

Create FTT database:

```psql
CREATE DATABASE ftt;
```

Show all databases (optional):

```psql
\l
```

Change to database ftt and show it's contents:

```psql
\c ftt
\dt
```

Enable postgis and postgis_topology extensions for the current database (ftt):

```psql
CREATE EXTENSION postgis;
CREATE EXTENSION postgis_topology;
```

Quit Postgres interactive terminal:

```psql
\q
```

Create the database schema and fill some tables with the required values:

```bash
cd <ros_workspace>/src
psql -U postgres -h localhost -d ftt -a -f field_test_tool/ftt_database/postgres/ftt_schema.sql
psql -U postgres -h localhost -d ftt -a -f field_test_tool/ftt_database/postgres/setup_queries.sql
```

Alternative: If postgres server is on a different host:

```bash
cd <ros_workspace>/src
psql -h hostname -d ftt -U postgres -p 5432 -a -q -f field_test_tool/ftt_database/postgres/ftt_schema.sql ftt
psql -h hostname -d ftt -U postgres -p 5432 -a -q -f field_test_tool/ftt_database/postgres/setup_queries.sql ftt
```

### **4.6. Installation texmaker (optional)**

Texmaker is an editor of LaTeX files.

```bash
sudo apt-get install texmaker
```

### **4.7. Installation pgAdmin 4 (optional)**

PgAdmin is a management software for the PostgreSQL-database. Follow the installation instruction from the [pgAdmin's website](https://www.pgadmin.org/download/pgadmin-4-apt/)

### **4.8. Installation of the FTT Web GUI**

Once cloned, the repository already contains all the files ready for the web server to serve the web application. **However**, to further develop, make changes to the JavaScript code or use the application in offline mode, certain tools/libraries will be needed and the following steps will walk you through the installation process.

The tools will basically allow you to **build** the separate JavaScript files into a single script, which could then also be optimized for browser execution. The download of the extra libraries will allow **offline** execution of the application.

**Alternatively**, the whole application can be made to work **_without_** the extra tools or libraries. Beware though, that this means loosing the optimization and script merging capabilities as well as offline usage.

To work without the extra tools, respectively change the following lines in index.html and config.html:

```html
<script src="assets/scripts/index.js" defer type="module"></script>
```

```html
<script src="assets/scripts/config.js" defer type="module"></script>
```

For:

```html
<script src="src/index.js" defer type="module"></script>
```

```html
<script src="src/config.js" defer type="module"></script>
```

Otherwise, proceed with the installation steps below.
<br/><br/>

#### **_4.8.1. Installation of Node.js_**

Install the latest version of Node.js for you machine from https://nodejs.org. This will give you access to the **_npm_** package manager needed for the next steps.
<br/><br/>

#### **_4.8.2. Installation of npm libraries_**

The following npm libraries (and their dependencies) will be installed:

| Name                       | Description                                 |
| -------------------------- | ------------------------------------------- |
| ESLint v7.17.0             | Linting for your JS code.                   |
| Html Webpack Plugin v4.5.1 | Webpack plugin to work with multiple HTMLs. |
| Webpack v4.46.0            | Script bundling and workflow managing.      |
| Webpack CLI v3.3.9         | Webpack set of commands for developing.     |
| Webpack Dev Server v3.11.2 | Webpack simple serving for applications.    |
| Material Icons             | Offline use of Google icons.                |
| Leaflet                    | Offline use of Leaflet map viewer.          |
| Easeljs                    | Offline use of Easeljs graphics library.    |

To install them, simply navigate to your project directory and run the following npm command:

```bash
cd <ros_workspace>/src/field_test_tool/ftt_web_interface

npm install
```

#### **_4.8.3. Installation of ESLint (optional for VS Code)_**

If you're working with VS Code, linting can be made available for this project to help with development. To do this, install the ESLint extension from the extensions tab of VSCode and enable it.

Linting will be shown according to the rules set in the _.eslintrc.json_ file.
<br/><br/>

#### **_4.8.4. Useful npm commands_**

From the project directory _<ros_workspace>/src/field_test_tool/ftt_web_interface_ the following npm commands are conveniently available:

| Command            | Description |
| ------------------ | ----------- |
| npm run build      | Bundles the different scripts and builds index.js and config.js under the assets/scripts folder. |
| npm run build:dev  | Bundles the scripts and starts a development server to serve the applications at _localhost:8080_. </br>**Warning**: Since the web app is meant to be served from the same server as the database API,</br>the app won't work properly unless the server address is manually changed in _src/utility/ServerInterface.js_. |
| npm run build:prod | Just like build, but the built scripts are also optimized for production. |


### **4.9. Installation with Docker**

The project includes Docker files and a docker-compose file to speed up the deployment of the application for testing purposes.
The docker containers will build the web application's front end (GUI) (as described in [section 4.8](#48-installation-of-the-ftt-web-gui)), so beware of incompatibilities that may arise if then trying to rebuild it from the host system.

First, make sure to have [Docker Desktop](https://docs.docker.com/desktop/install/ubuntu/) installed on your machine or follow the official documentation to install it. Then, simply step in the project's directory and run docker compose:
```
cd <ros_workspace>/src/field_test_tool

docker compose up
```

Doing so will build and start three docker containers:
  * One for the FTT ROS interface node, with access to all network ports.
  * One for the FTT server and web interface (back- and front-end), with access to port 5000.
  * One for the FTT database (access only to docker's internal network).

The input source code is made available to the docker containers via volume mounting. Likewise, a volume is mounted in the FTT database directory of the host environment for the database structure.

With the docker containers running, the execution steps from sections [5.1](#51-ftt-server) and [5.2](#52-ftt-ros-interface) can be skipped.
<br/><br/>

## _5. Execution_

### **5.1. FTT server**

The following commands starts the web server:

```bash
cd <ros_workspace>/src/field_test_tool/ftt_server/scripts/

python3 api.py
```

### **5.2. FTT ROS interface**

The FTT ROS node was originally implemented in Python and then ported to C++ in order to improve the processing performance. In particular, the processing of map messages and TFs was found to be very computationally intensive for the Python script. The decision was made to keep both nodes, because the Python node still offers some extra functionality that was not considered useful enough to transfer to the new implementation.

The following command launches the ROS data collector (C++ implementation by default) for sending the operation mode, GPS position, local map and base link position data. Additionally, the rosbridge_server's rosbrige_websocket launcher file will be executed, allowing direct communication between the ROS environment and the web application.

```bash
roslaunch ftt_ros_interface ftt_ros.launch
```
The launch file will load the parameters in the YAML file of the package's configuration folder (details below) and then run the ROS node.
This node will initially only advertise three configuration services (_/set_ftt_logging_, _/get_ftt_logging_, _/save_ftt_params_). If the Python implementation is used, it will also subscribe to two debug topics (_download_image_, _download_map_).

The service "/set_ftt_logging" is used to start or stop the POSTing of data to the FTT server/database, the service "/get_ftt_logging" returns the current state of logging activity (active: true or inactive: false), and "/save_ftt_params" can be used to store the - potentially updated - node's parameters in the ROS parameters server, back to the YAML configuration file.

| Service          | Type             |
| ---------------- | ---------------- |
| /set_ftt_logging | std_srvs/SetBool |
| /get_ftt_logging | std_srvs/Trigger |
| /save_ftt_params | std_srvs/Trigger |

Once the logging is activated, the node's parameters are read, the subscribers for the robot data are created and the POSTing to the FTT server starts executing. Specifically, the following topics are subscribed (**Note**: the topic names are specified in the node's parameters, which can also be modified via the FTT web GUI, as explained in the specific documentation).

| Topic                         | Type                        |
| ----------------------------- | --------------------------- |
| robot_mode                    | industrial_msgs/RobotMode   |
| gps_fix                       | sensor_msgs/NavSatFix       |
| local_pose                    | geometry_msgs/PoseStamped   |
| map                           | nav_msgs/OccupancyGrid      |
| map_jpeg<sup>\*</sup>         | sensor_msgs/CompressedImage |
| image                         | sensor_msgs/Image           |
| image_compressed<sup>\*</sup> | sensor_msgs/CompressedImage |

<span style="font-size:smaller">\* Only in the Python implementation.</span>
<br/><br/>

Additionally, a TF listener is started to get the robot's position (_robot_frame_ parameter) in the local coordinate frame (_map_frame_ parameter). 

**Note**: For system with limited resources, it is recommended to prevent the execution of the TF listener if the logging of the local robot position is not required. This is done by setting the node's _use_tf_ parameter to _false_.
<br/><br/>

As for the aforementioned debug topics (only in the Python implementation), "download_image" triggers a GET request to the server for all images stored under the segment ID specified in the message data, then downloads them to the "Pictures" folder of the home directoy. Similarly, "download_map" fetches the stored map for the given shift ID in the message data.

| Debug Topic    | Type           |
| -------------- | -------------- |
| download_image | std_msgs/Int32 |
| download_map   | std_msgs/Int32 |


#### **5.2.1 FTT ROS interface extras**

Some extra utility ROS nodes are provided:

- robotModePublisher (Python): This node subscribes to two velocity command topics (of type _geometry_msgs/TwistStamped_), one expected to come from the autonomous navigation stack and the other from a joystick interpreter. It then identifies the operation mode and publishes messages of type indutrial_msgs/RobotMode accordingly. The autonomous mode is flagged at the arrival of autonomous command messages. The publised message switches to manual mode the moment a non-zero joystick command message arrives. After joystick messages stop, a fixed time (parameter) must elapse before returning to the autonomous mode if navigation commands are still being received. This timeout routine is implemented in order to prevent unwanted switching between modes.

  To launch the robotModePublisher, the following command can be used (parameters in the launch file):
  ```bash
  roslaunch ftt_ros_interface robot_mode_publisher.launch <args>
  ```

- robot_pose_publisher (C++): This node subscribes the robot's TFs and publishes the robot position (as a _geometry_msgs/PoseStamped_ message). It was introduced to be used together with the Python implementation of the FTT interface node, due to the fact that TF processing is more efficient in C++.

  To launch the robot_pose_publisher, the following command can be used (parameters in the launch file):
  ```bash
  roslaunch ftt_ros_interface robot_pose_publisher.launch <args>
  ```

- map_to_jpeg (C++): This node subscribes the robot's map (of type _nav_msgs/OccupancyGrid_) and publishes it as an image (as a _sensor_msgs/CompressedImage_ message). It was introduced to be used together with the Python implementation of the FTT interface node, due to the fact that occupancy grid to image conversion is more efficient in C++. It's important to note that for this to work properly, FTT interface node must subscribe to both the occupancy grid map and map image topics, as the former includes resolution and origin information, which the latter doesn't.

  To launch the map_to_jpeg, the following command can be used (parameters in the launch file):
  ```bash
  roslaunch ftt_ros_interface map_to_jpeg.launch <args>
  ```

### **5.3. FTT web GUI**

After running the FTT server API, visit http://localhost:5000/ from your web browser (alternatively, the IP address of the machine running the server). Usage instructions can be found under the _docs_ directory of this repository.
<br/><br/>

### **5.4. PDF-Report generator**

The following commands will run the report generator. The output PDF report will be available at _<ros_workspace>/src/field_test_tool/ftt_report_generator/build/report.pdf_. The report configuration options are explained below.

```bash
cd <ros_workspace>/src/field_test_tool/ftt_report_generator/src/

python3 db2rep.py <path_to_config_file> (e.g. ../config/2021_fkie_test.xml)
```

### **5.5. FTT database**

Usually, you won't need to directly interact with the database after its initial setup, but the ```ftt_database``` folder of the repository does include a utility Python script called ```mergeDbs.py```. This script can be used to copy the data stored in a souce ftt database (e.g. running in your robot) to a target ftt database (e.g. running in your development pc). To run it, just specify the connection information for both databases as arguments to the program:
```bash
cd <ros_workspace>/src/field_test_tool/ftt_database/scripts/
python3 mergeDbs.py "host=<source_system_ip> dbname=ftt user=postgres password=postgres" "host=<target_system_ip> dbname=ftt user=postgres password=postgres"
```
<br/><br/>

## _6. FTT ROS node parameters_

The FTT ROS interface node has its parameters loaded from a YAML file in the package's configuration folder. These parameters are explained below. Please note that some parameters are only used in the Python implementation of the node.

| Parameter Name           | Default Value    | Description                                                              |
| ------------------------ | ---------------- | ------------------------------------------------------------------------ |
| /set_ftt_logging_service | /set_ftt_logging | Name of the service to start/stop the node. Should keep default.         |
| /get_ftt_logging_service | /get_ftt_logging | Name of the service to get the node's status. Should keep default.       |
| /save_ftt_params_service | /save_ftt_params | Name of the service to store the node's parameters. Should keep default. |
| topics/robot_mode        | robot_mode       | Name of the topic for robot operating mode.                              |
| topics/gps_fix           | gps_fix          | Name of the topic for GPS position data.                                 |
| topics/local_pose        | local_pose       | Name of the topic for robot local position data.                         |
| topics/map               | map              | Name of the topic for the map of the environment.                        |
| topics/map_jpeg          | map_jpeg         | Name of the topic for the map image (must be the same map as the above). |
| topics/image             | image            | Name of the topic for robot frontal camera (raw) images.                 |
| topics/image_compressed  | image_compressed | Name of the topic for robot frontal camera (compressed) images.          |
| params/use_tf            | true             | Flag to create a TF listener to extract the robot's local position.      |
| params/map_frame         | map              | Name of the map frame for the tf listener.                               |
| params/robot_frame       | base_link        | Name of the base link frame for the tf listener.                         |  
| params/server_address    | localhost:5000   | Database API server IP and port.                                         |
| params/send_pose_period  | 2.0              | Time period for sending position data to the database.                   |
| params/send_map_period   | 2.0              | Time period for sending map data to the database.                        |
| image_buffer_size        | 1                | Number of images keep in buffer.                                         |
| image_buffer_step        | 1.0              | Minimum time interval between buffered images.                           |

<br/><br/>

### **6.1 Extra nodes parameters**

The robotModePublisher node also requires some parameters to be set when using it. Unlike the ftt_ros node, these are set directly in the launcher file: robot_mode_publisher.launch.

| Parameter Name   | Description                                                                                           |
| ---------------- | ----------------------------------------------------------------------------------------------------- |
| JOY_CMD_TOPIC    | ROS topic for joystick velocity command messsages of type _geometry_msgs/TwistStamped_.               |
| NAV_CMD_TOPIC    | ROS topic for navigation velocity command messsages of type _geometry_msgs/TwistStamped_.             |
| ROBOT_MODE_TOPIC | ROS topic for robot operating mode of type _industrial_msgs/RobotMode_.                               |
| JOY_CMD_TIMEOUT  | Parameter to set the mininum time to switch out of manual mode after joystick commands stop arriving. |
| PUBLISH_PERIOD   | Parameter to set the interval time for publishing robot mode messages.                                |

<br/><br/>

The same holds true for the robot_pose_publisher node:

| Parameter Name   | Description                                                                               |
| ---------------- | ----------------------------------------------------------------------------------------- |
| robot_pose_topic | ROS topic for the published robot pose of type _geometry_msgs/PoseStamped_.               |
| robot_frame      | Parameter to set the name of the robot frame (position of this frame relative to global). |
| global_frame     | Parameter to set the name of the global frame.                                            |
| publish_rate     | Parameter to set the frequency for publishing robot pose messages.                        |

<br/><br/>

And also for the map_to_jpeg node:

| Parameter Name | Description                                                                  |
| -------------- | ---------------------------------------------------------------------------- |
| map_topic      | ROS topic for the subscribed map of type _nav_msgs/OccupancyGrid_.           |
| map_jpeg_topic | ROS topic for the published map image of type _sensor_msgs/CompressedImage_. |
| publish_rate   | Parameter to set the frequency for publishing robot pose messages.           |

<br/><br/>

## _7. Report Generator Configuration File_

The report generator script requires an XML file with the following structure:

```xml
<testevent>

  <resource tile_server = "<server_dir>" zoom_level = "<integer>"/>

  <postgis host = "localhost" dbname="ftt" user ="postgres" password = "postgres"/>

  <report name="<report_name>" version="<version>">
    <test_event id="<test_event_id>" min_dur="<min_duration_in_hours>" local="<true/false>"/>
    <recipient name="<name>" address="<address>"/>
    <creator name="<name>" address="<address>"/>
    <logos top_logo_path = "<path_to_front_page_top_logo>" bottom_logo_path = "<path_to_front_page_bottom_logo>"/>
  </report>

</testevent>
```

Most of the parameters are self-explanatory, but the following should be noted:

- The **_server_dir_** parameter expects a URL to a tile server with an API that considers the _x_ and _y_ coordinates, and _zoom_ (z) of the requested tile. The URL should be parameterized as follows: **http://your.tile.server/{z}/{x}/{y}**. An example would be _<span>http://a.tile.openstreetmap.org/{z}/{x}/{y}.png</span>_.
- The **_zoom_level_** parameter sets the zoom level used for fetching the tiles from the tile server.
- The **_report_name_** and **_version_** parameters are simply for the text to be displayed on the front page of the report.
- The **_test_event_id_** parameter specifies the test event for which the information will be processed and written to the report.
- The **_local_** parameter controls the usage of either locally referenced positions or GPS data.
- The **_min_dur_** parameter is simply used to set a minimum length for the timeline generated in the report and the number should be specified in hours.

<br/><br/>

## _8. Offline usage of the web GUI (local tile server)_

Completing the installation steps for the web GUI enables the offline usage of the application on your machine, the one **exception** being the tile server.

A tile server is needed for the leaflet map viewer in the FTT. By default, OpenStreetMap is used. Without internet access, a local tile server must be configured. 

### **8.1. Configuring a tile server with OSM data**

A guide on how to do this can be found at:

- https://switch2osm.org/serving-tiles/manually-building-a-tile-server-18-04-lts

A docker container is also available at:

- https://github.com/Overv/openstreetmap-tile-server

### **8.2. Configuring a tile server with a GeoTIFF image**

Serving tiles from a GeoTIFF image is easier done with the [GeoServer](https://geoserver.org/) tool. A handy docker container can be found at:

- https://github.com/kartoza/docker-geoserver

That repository includes sample docker-compose.yml and .env files to quicky configure and install the application. Once installed, the GeoServer needs to be set up to serve tiles from the GeoTIFF image. The following steps will walk you through this process.

1. Move your GeoTIFF data to the GeoServer docker image data volume. This is easier done by modifying the docker-compose file to mount the volume in a local rounte of the host machine, e.g. by changig:
    ```
    geoserver:
      volumes:
        - geoserver-data:/opt/geoserver/data_dir
    ```
    To:
    ```
    geoserver:
      volumes:
        - ./geoserver_data:/opt/geoserver/data_dir
    ```
    And then creating a directory called `geoserver_data` in the same location as your docker-compose file.

2. Set up a Workspace in the GeoServer. Go to the GeoServer home page (`http://localhost:8600/geoserver` by default in the sample configuration) and log in using your credentials (by default, user: `admin`, password: `myawesomegeoserver`). On the left side, click on `Workspaces` then `Add new workspace`. You can choose a name and URI, such as `ftt` and `http://localhost:8600/geoserver/ftt`, then click `save`.

3. Set up a Store. Again on the left side, click on `Store`, then `Add new store`. Select `GeoTIFF` as the data source type. Pick a data source name, write a description and browse for your .tiff image. Click `save`. 

4. Set up a Layer. On the page that opens after step 3, click on `Publish`. Change the `Name` of the layer to something simple (e.g. `MyLayer`), then click on `save` at the bottom of the page. 

5. Set up Tile Caching. Once again on the left side, click on `Tile Layers`. Select your newly created layer (ftt:MyLayer in this example) and under the `Tile Caching` tab, near the bottom of the page, expand the list for `Add grid subset`. Type and select `WebMercatorQuad` and click on the plus icon on the right to add it. Click `save`.

6. You can now fetch the tiles using the following URL format (note that this may vary depending on your workspace and layer name).

    - http://localhost:8600/geoserver/gwc/service/tms/1.0.0/ftt:MyLayer/{z}/{x}/{y}.png?flipY=true
