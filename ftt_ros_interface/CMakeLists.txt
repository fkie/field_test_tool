cmake_minimum_required(VERSION 3.5)
project(ftt_ros_interface)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.27")
    cmake_policy(SET CMP0135 NEW)
endif()

include(FetchContent)
set(CPR_USE_SYSTEM_CURL ON) # System CURL must be >=7.64.0. Otherwise change this to OFF and use the CPR commit hash for version 1.8.3
FetchContent_Declare(cpr GIT_REPOSITORY https://github.com/libcpr/cpr.git
                        #  GIT_TAG db351ffbbadc6c4e9239daaa26e9aefa9f0ec82d # The commit hash for 1.8.3.
                         GIT_TAG dd967cb48ea6bcbad9f1da5ada0db8ac0d532c06) # The commit hash for 1.11.2. Replace with the latest from: https://github.com/libcpr/cpr/releases
FetchContent_MakeAvailable(cpr)

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

add_executable(ros2api src/ros2api.cpp src/base64.cpp)
target_include_directories(ros2api PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(ros2api PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2::tf2
  image_transport::image_transport
  cv_bridge::cv_bridge
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  cpr::cpr
  nlohmann_json::nlohmann_json
)

# Install Python executables
install(PROGRAMS
  scripts/robot_mode_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
    ros2api
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
