#!/bin/bash

# Define colors for highlighting
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if "screen" is available, if not, install it
if ! command -v screen &> /dev/null; then
    echo -e "${RED}'screen' utility not found. Installing...${NC}"
    sudo apt update
    sudo apt install -y screen
    echo -e "${GREEN}'screen' utility installed successfully.${NC}"
fi

# Resolve the actual script directory, considering symbolic link
SOURCE_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

# Step 1: Check for existing ROS2 node and screen session
SCREEN_NAME="ftt_ros_launch"
ROS_NODE_NAME="ftt_ros"

ROS_NODE_EXISTS=$(ros2 node list | grep -w "$ROS_NODE_NAME")
SCREEN_EXISTS=$(screen -list | grep -w "$SCREEN_NAME")

if [ -n "$ROS_NODE_EXISTS" ]; then
    echo -e "${RED}ROS2 node '$ROS_NODE_NAME' is already running. Skipping launch.${NC}"
elif [ -n "$SCREEN_EXISTS" ]; then
    echo -e "${RED}Screen session '$SCREEN_NAME' already exists. Skipping launch.${NC}"
else
    echo -e "${YELLOW}Starting ROS2 nodes in a separate screen...${NC}"
    screen -dmS "$SCREEN_NAME" bash -c "source ~/.bashrc && ros2 launch ftt_ros_interface ftt_ros.launch.xml"
    echo -e "${GREEN}ROS2 nodes launched in screen: $SCREEN_NAME${NC}"
fi

# Step 2: Execute the Python script for the web server with forwarded arguments
echo -e "${YELLOW}Starting Python web server...${NC}"
python3 "$SOURCE_DIR/ftt_server/scripts/api.py" "$@"

# Step 3: Kill the screen session for the ROS nodes before exiting
echo -e "${YELLOW}Terminating ROS2 nodes screen...${NC}"
screen -S "$SCREEN_NAME" -X quit
echo -e "${GREEN}ROS2 nodes screen terminated.${NC}"

echo -e "${GREEN}Exiting.${NC}"
echo -e "${GREEN}Bye~${NC}"