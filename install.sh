#!/bin/bash

# Define colors for highlighting
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Store the directory of the installation script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Step 1: Install dependencies
echo -e "${YELLOW}Starting installation of dependencies...${NC}"
sudo apt update
sudo apt install -y build-essential python3-pyproj python3-jinja2 python3-parse python3-lxml python3-ruamel.yaml python3-matplotlib python3-numpy python3-tk python3-opencv libopencv-dev libyaml-cpp-dev libcurl4-openssl-dev python3-requests python3-pil python3-psycopg2 python3-flask python3-flask-restful python3-flask-cors texlive texlive-lang-german texlive-latex-extra texlive-fonts-extra texlive-xetex postgresql postgresql-client postgis
echo -e "${GREEN}Dependencies installed successfully.${NC}"

# Step 2: Deduce ROS2 workspace using COLCON_PREFIX_PATH
echo -e "${YELLOW}Locating ROS2 workspace...${NC}"
if [ -z "$COLCON_PREFIX_PATH" ]; then
  echo -e "${RED}Error: COLCON_PREFIX_PATH is not set! Please initialize your ROS2 workspace before executing this script.${NC}"
  exit 1
fi

ROS2_WORKSPACE=$(dirname "$COLCON_PREFIX_PATH")
echo -e "${GREEN}ROS2 workspace located at: $ROS2_WORKSPACE${NC}"

cd "$ROS2_WORKSPACE"
echo -e "${YELLOW}Installing FTT ROS dependencies...${NC}"
rosdep install --from-paths src/field_test_tool/ -y --ignore-src
echo -e "${YELLOW}Building FTT ROS package...${NC}"
colcon build --symlink-install --packages-select ftt_ros_interface
echo -e "${GREEN}FTT ROS package built successfully.${NC}"

# Step 3: Run database setup script
echo -e "${YELLOW}Setting up database...${NC}"
./src/field_test_tool/ftt_database/postgres/setup_ftt_db.sh
echo -e "${GREEN}Database setup completed.${NC}"

# Step 4: Check and install Node.js version if necessary
echo -e "${YELLOW}Checking Node.js version...${NC}"
NODE_VERSION=$(node -v 2>/dev/null | grep -oP "\d+" | head -n 1)
if [ -z "$NODE_VERSION" ] || [ "$NODE_VERSION" -lt 20 ]; then
  echo -e "${RED}Node.js version is too low. Installing Node.js version 22.x...${NC}"
  curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
  sudo apt install -y nodejs
  echo -e "${GREEN}Node.js version 22.x installed.${NC}"
else
  echo -e "${GREEN}Node.js version is sufficient: v$NODE_VERSION${NC}"
fi

# Step 5: Execute npm install
echo -e "${YELLOW}Installing npm packages...${NC}"
cd src/field_test_tool/ftt_web_interface
npm install
echo -e "${GREEN}npm packages installed successfully.${NC}"

# Step 6: Create a symbolic link to the run script in ~/.local/bin
TARGET_DIR="$HOME/.local/bin"
TARGET_LINK="$TARGET_DIR/field-test-tool"

echo -e "${YELLOW}Creating symbolic link to run script as 'field-test-tool' in ~/.local/bin...${NC}"
mkdir -p "$TARGET_DIR"
ln -sf "$SCRIPT_DIR/run.sh" "$TARGET_LINK"
echo -e "${GREEN}Symbolic link created successfully as 'field-test-tool'.${NC}"

# Ensure ~/.local/bin is in PATH
if ! echo "$PATH" | grep -q "$TARGET_DIR"; then
  echo -e "${YELLOW}Adding ~/.local/bin to PATH...${NC}"
  echo "export PATH=\"\$PATH:$TARGET_DIR\"" >> ~/.bashrc
  source ~/.bashrc
  echo -e "${GREEN}PATH updated successfully.${NC}"
fi

# Finished
echo -e "${GREEN}Installation completed.${NC}"
