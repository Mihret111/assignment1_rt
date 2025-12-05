# Makefile for assignment1_rt

# Workspace root (assuming we are in src/assignment1_rt)
WS_DIR = ../..

# Package name
PKG_NAME = assignment1_rt

.PHONY: all build clean run run_sim run_spawner run_safety run_ui

all: build

build:
	@echo "Building package $(PKG_NAME)..."
	cd $(WS_DIR) && colcon build --packages-select $(PKG_NAME) --symlink-install

clean:
	@echo "Cleaning package $(PKG_NAME)..."
	rm -rf $(WS_DIR)/build/$(PKG_NAME)
	rm -rf $(WS_DIR)/install/$(PKG_NAME)
	@echo "Clean complete."

# ROS2 Setup (Adjust if needed, detected: jazzy)
ROS_DISTRO_SETUP = /opt/ros/jazzy/setup.bash

# Run all components in separate windows
# Run all components in separate windows
# Run all components in separate windows
run:
# Run all components
run:
	@echo "Killing previous processes..."
	@killall -q turtlesim_node safety_node ui_node || true
	@sleep 1
	@echo "Launching system..."
	@# Launch Turtlesim (Tab)
	gnome-terminal --tab --title="Turtlesim Simulation" -- bash -c "source $(ROS_DISTRO_SETUP); source $(WS_DIR)/install/setup.bash; ros2 run turtlesim turtlesim_node"
	@echo "Waiting for Turtlesim to start..."
	@sleep 3
	@# Launch Spawner (Inline, wait for exit)
	bash -c "source $(ROS_DISTRO_SETUP); source $(WS_DIR)/install/setup.bash; python3 spawner/turtle_spawn.py"
	@# Launch Safety Node (Separate Window)
	gnome-terminal --title="Safety Node Inspection" --geometry=80x20 -- bash -c "source $(ROS_DISTRO_SETUP); source $(WS_DIR)/install/setup.bash; ros2 run $(PKG_NAME) safety_node"
	@# Launch UI Node (New Tab - to ensure it gets focus at the end)
	@echo "Starting User Interface in a new tab..."
	gnome-terminal --tab --title="User Interface" -- bash -c "source $(ROS_DISTRO_SETUP); source $(WS_DIR)/install/setup.bash; ros2 run $(PKG_NAME) ui_node"

# Individual run commands (require separate terminals manually)

run_sim:
	@echo "Starting Turtlesim..."
	ros2 run turtlesim turtlesim_node

run_spawner:
	@echo "Spawning Turtle2..."
	python3 spawner/turtle_spawn.py

run_safety:
	@echo "Starting Safety Node..."
	ros2 run $(PKG_NAME) safety_node

run_ui:
	@echo "Starting UI Node..."
	ros2 run $(PKG_NAME) ui_node

# Helper to source the setup file
setup:
	@echo "Run this in your shell: source $(WS_DIR)/install/setup.bash"
