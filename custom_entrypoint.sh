#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Change to workspace directory
cd /ros2_ws

# Optional: resolve system dependencies with rosdep if requested
if [ "${ROSDEP_INSTALL}" = "1" ]; then
	echo "[entrypoint] ROSDEP_INSTALL=1; resolving system dependencies with rosdep..."
	if [ "$(id -u)" -ne 0 ]; then
		echo "[entrypoint] Insufficient privileges. Re-run with --user root (or include sudo/gosu) to allow package installation."
	else
		rosdep update
		rosdep install --from-paths src --ignore-src -r -y
	fi
fi

# Detect packages before building
shopt -s nullglob
pkgs=(/ros2_ws/src/*/package.xml /ros2_ws/src/*/*/package.xml)
if [ ${#pkgs[@]} -gt 0 ]; then
	if [ "${SKIP_COLCON_BUILD}" = "1" ] && [ -f /ros2_ws/install/setup.bash ]; then
		echo "[entrypoint] SKIP_COLCON_BUILD=1; using existing overlay."
		source /ros2_ws/install/setup.bash
	else
		echo "[entrypoint] Packages detected; running colcon build..."
		colcon build --symlink-install
		source /ros2_ws/install/setup.bash
	fi
else
	echo "[entrypoint] No packages found under /ros2_ws/src; skipping build."
fi
exec "$@"
