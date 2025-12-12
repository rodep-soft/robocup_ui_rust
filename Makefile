
# Build ROS2 workspace with all dependencies
build-ros2:
	cd ros2_ws && colcon build --packages-up-to rclrs std_msgs builtin_interfaces

# Source ROS2 and run Tauri app
run: build-ros2
	bash -c "source ros2_ws/install/setup.bash && pnpm tauri dev"

# Run without rebuilding ROS2
dev:
	bash -c "source ros2_ws/install/setup.bash && pnpm tauri dev"

# Clean build artifacts
clean:
	cd ros2_ws && sudo rm -rf build install log || true
	cd src-tauri && cargo clean || true

# Fix permissions after building as root
fix-perms:
	chown -R yano:yano ros2_ws/install ros2_ws/build ros2_ws/log 2>/dev/null || true

