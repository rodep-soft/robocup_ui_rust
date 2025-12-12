
# Build ROS2 workspace with all dependencies (run inside Docker container)
build-ros2:
	cd ros2_ws && colcon build --packages-up-to rclrs std_msgs builtin_interfaces sensor_msgs

# Source ROS2 and run Tauri app (run inside Docker container)
run: build-ros2
	bash -c "source ros2_ws/install/setup.bash && pnpm tauri dev"

# Run without rebuilding ROS2 (run inside Docker container)
dev:
	bash -c "source ros2_ws/install/setup.bash && pnpm tauri dev"

# Clean build artifacts
clean:
	cd ros2_ws && rm -rf build install log || true
	cd src-tauri && cargo clean || true

# Docker helpers - run these on HOST
docker-shell:
	docker exec -it robot_ui bash

docker-up:
	docker-compose up -d

docker-down:
	docker-compose down

