# Robot UI - Tauri + React + ROS2

A Tauri desktop application with ROS2 integration for robot control and monitoring.

## Features

- ROS2 integration with rclrs (Rust bindings)
- Subscribes to `/chatter` topic (std_msgs/String)
- Real-time message display in UI
- Built with Tauri, React, and TypeScript

## Prerequisites

- ROS2 (Humble or later)
- Node.js and pnpm
- Rust toolchain
- colcon build tool

## How to Run

1. **First time setup - Build ROS2 workspace:**
   ```bash
   cd ros2_ws
   colcon build --packages-up-to rclrs std_msgs builtin_interfaces
   ```

2. **Run the application:**
   ```bash
   source ros2_ws/install/setup.bash
   pnpm tauri dev
   ```

3. **Test with ROS2 publisher** (in another terminal):
   
   Publish manually:
   ```bash
   source ros2_ws/install/setup.bash
   ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2!'" --rate 1
   ```
   
   Or use Python:
   ```bash
   source ros2_ws/install/setup.bash
   ros2 run demo_nodes_py talker
   ```

## Makefile Commands (convenience wrappers)

- `make build-ros2` - Build the ROS2 workspace
- `make dev` - Source ROS2 and start the app
- `make clean` - Clean all build artifacts

**Note:** Run make commands as the same user who owns the workspace to avoid permission issues.

## Makefile Commands

- `make build-ros2` - Build the ROS2 workspace
- `make run` - Build ROS2 workspace and start the app
- `make dev` - Start the app (assumes ROS2 already built)
- `make clean` - Clean all build artifacts

## Recommended IDE Setup

- [VS Code](https://code.visualstudio.com/) + [Tauri](https://marketplace.visualstudio.com/items?itemName=tauri-apps.tauri-vscode) + [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer)
