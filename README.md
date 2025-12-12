# Robot UI - Tauri + React + ROS2

A modern desktop application for robot visualization and monitoring, built with Tauri, React, and ROS2 integration using rclrs (Rust ROS2 bindings).

## 🎯 Features

- **Real-time ROS2 Integration**: Native Rust ROS2 bindings using rclrs 0.6.0
- **Live Camera Feed**: Subscribe to and display compressed images from `/image/compressed` topic
- **Text Message Monitoring**: Display messages from `/chatter` topic in real-time
- **Modern UI**: Built with React + TypeScript + Vite
- **Desktop Native**: Powered by Tauri for lightweight, secure desktop apps
- **Docker Support**: Fully containerized development environment

## 📋 Prerequisites

### Docker Setup (Recommended)
- Docker and Docker Compose
- X11 server access for GUI (Linux/WSL)

### Native Setup
- ROS2 Jazzy
- Rust toolchain (1.75+)
- Node.js (v18+) and pnpm
- colcon build tool
- Python 3 with colcon plugins

## 🚀 Quick Start (Docker)

### 1. Start the Docker Container

```bash
# Start the container
docker-compose up -d

# Enter the container
docker exec -it robot_ui bash
```

### 2. Build ROS2 Workspace (First Time Only)

Inside the container:
```bash
cd /root
make build-ros2
```

This builds all required ROS2 packages including:
- rclrs (ROS2 Rust client library)
- sensor_msgs (for compressed images)
- std_msgs (for text messages)
- All message dependencies

### 3. Run the Application

Inside the container:
```bash
make dev
```

The app will start and display a GUI window with:
- 📷 Camera feed viewer (top)
- 📡 Text message viewer (bottom)

### 4. Publish Test Data

**In another terminal**, enter the container and publish test data:

#### For Text Messages:
```bash
docker exec -it robot_ui bash
source /root/ros2_ws/install/setup.bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2!'" --rate 1
```

#### For Camera Images:
```bash
# Terminal 1: Publish raw images from webcam
ros2 run image_tools cam2image --ros-args -r image:=/image

# Terminal 2: Convert to compressed format
ros2 run image_transport republish raw compressed \
  --ros-args -r in:=/image -r out/compressed:=/image/compressed
```

## 📁 Project Structure

```
/root/ (container) = /home/yano/rust/robot_ui/ (host)
├── src/                        # React frontend
│   ├── App.tsx                # Main UI component
│   ├── App.css                # Styles
│   └── main.tsx               # React entry point
├── src-tauri/                  # Rust backend
│   ├── src/
│   │   └── lib.rs             # ROS2 subscriber & Tauri app logic
│   ├── Cargo.toml             # Rust dependencies with ROS2 paths
│   └── tauri.conf.json        # Tauri configuration
├── ros2_ws/                    # ROS2 workspace
│   ├── src/                   # ROS2 source packages
│   ├── install/               # Built ROS2 packages
│   └── build/                 # Build artifacts
├── Makefile                    # Build commands
├── docker-compose.yml          # Docker configuration
├── Dockerfile                  # Container definition
└── README.md                   # This file
```

## 🔧 Development Workflow

### Inside Docker Container (`/root`)

**Run in dev mode** (hot reload):
```bash
make dev
```

**Rebuild ROS2 packages**:
```bash
make build-ros2
```

**Clean build artifacts**:
```bash
make clean
```

### On Host Machine

**Edit code**: Changes in `/home/yano/rust/robot_ui/` sync automatically via volume mount
- Frontend: `src/`
- Backend: `src-tauri/src/`
- Vite hot-reloads frontend changes
- Cargo watches for backend changes

**Docker commands**:
```bash
# Enter container
make docker-shell  # or: docker exec -it robot_ui bash

# Start container
make docker-up     # or: docker-compose up -d

# Stop container
make docker-down   # or: docker-compose down
```

## 📡 ROS2 Topics

The application subscribes to:

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/chatter` | `std_msgs/msg/String` | Text messages for monitoring |
| `/image/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG/PNG compressed camera images |

## 🛠️ Technical Details

### Backend (Rust)
- **Framework**: Tauri 2.x
- **ROS2 Client**: rclrs 0.6.0 (ros2-rust)
- **Architecture**: 
  - ROS2 subscriber runs in background thread
  - Emits Tauri events to frontend on message receipt
  - Base64 encodes compressed images for web display

### Frontend (React + TypeScript)
- **Build Tool**: Vite 6.x
- **Event Handling**: Tauri API event listeners
- **Image Display**: Base64-encoded data URLs
- **Styling**: Inline styles with dark theme

### ROS2 Integration
- **Distribution**: ROS2 Jazzy
- **Workspace**: Built from source (ros2-rust)
- **Dependencies**: Uses path dependencies to workspace install directory
- **Message Patching**: All ROS2 message packages patched in Cargo.toml to use local builds

## 🐛 Troubleshooting

### White Screen / UI Not Loading
- Refresh the window (Ctrl+R)
- Check browser console (F12) for errors
- Verify Vite server is running on port 1420

### ROS2 Build Errors
```bash
# Clean and rebuild
make clean
make build-ros2
```

### Permission Issues
If you see permission errors, ensure you're running inside the Docker container as root.

### No Images Displayed
1. Check topic is publishing: `ros2 topic hz /image/compressed`
2. Verify message type: `ros2 topic info /image/compressed -v`
3. If only `/image` exists, run the republish command:
   ```bash
   ros2 run image_transport republish raw compressed \
     --ros-args -r in:=/image -r out/compressed:=/image/compressed
   ```

### Cargo Dependency Errors
If you see "yanked version" errors, add the package to `[patch.crates-io]` section in `src-tauri/Cargo.toml`:
```toml
[patch.crates-io]
package_name = { path = "/root/ros2_ws/install/package_name/share/package_name/rust" }
```

## 📝 Notes

- **GTK Warnings**: Harmless warnings about accessibility bus and canberra-gtk-module can be ignored
- **X11 Display**: The Docker container shares your host's X11 display for GUI rendering
- **Network Mode**: Container uses host networking for easy ROS2 communication
- **Volume Mounting**: Code changes on host automatically sync to container

## 🎓 Resources

- [Tauri Documentation](https://tauri.app/)
- [ros2-rust GitHub](https://github.com/ros2-rust/ros2_rust)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [rclrs Documentation](https://docs.rs/rclrs/)

## �� License

[Add your license here]

## 👥 Contributing

[Add contribution guidelines here]

---

**Built with** 🦀 Rust • ⚛️ React • 🤖 ROS2
