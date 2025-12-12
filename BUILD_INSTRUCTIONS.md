# Robot UI Development Guide

## ✅ Current Status

**FULLY WORKING** - Application successfully integrates ROS2 with Tauri!

### Implemented Features:
- ✅ Tauri app builds and runs
- ✅ ROS2 rclrs integration working
- ✅ `/chatter` topic subscription (std_msgs/String)
- ✅ `/image/compressed` topic subscription (sensor_msgs/CompressedImage)
- ✅ Real-time UI updates with live camera feed
- ✅ Base64 image encoding for web display
- ✅ Docker containerized development

## 🐳 Docker Workflow

### Working Directory
You work inside the Docker container at `/root/`, which is mounted from your host at `/home/yano/rust/robot_ui/`.

### Enter the container:
```bash
docker-compose up -d
docker exec -it robot_ui bash
```

## 🏗️ Build & Run

### First Time Setup

1. **Build ROS2 Workspace** (only needed once):
```bash
cd /root
make build-ros2
```

This compiles:
- rclrs (ROS2 Rust client library)
- sensor_msgs, std_msgs, geometry_msgs
- All message dependencies with Rust bindings

2. **Run the app**:
```bash
make dev
```

### Daily Development Workflow

```bash
# Run the app
make dev

# In another terminal, publish test data:
docker exec -it robot_ui bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Test'" --rate 1

# For camera images:
ros2 run image_tools cam2image --ros-args -r image:=/image
ros2 run image_transport republish raw compressed \
  --ros-args -r in:=/image -r out/compressed:=/image/compressed
```

### Rebuild After Changes

- **Frontend changes**: Auto hot-reload (Vite)
- **Backend changes**: Auto recompile (Cargo watch)
- **ROS2 package changes**: `make build-ros2`
- **Clean everything**: `make clean`

## 📝 Key Implementation Details

### ROS2 Integration Solution

The challenge was integrating colcon-built ROS2 packages with Tauri's cargo build system.

**Solution Implemented:**
1. Build ros2_rust workspace separately with colcon
2. Use path dependencies in `Cargo.toml` pointing to install directory
3. Add `[patch.crates-io]` section to override yanked crates.io versions
4. All message packages reference local workspace builds

### Cargo.toml Configuration

```toml
[dependencies]
rclrs = { path = "/root/ros2_ws/install/rclrs/share/rclrs/rust" }
std_msgs = { path = "/root/ros2_ws/install/std_msgs/share/std_msgs/rust" }
sensor_msgs = { path = "/root/ros2_ws/install/sensor_msgs/share/sensor_msgs/rust" }

[patch.crates-io]
builtin_interfaces = { path = "/root/ros2_ws/install/builtin_interfaces/share/builtin_interfaces/rust" }
std_msgs = { path = "/root/ros2_ws/install/std_msgs/share/std_msgs/rust" }
geometry_msgs = { path = "/root/ros2_ws/install/geometry_msgs/share/geometry_msgs/rust" }
service_msgs = { path = "/root/ros2_ws/install/service_msgs/share/service_msgs/rust" }
action_msgs = { path = "/root/ros2_ws/install/action_msgs/share/action_msgs/rust" }
unique_identifier_msgs = { path = "/root/ros2_ws/install/unique_identifier_msgs/share/unique_identifier_msgs/rust" }
```

### Backend Architecture (src-tauri/src/lib.rs)

```rust
// ROS2 runs in background thread
std::thread::spawn(move || {
    run_ros2_subscriber(app_handle)
});

// Two separate workers for independent subscriptions
let worker1 = node.create_worker::<usize>(0);  // /chatter
let worker2 = node.create_worker::<usize>(0);  // /image/compressed

// Emit Tauri events to frontend
app_handle.emit("chatter-message", data);
app_handle.emit("compressed-image", image_data);
```

### Frontend Architecture (src/App.tsx)

```typescript
// Listen for ROS2 events
useEffect(() => {
  const unlistenChatter = listen<ChatterMessage>("chatter-message", ...);
  const unlistenImage = listen<CompressedImageMessage>("compressed-image", ...);
  
  return () => {
    unlistenChatter.then(fn => fn());
    unlistenImage.then(fn => fn());
  };
}, []);

// Display base64 encoded images
<img src={`data:${mimeType};base64,${data_base64}`} />
```

## 🔧 Adding New ROS2 Topics

To subscribe to additional topics:

1. **Add message package to Cargo.toml**:
```toml
nav_msgs = { path = "/root/ros2_ws/install/nav_msgs/share/nav_msgs/rust" }
```

2. **Build the package**:
```bash
cd /root/ros2_ws
colcon build --packages-up-to nav_msgs
```

3. **Add subscription in lib.rs**:
```rust
let worker = node.create_worker::<usize>(0);
let _sub = worker.create_subscription::<nav_msgs::msg::Odometry, _>(
    "odom",
    move |count, msg| {
        app_handle.emit("odom-data", msg);
    },
)?;
```

4. **Handle in frontend**:
```typescript
const unlisten = listen<OdomMessage>("odom-data", (event) => {
  // Update UI
});
```

## 🐛 Common Issues & Solutions

### Issue: Cargo can't find sensor_msgs
**Solution**: Run `make build-ros2` to build all ROS2 packages

### Issue: "yanked version" error
**Solution**: Add package to `[patch.crates-io]` in Cargo.toml

### Issue: White screen in UI
**Solution**: Refresh (Ctrl+R) or check browser console (F12)

### Issue: No images displayed
**Solution**: 
1. Verify topic: `ros2 topic hz /image/compressed`
2. Run image_transport republish if needed

### Issue: Permission denied
**Solution**: You must be inside Docker container as root

## 📁 File Structure

```
/root/
├── src/                     # Frontend (React/TS)
│   ├── App.tsx             # Main UI with ROS2 listeners
│   ├── App.css             # Styles
│   └── main.tsx            # React entry
├── src-tauri/              # Backend (Rust)
│   ├── src/lib.rs         # ROS2 subscribers & Tauri setup
│   ├── Cargo.toml         # Dependencies with ROS2 paths
│   └── tauri.conf.json    # Tauri config
├── ros2_ws/                # ROS2 workspace
│   ├── src/               # ros2_rust source
│   ├── install/           # Built packages (referenced by Cargo.toml)
│   └── build/             # Build artifacts
├── Makefile                # Build shortcuts
├── docker-compose.yml      # Docker setup
└── Dockerfile              # Container image
```

## 🎯 Testing

### Test Text Messages
```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello!'" --rate 1
```

### Test Camera Feed
```bash
# Publish from webcam
ros2 run image_tools cam2image --ros-args -r image:=/image

# Compress for /image/compressed
ros2 run image_transport republish raw compressed \
  --ros-args -r in:=/image -r out/compressed:=/image/compressed
```

### View All Topics
```bash
ros2 topic list
ros2 topic hz /image/compressed
ros2 topic info /image/compressed -v
```

## 📚 References

- [ros2-rust GitHub](https://github.com/ros2-rust/ros2_rust)
- [rclrs docs](https://docs.rs/rclrs/)
- [Tauri docs](https://tauri.app/)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/)

## ✨ Success Indicators

When everything is working:
1. `make dev` starts without errors
2. GUI window opens showing camera and message viewers
3. Published messages appear in real-time
4. Camera feed displays live video
5. No Cargo dependency errors

---

**Status**: Production Ready 🚀
**Last Updated**: 2025-12-12
