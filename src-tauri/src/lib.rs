use tauri::Emitter;
use rclrs::CreateBasicExecutor;

#[derive(Clone, serde::Serialize)]
struct ChatterMessage {
    data: String,
}

#[derive(Clone, serde::Serialize)]
struct CompressedImageMessage {
    format: String,
    data_base64: String,
    width: u32,
    height: u32,
}

#[tauri::command]
fn greet(name: &str) -> String {
    format!("Hello, {}! You've been greeted from Rust!", name)
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    println!("Starting Tauri app with ROS2 /chatter subscription");

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![greet])
        .setup(|app| {
            let app_handle = app.handle().clone();
            
            // Spawn ROS2 subscription in background thread
            std::thread::spawn(move || {
                if let Err(e) = run_ros2_subscriber(app_handle) {
                    eprintln!("ROS2 subscriber error: {}", e);
                }
            });
            
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

fn run_ros2_subscriber(app_handle: tauri::AppHandle) -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("robot_ui_subscriber")?;
    
    // Subscribe to /chatter
    let worker1 = node.create_worker::<usize>(0);
    let app_handle_chatter = app_handle.clone();
    let _subscription_chatter = worker1.create_subscription::<std_msgs::msg::String, _>(
        "chatter",
        move |count: &mut usize, msg: std_msgs::msg::String| {
            *count += 1;
            println!("[{}] Received on /chatter: {}", *count, msg.data);
            
            let _ = app_handle_chatter.emit("chatter-message", ChatterMessage {
                data: msg.data.clone(),
            });
        },
    )?;
    
    // Subscribe to /image/compressed
    let worker2 = node.create_worker::<usize>(0);
    let _subscription_image = worker2.create_subscription::<sensor_msgs::msg::CompressedImage, _>(
        "image/compressed",
        move |count: &mut usize, msg: sensor_msgs::msg::CompressedImage| {
            *count += 1;
            println!("[{}] Received compressed image: {} bytes, format: {}", 
                     *count, msg.data.len(), msg.format);
            
            // Convert image data to base64
            use base64::{Engine as _, engine::general_purpose};
            let data_base64 = general_purpose::STANDARD.encode(&msg.data);
            
            let _ = app_handle.emit("compressed-image", CompressedImageMessage {
                format: msg.format.clone(),
                data_base64,
                width: 0,  // Not available in CompressedImage
                height: 0,
            });
        },
    )?;
    
    println!("Subscribed to /chatter and /image/compressed topics...");
    executor.spin(rclrs::SpinOptions::default());
    
    Ok(())
}
