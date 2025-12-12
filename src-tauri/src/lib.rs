use std::sync::{Arc, Mutex};
use tauri::{Manager, Emitter};
use rclrs::CreateBasicExecutor;

#[derive(Clone, serde::Serialize)]
struct ChatterMessage {
    data: String,
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
    
    let worker = node.create_worker::<usize>(0);
    let _subscription = worker.create_subscription::<std_msgs::msg::String, _>(
        "chatter",
        move |count: &mut usize, msg: std_msgs::msg::String| {
            *count += 1;
            println!("[{}] Received on /chatter: {}", *count, msg.data);
            
            // Emit event to frontend
            let _ = app_handle.emit("chatter-message", ChatterMessage {
                data: msg.data.clone(),
            });
        },
    )?;
    
    println!("Subscribed to /chatter topic, waiting for messages...");
    executor.spin(rclrs::SpinOptions::default());
    
    
    Ok(())
}
