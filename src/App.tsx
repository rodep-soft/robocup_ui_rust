import { useState, useEffect } from "react";
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import "./App.css";

interface ChatterMessage {
  data: string;
}

interface CompressedImageMessage {
  format: string;
  data_base64: string;
  width: number;
  height: number;
}

function App() {
  const [greetMsg, setGreetMsg] = useState("");
  const [name, setName] = useState("");
  const [chatterMessages, setChatterMessages] = useState<string[]>([]);
  const [currentImage, setCurrentImage] = useState<string | null>(null);
  const [imageFormat, setImageFormat] = useState<string>("");

  useEffect(() => {
    const unlistenChatter = listen<ChatterMessage>("chatter-message", (event) => {
      setChatterMessages((prev) => [...prev.slice(-9), event.payload.data]);
    });

    const unlistenImage = listen<CompressedImageMessage>("compressed-image", (event) => {
      const { format, data_base64 } = event.payload;
      setImageFormat(format);
      
      // Determine MIME type from format
      let mimeType = "image/jpeg"; // default
      if (format.includes("png")) {
        mimeType = "image/png";
      }
      
      setCurrentImage(`data:${mimeType};base64,${data_base64}`);
    });

    return () => {
      unlistenChatter.then((fn) => fn());
      unlistenImage.then((fn) => fn());
    };
  }, []);

  async function greet() {
    setGreetMsg(await invoke("greet", { name }));
  }

  return (
    <main className="container">
      <h1>🤖 Robot UI - ROS2 Integration</h1>

      {/* Compressed Image Display */}
      <div style={{ 
        marginTop: "2rem", 
        padding: "1.5rem", 
        border: "2px solid #4CAF50", 
        borderRadius: "12px",
        background: "rgba(76, 175, 80, 0.1)"
      }}>
        <h2 style={{ margin: "0 0 1rem 0", color: "#4CAF50" }}>
          📷 ROS2 /image/compressed
        </h2>
        <div style={{ 
          minHeight: "200px",
          background: "#0f0f0f", 
          borderRadius: "8px",
          border: "1px solid #333",
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          overflow: "hidden"
        }}>
          {currentImage ? (
            <img 
              src={currentImage} 
              alt="ROS2 Camera Feed" 
              style={{ 
                maxWidth: "100%", 
                maxHeight: "400px",
                objectFit: "contain"
              }} 
            />
          ) : (
            <p style={{ color: "#888", textAlign: "center", padding: "3rem" }}>
              ⏳ Waiting for images on /image/compressed...<br/>
              <small style={{ fontSize: "12px" }}>
                Format: {imageFormat || "Not received yet"}
              </small>
            </p>
          )}
        </div>
        {currentImage && (
          <div style={{ marginTop: "0.5rem", fontSize: "12px", color: "#888" }}>
            Format: {imageFormat}
          </div>
        )}
      </div>

      {/* Text Messages Display */}
      <div style={{ 
        marginTop: "2rem", 
        padding: "1.5rem", 
        border: "2px solid #646cff", 
        borderRadius: "12px",
        background: "rgba(100, 108, 255, 0.1)"
      }}>
        <h2 style={{ margin: "0 0 1rem 0", color: "#646cff" }}>
          📡 ROS2 /chatter Topic
        </h2>
        <div style={{ 
          minHeight: "150px",
          maxHeight: "300px", 
          overflow: "auto", 
          padding: "1rem", 
          background: "#0f0f0f", 
          borderRadius: "8px",
          border: "1px solid #333",
          fontFamily: "monospace",
          fontSize: "14px"
        }}>
          {chatterMessages.length === 0 ? (
            <p style={{ color: "#888", textAlign: "center", margin: "3rem 0" }}>
              ⏳ Waiting for messages on /chatter...<br/>
              <small style={{ fontSize: "12px" }}>
                Publish with: ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello!'"
              </small>
            </p>
          ) : (
            chatterMessages.map((msg, idx) => (
              <div key={idx} style={{ 
                padding: "0.5rem", 
                marginBottom: "0.5rem",
                borderLeft: "3px solid #646cff",
                paddingLeft: "0.75rem",
                background: "rgba(100, 108, 255, 0.05)"
              }}>
                <span style={{ color: "#61dafb", marginRight: "0.5rem" }}>
                  [{chatterMessages.length - chatterMessages.length + idx + 1}]
                </span>
                <span style={{ color: "#fff" }}>{msg}</span>
              </div>
            ))
          )}
        </div>
        <div style={{ marginTop: "0.5rem", fontSize: "12px", color: "#888" }}>
          Total messages: {chatterMessages.length}
        </div>
      </div>

      <form
        className="row"
        style={{ marginTop: "2rem" }}
        onSubmit={(e) => {
          e.preventDefault();
          greet();
        }}
      >
        <input
          id="greet-input"
          onChange={(e) => setName(e.currentTarget.value)}
          placeholder="Enter a name..."
        />
        <button type="submit">Greet</button>
      </form>
      <p>{greetMsg}</p>
    </main>
  );
}

export default App;
