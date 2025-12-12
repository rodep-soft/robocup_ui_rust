import { useState, useEffect } from "react";
import reactLogo from "./assets/react.svg";
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import "./App.css";

interface ChatterMessage {
  data: string;
}

function App() {
  const [greetMsg, setGreetMsg] = useState("");
  const [name, setName] = useState("");
  const [chatterMessages, setChatterMessages] = useState<string[]>([]);

  useEffect(() => {
    const unlisten = listen<ChatterMessage>("chatter-message", (event) => {
      setChatterMessages((prev) => [...prev.slice(-9), event.payload.data]);
    });

    return () => {
      unlisten.then((fn) => fn());
    };
  }, []);

  async function greet() {
    setGreetMsg(await invoke("greet", { name }));
  }

  return (
    <main className="container">
      <h1>Robot UI - ROS2 Integration</h1>

      <div className="row">
        <a href="https://vitejs.dev" target="_blank">
          <img src="/vite.svg" className="logo vite" alt="Vite logo" />
        </a>
        <a href="https://tauri.app" target="_blank">
          <img src="/tauri.svg" className="logo tauri" alt="Tauri logo" />
        </a>
        <a href="https://reactjs.org" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>

      <div style={{ marginTop: "2rem", padding: "1rem", border: "1px solid #ccc", borderRadius: "8px" }}>
        <h2>ROS2 /chatter Topic</h2>
        <div style={{ maxHeight: "200px", overflow: "auto", padding: "0.5rem", background: "#1a1a1a", borderRadius: "4px" }}>
          {chatterMessages.length === 0 ? (
            <p style={{ color: "#888" }}>Waiting for messages on /chatter...</p>
          ) : (
            chatterMessages.map((msg, idx) => (
              <div key={idx} style={{ padding: "0.25rem", borderBottom: "1px solid #333" }}>
                {msg}
              </div>
            ))
          )}
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
