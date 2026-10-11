"use client";

import { useState, useEffect, useCallback } from "react";

const GESTURE_COMMAND_LIST = [
  {
    id: "takeoff-land",
    pose: "Wide arms overhead in V",
    action: "Take off (ground) / Land (flight)",
    icon: "🛫",
    color: "from-amber-500/20 to-yellow-500/20 border-yellow-500/40 text-yellow-300",
  },
  {
    id: "fly-up",
    pose: "Arms raised straight up",
    action: "Vertical climb up",
    icon: "⬆️",
    color: "from-emerald-500/20 to-teal-500/20 border-emerald-500/40 text-emerald-300",
  },
  {
    id: "fly-down",
    pose: "Both arms moving downward",
    action: "Vertical descend down",
    icon: "⬇️",
    color: "from-blue-500/20 to-indigo-500/20 border-blue-500/40 text-blue-300",
  },
  {
    id: "move-away",
    pose: "Arms out full width (wingspan)",
    action: "Fly farther back (reverse)",
    icon: "⬅️",
    color: "from-cyan-500/20 to-sky-500/20 border-cyan-500/40 text-cyan-300",
  },
  {
    id: "move-closer",
    pose: "Hands inward in front of face",
    action: "Fly closer (forward)",
    icon: "➡️",
    color: "from-violet-500/20 to-purple-500/20 border-purple-500/40 text-purple-300",
  },
  {
    id: "hover",
    pose: "Arms resting straight down",
    action: "Hold position & altitude",
    icon: "⏸️",
    color: "from-zinc-500/20 to-neutral-500/20 border-zinc-500/40 text-zinc-300",
  },
];

const VISION_EFFECTS = [
  { id: "", label: "Raw Color", desc: "Native BGR camera stream" },
  { id: "gesture", label: "Gesture AI", desc: "Pose detector + Yellow box & command" },
  { id: "yellow", label: "Yellow Box", desc: "HSV object isolation & crop" },
  { id: "gray", label: "Grayscale", desc: "Single channel luminance" },
  { id: "blur", label: "Box Blur", desc: "15px pixel averaging" },
  { id: "gaussian", label: "Gaussian", desc: "Smooth Gaussian kernel" },
];

export default function Home() {
  const [feedSource, setFeedSource] = useState("video"); // "video" | "camera" | "off"
  const [activeEffect, setActiveEffect] = useState("gesture");
  const [backendEndpoint, setBackendEndpoint] = useState("/backend");
  const [backendOnline, setBackendOnline] = useState(false);
  const [backendLatencyMs, setBackendLatencyMs] = useState(null);
  const [testFlightInfo, setTestFlightInfo] = useState(null);
  const [flightInfoLoading, setFlightInfoLoading] = useState(false);
  const [recordingState, setRecordingState] = useState("idle"); // "idle" | "recording" | "done" | "error"
  const [recordingDetails, setRecordingDetails] = useState(null);
  const [streamError, setStreamError] = useState(false);

  // Ping backend health and fetch flight route status
  const checkBackendStatus = useCallback(async () => {
    setFlightInfoLoading(true);
    const start = performance.now();
    try {
      const response = await fetch(`${backendEndpoint}/testflight`, {
        cache: "no-store",
      });
      const latency = Math.round(performance.now() - start);
      if (response.ok) {
        const data = await response.json();
        setBackendOnline(true);
        setBackendLatencyMs(latency);
        setTestFlightInfo(data);
      } else {
        setBackendOnline(false);
        setBackendLatencyMs(null);
      }
    } catch {
      setBackendOnline(false);
      setBackendLatencyMs(null);
    } finally {
      setFlightInfoLoading(false);
    }
  }, [backendEndpoint]);

  useEffect(() => {
    checkBackendStatus();
    const interval = setInterval(checkBackendStatus, 10000);
    return () => clearInterval(interval);
  }, [checkBackendStatus]);

  // Record gesture training clip (3s @ 10fps on Pi camera)
  const handleRecordGesture = async () => {
    try {
      setRecordingState("recording");
      setRecordingDetails(null);
      const response = await fetch(`${backendEndpoint}/camera/record`, {
        method: "POST",
      });
      const data = await response.json();
      if (!response.ok) {
        throw new Error(data.detail || "Recording failed");
      }
      setRecordingState("done");
      setRecordingDetails(data);
    } catch (err) {
      setRecordingState("error");
      setRecordingDetails({ error: err.message });
    }
  };

  const effectSuffix = activeEffect ? `/${activeEffect}` : "";
  const streamUrl =
    feedSource === "off"
      ? null
      : `${backendEndpoint}/${feedSource}${effectSuffix}`;

  return (
    <div className="min-h-screen bg-neutral-950 text-neutral-100 flex flex-col font-sans selection:bg-cyan-500/30 selection:text-cyan-200">
      {/* Top Navigation & Status Bar */}
      <header className="border-b border-neutral-800 bg-neutral-900/60 backdrop-blur-md sticky top-0 z-30">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 h-16 flex items-center justify-between gap-4">
          <div className="flex items-center gap-3">
            <div className="w-9 h-9 rounded-lg bg-gradient-to-tr from-cyan-500 to-blue-600 flex items-center justify-center font-bold text-white shadow-lg shadow-cyan-500/20 text-lg">
              ✕
            </div>
            <div>
              <div className="flex items-center gap-2">
                <h1 className="font-bold tracking-wider uppercase text-sm sm:text-base text-neutral-50">
                  Drone Vision & Flight Deck
                </h1>
                <span className="text-[10px] font-mono px-1.5 py-0.5 rounded bg-cyan-950/80 text-cyan-400 border border-cyan-800/60 uppercase">
                  v1.0
                </span>
              </div>
              <p className="text-xs text-neutral-400 font-mono hidden sm:block">
                Quad X Computer Vision & Gesture Control
              </p>
            </div>
          </div>

          <div className="flex items-center gap-3">
            {/* Host endpoint switcher */}
            <div className="hidden md:flex items-center gap-1.5 bg-neutral-900 border border-neutral-800 rounded-lg p-1 text-xs">
              <span className="text-neutral-500 px-2 font-mono">Target:</span>
              <button
                type="button"
                onClick={() => setBackendEndpoint("/backend")}
                className={`px-2.5 py-1 rounded font-mono transition-all ${
                  backendEndpoint === "/backend"
                    ? "bg-neutral-800 text-cyan-300 font-semibold"
                    : "text-neutral-400 hover:text-neutral-200"
                }`}
              >
                Proxy (/backend)
              </button>
              <button
                type="button"
                onClick={() => setBackendEndpoint("http://192.168.0.138:8000")}
                className={`px-2.5 py-1 rounded font-mono transition-all ${
                  backendEndpoint === "http://192.168.0.138:8000"
                    ? "bg-neutral-800 text-cyan-300 font-semibold"
                    : "text-neutral-400 hover:text-neutral-200"
                }`}
              >
                Pi (192.168.0.138)
              </button>
              <button
                type="button"
                onClick={() => setBackendEndpoint("http://127.0.0.1:8000")}
                className={`px-2.5 py-1 rounded font-mono transition-all ${
                  backendEndpoint === "http://127.0.0.1:8000"
                    ? "bg-neutral-800 text-cyan-300 font-semibold"
                    : "text-neutral-400 hover:text-neutral-200"
                }`}
              >
                Local (127.0.0.1)
              </button>
            </div>

            {/* Connection badge */}
            <button
              type="button"
              onClick={checkBackendStatus}
              title="Click to re-ping backend"
              className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-neutral-900 border border-neutral-800 text-xs font-mono hover:border-neutral-700 transition"
            >
              <span
                className={`w-2 h-2 rounded-full ${
                  backendOnline
                    ? "bg-emerald-400 shadow-sm shadow-emerald-400 animate-pulse"
                    : "bg-rose-500 shadow-sm shadow-rose-500"
                }`}
              />
              <span className={backendOnline ? "text-emerald-300" : "text-rose-400"}>
                {backendOnline ? "BACKEND LIVE" : "BACKEND OFFLINE"}
              </span>
              {backendLatencyMs !== null && (
                <span className="text-neutral-500 text-[10px] hidden sm:inline">
                  {backendLatencyMs}ms
                </span>
              )}
            </button>
          </div>
        </div>
      </header>

      {/* Main Workspace Layout */}
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-6 w-full flex-1 grid grid-cols-1 lg:grid-cols-12 gap-6">
        {/* Left Column: Vision Viewport & Live Stream (8 cols on lg) */}
        <div className="lg:col-span-8 flex flex-col gap-6">
          {/* Main Monitor Card */}
          <div className="bg-neutral-900/70 border border-neutral-800 rounded-2xl overflow-hidden shadow-2xl flex flex-col">
            {/* Viewport Top Bar */}
            <div className="px-4 py-3 bg-neutral-900/90 border-b border-neutral-800 flex items-center justify-between text-xs font-mono">
              <div className="flex items-center gap-3">
                <span className="flex items-center gap-1.5 text-neutral-300">
                  <span
                    className={`w-2 h-2 rounded-full ${
                      feedSource === "camera"
                        ? "bg-rose-500 animate-ping"
                        : feedSource === "video"
                        ? "bg-cyan-400"
                        : "bg-neutral-600"
                    }`}
                  />
                  FEED:{" "}
                  <strong className="text-white uppercase">
                    {feedSource === "off" ? "STANDBY" : feedSource}
                  </strong>
                </span>
                <span className="text-neutral-600">|</span>
                <span className="text-neutral-400">RES: 640×480</span>
                <span className="text-neutral-600">|</span>
                <span className="text-cyan-400 uppercase font-semibold">
                  FX: {activeEffect || "NONE (COLOR)"}
                </span>
              </div>

              {/* Feed Switchers */}
              <div className="flex items-center gap-1 bg-neutral-950 p-1 rounded-lg border border-neutral-800">
                <button
                  type="button"
                  onClick={() => {
                    setFeedSource("video");
                    setStreamError(false);
                  }}
                  className={`px-3 py-1 rounded text-xs transition-all ${
                    feedSource === "video"
                      ? "bg-cyan-500 text-black font-semibold shadow"
                      : "text-neutral-400 hover:text-white"
                  }`}
                >
                  Test Video
                </button>
                <button
                  type="button"
                  onClick={() => {
                    setFeedSource("camera");
                    setStreamError(false);
                  }}
                  className={`px-3 py-1 rounded text-xs transition-all ${
                    feedSource === "camera"
                      ? "bg-rose-500 text-white font-semibold shadow"
                      : "text-neutral-400 hover:text-white"
                  }`}
                >
                  Pi Camera
                </button>
                <button
                  type="button"
                  onClick={() => setFeedSource("off")}
                  className={`px-2.5 py-1 rounded text-xs transition-all ${
                    feedSource === "off"
                      ? "bg-neutral-800 text-neutral-300 font-semibold"
                      : "text-neutral-500 hover:text-neutral-300"
                  }`}
                >
                  Mute
                </button>
              </div>
            </div>

            {/* Video Canvas Container */}
            <div className="relative aspect-video sm:aspect-[4/3] max-h-[580px] bg-black flex items-center justify-center overflow-hidden">
              {/* Drone Crosshairs / HUD Overlay */}
              <div className="absolute inset-0 pointer-events-none z-10 flex flex-col justify-between p-4">
                <div className="flex justify-between items-start text-[10px] font-mono text-cyan-400/70">
                  <div className="border-t-2 border-l-2 border-cyan-400/60 w-6 h-6" />
                  <div className="bg-black/40 px-2 py-0.5 rounded backdrop-blur">
                    OPTICAL DETECTOR // ACTIVE
                  </div>
                  <div className="border-t-2 border-r-2 border-cyan-400/60 w-6 h-6" />
                </div>

                {/* Center target crosshair */}
                <div className="self-center flex items-center justify-center opacity-30">
                  <div className="w-12 h-12 border border-cyan-300 rounded-full flex items-center justify-center">
                    <div className="w-1.5 h-1.5 bg-cyan-300 rounded-full" />
                  </div>
                </div>

                <div className="flex justify-between items-end text-[10px] font-mono text-cyan-400/70">
                  <div className="border-b-2 border-l-2 border-cyan-400/60 w-6 h-6" />
                  <div className="bg-black/40 px-2 py-0.5 rounded backdrop-blur">
                    QUAD_X // OV5647
                  </div>
                  <div className="border-b-2 border-r-2 border-cyan-400/60 w-6 h-6" />
                </div>
              </div>

              {/* Stream Image */}
              {streamUrl && !streamError ? (
                <img
                  key={streamUrl}
                  src={streamUrl}
                  alt="Drone computer vision live stream"
                  onError={() => setStreamError(true)}
                  className="w-full h-full object-contain"
                />
              ) : (
                <div className="flex flex-col items-center justify-center p-8 text-center text-neutral-500">
                  <div className="w-14 h-14 rounded-full border border-dashed border-neutral-700 flex items-center justify-center mb-3 text-2xl">
                    {streamError ? "⚠️" : "📺"}
                  </div>
                  <h3 className="font-mono text-sm text-neutral-300">
                    {streamError
                      ? "Stream Unavailable from Endpoint"
                      : "Feed is in Standby"}
                  </h3>
                  <p className="text-xs text-neutral-500 max-w-sm mt-1">
                    {streamError ? (
                      feedSource === "camera" ? (
                        <>
                          The Raspberry Pi camera is only accessible when the
                          backend runs directly on the Pi (at{" "}
                          <code className="text-cyan-400">192.168.0.138:8000</code>).
                          Switch to <strong>Test Video</strong> to preview the recorded
                          gesture footage locally.
                        </>
                      ) : (
                        <>
                          Make sure the backend is running with{" "}
                          <code className="text-cyan-400 font-mono">
                            python main.py
                          </code>{" "}
                          on port 8000.
                        </>
                      )
                    ) : (
                      "Select 'Test Video' or 'Pi Camera' above to start real-time telemetry."
                    )}
                  </p>
                  {streamError && (
                    <button
                      type="button"
                      onClick={() => setStreamError(false)}
                      className="mt-3 px-3 py-1 bg-neutral-800 hover:bg-neutral-700 text-xs font-mono text-neutral-200 rounded"
                    >
                      Retry Connection
                    </button>
                  )}
                </div>
              )}
            </div>

            {/* Viewport Bottom Controls: CV Filters */}
            <div className="p-4 bg-neutral-900 border-t border-neutral-800 flex flex-col gap-3">
              <div className="flex items-center justify-between text-xs font-mono text-neutral-400">
                <span className="font-semibold text-neutral-200 uppercase tracking-wider">
                  Vision Filters &amp; Neural Pipeline
                </span>
                <span>Select active pipeline modifier</span>
              </div>
              <div className="grid grid-cols-2 sm:grid-cols-3 md:grid-cols-6 gap-2">
                {VISION_EFFECTS.map((eff) => {
                  const isActive = activeEffect === eff.id;
                  return (
                    <button
                      key={eff.id}
                      type="button"
                      onClick={() => {
                        setActiveEffect(eff.id);
                        setStreamError(false);
                      }}
                      className={`flex flex-col items-start p-2.5 rounded-xl border text-left transition-all ${
                        isActive
                          ? "bg-cyan-950/60 border-cyan-500 text-cyan-200 shadow-md shadow-cyan-900/30 ring-1 ring-cyan-500/50"
                          : "bg-neutral-950/70 border-neutral-800 text-neutral-400 hover:border-neutral-700 hover:text-neutral-200"
                      }`}
                    >
                      <span className="text-xs font-bold font-mono">
                        {eff.label}
                      </span>
                      <span className="text-[10px] text-neutral-500 leading-tight mt-0.5 line-clamp-1">
                        {eff.desc}
                      </span>
                    </button>
                  );
                })}
              </div>
            </div>
          </div>

          {/* Dataset Capture Card */}
          <div className="bg-neutral-900/70 border border-neutral-800 rounded-2xl p-5 flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4">
            <div>
              <div className="flex items-center gap-2">
                <span className="text-xl">📹</span>
                <h3 className="font-bold text-sm text-neutral-100 uppercase tracking-wide">
                  Media Dataset Recorder
                </h3>
              </div>
              <p className="text-xs text-neutral-400 mt-1 max-w-lg">
                Records a 3-second live gesture clip (10 FPS) from the Pi
                camera, stores the MP4 in <code className="text-neutral-300 font-mono">media/videos</code>,
                and extracts labeled frames into <code className="text-neutral-300 font-mono">media/pictures</code>.
              </p>
            </div>

            <div className="flex flex-col sm:items-end w-full sm:w-auto">
              <button
                type="button"
                onClick={handleRecordGesture}
                disabled={recordingState === "recording"}
                className={`px-5 py-2.5 rounded-xl font-medium text-xs tracking-wider uppercase transition-all shadow-lg flex items-center justify-center gap-2 w-full sm:w-auto ${
                  recordingState === "recording"
                    ? "bg-amber-600 text-white cursor-wait animate-pulse"
                    : "bg-gradient-to-r from-red-600 to-rose-600 hover:from-red-500 hover:to-rose-500 text-white shadow-rose-900/30"
                }`}
              >
                <span className="w-2 h-2 rounded-full bg-white animate-ping" />
                {recordingState === "recording"
                  ? "Capturing (3s)..."
                  : "Record New Gesture"}
              </button>

              {recordingDetails && (
                <div className="mt-2 text-[11px] font-mono text-neutral-300 bg-neutral-950 px-3 py-1.5 rounded-lg border border-neutral-800 max-w-sm">
                  {recordingDetails.error ? (
                    <span className="text-rose-400">
                      Error: {recordingDetails.error}
                    </span>
                  ) : (
                    <span className="text-emerald-400">
                      ✓ Saved {recordingDetails.frameCount} frames to{" "}
                      <span className="underline">
                        {recordingDetails.pictures?.split("/").pop()}
                      </span>
                    </span>
                  )}
                </div>
              )}
            </div>
          </div>
        </div>

        {/* Right Column: Flight Commands & Drone Deck (4 cols on lg) */}
        <div className="lg:col-span-4 flex flex-col gap-6">
          {/* Test Flight Status Card */}
          <div className="bg-neutral-900/70 border border-neutral-800 rounded-2xl p-5 flex flex-col gap-4 shadow-xl">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2">
                <span className="text-xl">🛸</span>
                <div>
                  <h3 className="font-bold text-sm text-neutral-100 uppercase tracking-wide">
                    Flight Controller Bridge
                  </h3>
                  <span className="text-[10px] font-mono text-neutral-400">
                    Endpoint: <code className="text-cyan-400">/testflight</code>
                  </span>
                </div>
              </div>

              <span
                className={`text-[10px] font-mono px-2 py-0.5 rounded-full font-bold uppercase ${
                  testFlightInfo?.armed
                    ? "bg-rose-500/20 text-rose-300 border border-rose-500/40"
                    : "bg-emerald-500/20 text-emerald-300 border border-emerald-500/40"
                }`}
              >
                {testFlightInfo?.armed ? "ARMED" : "DISARMED"}
              </span>
            </div>

            <p className="text-xs text-neutral-400 leading-relaxed">
              Synchronizes with the Pixhawk autopilot via MAVLink pins 8 &amp; 10.
              Gestures recognized by MediaPipe are converted to discrete flight
              maneuvers once held steadily for 1 second.
            </p>

            <button
              type="button"
              onClick={checkBackendStatus}
              disabled={flightInfoLoading}
              className="w-full py-2 px-3 rounded-xl bg-neutral-800 hover:bg-neutral-700 text-xs font-mono font-medium text-neutral-200 transition border border-neutral-700 flex items-center justify-center gap-2"
            >
              <span className={flightInfoLoading ? "animate-spin" : ""}>🔄</span>
              {flightInfoLoading ? "Querying..." : "Refresh Flight Status"}
            </button>
          </div>

          {/* Gesture Recognition Command Matrix */}
          <div className="bg-neutral-900/70 border border-neutral-800 rounded-2xl p-5 flex flex-col gap-4 shadow-xl flex-1">
            <div className="flex items-center justify-between border-b border-neutral-800 pb-3">
              <h3 className="font-bold text-sm text-neutral-100 uppercase tracking-wide flex items-center gap-2">
                <span>🤖</span>
                <span>Recognized Poses</span>
              </h3>
              <span className="text-[10px] font-mono text-neutral-500">
                6 Maneuvers
              </span>
            </div>

            <div className="flex flex-col gap-2.5 overflow-y-auto max-h-[460px] pr-1">
              {GESTURE_COMMAND_LIST.map((cmd) => (
                <div
                  key={cmd.id}
                  className={`p-3 rounded-xl border bg-gradient-to-r flex items-start gap-3 transition-all hover:scale-[1.01] ${cmd.color}`}
                >
                  <span className="text-xl mt-0.5 select-none">{cmd.icon}</span>
                  <div className="flex-1 min-w-0">
                    <div className="flex items-center justify-between gap-2">
                      <h4 className="text-xs font-bold font-mono truncate">
                        {cmd.action}
                      </h4>
                      <span className="text-[9px] font-mono uppercase px-1.5 py-0.2 rounded bg-black/40 border border-white/10 text-white/80 shrink-0">
                        {cmd.id}
                      </span>
                    </div>
                    <p className="text-[11px] text-neutral-400 mt-0.5 leading-snug">
                      Pose: <span className="text-neutral-200">{cmd.pose}</span>
                    </p>
                  </div>
                </div>
              ))}
            </div>

            <div className="pt-2 border-t border-neutral-800/80 text-[11px] text-neutral-500 font-mono">
              💡 Tip: Activate <strong className="text-cyan-400">Gesture AI</strong> above to see
              the yellow tracking box and real-time command classification on your video feed.
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
