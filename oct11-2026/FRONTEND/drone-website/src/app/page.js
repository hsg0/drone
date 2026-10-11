"use client";

import { useState } from "react";

export default function Home() {
  const [playing, setPlaying] = useState(false);
  const [cameraOn, setCameraOn] = useState(false);

  return (
    <main className="flex flex-1 flex-col items-center justify-center gap-6 p-8">
      <h1 className="text-3xl font-semibold tracking-tight">Drone</h1>
      <button
        type="button"
        onClick={() => setPlaying(true)}
        className="rounded-full bg-foreground px-5 py-3 text-background"
      >
        Play video
      </button>
      <button
        type="button"
        onClick={() => setCameraOn((open) => !open)}
        className="rounded-full border border-foreground px-5 py-3"
      >
        {cameraOn ? "Close camera" : "Open camera"}
      </button>
      {playing ? (
        <img
          src="/backend/video"
          alt="Video from the backend"
          className="max-h-[70vh] max-w-full"
        />
      ) : null}
      {cameraOn ? (
        <img
          src="/backend/camera"
          alt="Raspberry Pi camera"
          className="max-h-[70vh] max-w-full"
        />
      ) : null}
    </main>
  );
}
