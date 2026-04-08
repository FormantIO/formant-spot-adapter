import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";

type ImmediateCallback = (...args: unknown[]) => void;

const globalWithImmediate = globalThis as typeof globalThis & {
  setImmediate?: (callback: ImmediateCallback, ...args: unknown[]) => number;
  clearImmediate?: (handle: number) => void;
};

if (
  typeof globalWithImmediate.setImmediate !== "function" ||
  typeof globalWithImmediate.clearImmediate !== "function"
) {
  const scheduled = new Map<number, number>();
  let nextHandle = 1;

  globalWithImmediate.setImmediate = (callback, ...args) => {
    const handle = nextHandle++;
    const timeoutId = window.setTimeout(() => {
      scheduled.delete(handle);
      callback(...args);
    }, 0);
    scheduled.set(handle, timeoutId);
    return handle;
  };

  globalWithImmediate.clearImmediate = (handle) => {
    const timeoutId = scheduled.get(handle);
    if (typeof timeoutId === "number") {
      window.clearTimeout(timeoutId);
      scheduled.delete(handle);
    }
  };
}

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
