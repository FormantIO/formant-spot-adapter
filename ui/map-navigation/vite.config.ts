import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { nodePolyfills } from "vite-plugin-node-polyfills";

export default defineConfig({
  base: "./",
  plugins: [
    react(),
    nodePolyfills({
      include: ["process", "buffer", "util", "timers", "events", "stream", "crypto", "path"],
      globals: {
        Buffer: true,
        process: true,
        setImmediate: true,
        clearImmediate: true
      },
      protocolImports: true
    })
  ],
  define: {
    "process.env.NODE_ENV": JSON.stringify(process.env.NODE_ENV || "development")
  },
  optimizeDeps: {
    include: ["@formant/data-sdk"]
  }
});
