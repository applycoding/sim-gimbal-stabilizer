// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from "vite";

export default defineConfig({
  base: "/sim-gimbal-stabilizer/",
  publicDir: "public",
  build: {
    outDir: "dist",
    emptyOutDir: true,
  },
});
