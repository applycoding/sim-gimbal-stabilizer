// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from "vitest/config";

export default defineConfig({
  base: "/sim-gimbal-stabilizer/",
  test: {
    include: ["tests/**/*.test.ts"],
    exclude: ["e2e/**"],
  },
});
