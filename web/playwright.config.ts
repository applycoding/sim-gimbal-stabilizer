// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./e2e",
  timeout: 120_000,
  use: {
    headless: true,
    baseURL: "http://127.0.0.1:4173",
  },
  webServer: {
    command: "npm run preview",
    url: "http://127.0.0.1:4173/sim-gimbal-stabilizer/",
    reuseExistingServer: !process.env.CI,
  },
});
