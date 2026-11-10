// SPDX-License-Identifier: Apache-2.0
import { expect, test } from "@playwright/test";
import { mkdirSync } from "node:fs";
import path from "node:path";

test("wasm demo smoke", async ({ page, request }) => {
  await page.goto("/sim-gimbal-stabilizer/");
  await expect(page.locator("#status")).toHaveText(/ready/, { timeout: 90_000 });
  const slider = page.locator("#gain-att-kp");
  const before = await slider.inputValue();
  await slider.fill("15");
  await expect(page.getByText(/k_p=15/)).toBeVisible();
  expect(before).not.toBe("15");
  await page.locator("#btn-step").click();
  await page.waitForTimeout(500);
  const wasm = await request.get("/sim-gimbal-stabilizer/sgs.wasm");
  expect(wasm.headers()["content-type"] ?? "").toContain("application/wasm");
  mkdirSync("test-results", { recursive: true });
  await page.screenshot({ path: "test-results/demo.png", fullPage: true });
});
