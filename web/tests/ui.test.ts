// SPDX-License-Identifier: Apache-2.0
import { describe, expect, it } from "vitest";

import { clampSlider, joinBase } from "../src/ui";

describe("ui helpers", () => {
  it("joins base path", () => {
    expect(joinBase("/sim-gimbal-stabilizer/", "sgs.wasm")).toBe(
      "/sim-gimbal-stabilizer/sgs.wasm",
    );
  });
  it("clamps slider", () => {
    expect(clampSlider(99, 0, 50)).toBe(50);
  });
});
