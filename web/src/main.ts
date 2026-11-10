// SPDX-License-Identifier: Apache-2.0
import { baseUrlJoin } from "./ui";
import { plotSeries, setupCanvas } from "./plot";

declare global {
  interface Window {
    createSgsModule?: (config: { locateFile: (p: string) => string }) => Promise<SgsModule>;
  }
}

export interface SgsModule {
  cwrap: (
    name: string,
    ret: string | null,
    arg: string[],
  ) => (...args: number[]) => number;
  _sgs_trace_ptr: () => number;
  HEAPF32: Float32Array;
}

const gainPitch = document.querySelector<HTMLInputElement>("#gain-att-kp")!;
const gainRate = document.querySelector<HTMLInputElement>("#gain-rate-kp")!;
const btnStep = document.querySelector<HTMLButtonElement>("#btn-step")!;
const statusEl = document.querySelector<HTMLSpanElement>("#status")!;
const errCanvas = document.querySelector<HTMLCanvasElement>("#err-plot")!;

let mod: SgsModule | null = null;
let traceLen = 0;

function clamp(v: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, v));
}

export function readGain(): number {
  return clamp(Number(gainPitch.value), 0, 50);
}

async function init(): Promise<void> {
  const url = baseUrlJoin("sgs.js");
  const m = (await import(/* @vite-ignore */ url)) as {
    default?: Window["createSgsModule"];
    createSgsModule?: Window["createSgsModule"];
  };
  const create = m.default ?? m.createSgsModule;
  if (!create) {
    throw new Error("WASM glue missing createSgsModule");
  }
  mod = await create({
    locateFile: (p: string) => baseUrlJoin(p),
  });
  const reset = mod.cwrap("sgs_reset", null, []);
  reset();
  statusEl.textContent = "ready";
  setupCanvas(errCanvas);
  tick();
}

function tick(): void {
  if (!mod) return;
  const setGains = mod.cwrap("sgs_set_gains", null, ["number", "number", "number", "number"]);
  setGains(Number(gainRate.value), 0.4, readGain(), 2.0);
  const step = mod.cwrap("sgs_step_ms", "number", ["number"]);
  traceLen = step(16);
  const ptr = mod._sgs_trace_ptr();
  const stride = 5;
  const ys: number[] = [];
  for (let i = Math.max(0, traceLen - 200); i < traceLen; i++) {
    ys.push(mod.HEAPF32[(ptr >> 2) + i * stride + 1]);
  }
  plotSeries(errCanvas, ys);
  requestAnimationFrame(tick);
}

gainPitch.addEventListener("input", () => {
  statusEl.textContent = `k_p=${readGain().toFixed(2)}`;
});

btnStep.addEventListener("click", () => {
  if (!mod) return;
  const pitchStep = mod.cwrap("sgs_pitch_step", null, []);
  pitchStep();
});

init().catch((e) => {
  statusEl.textContent = `error: ${e}`;
});
