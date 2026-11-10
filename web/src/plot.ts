// SPDX-License-Identifier: Apache-2.0
export function setupCanvas(canvas: HTMLCanvasElement): void {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * dpr;
  canvas.height = canvas.clientHeight * dpr;
}

export function plotSeries(canvas: HTMLCanvasElement, ys: number[]): void {
  const ctx = canvas.getContext("2d");
  if (!ctx || ys.length < 2) return;
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  let min = ys[0];
  let max = ys[0];
  for (const y of ys) {
    min = Math.min(min, y);
    max = Math.max(max, y);
  }
  const span = max - min || 1;
  ctx.strokeStyle = "#2563eb";
  ctx.beginPath();
  ys.forEach((y, i) => {
    const x = (i / (ys.length - 1)) * (w - 8) + 4;
    const py = h - ((y - min) / span) * (h - 8) - 4;
    if (i === 0) ctx.moveTo(x, py);
    else ctx.lineTo(x, py);
  });
  ctx.stroke();
}
