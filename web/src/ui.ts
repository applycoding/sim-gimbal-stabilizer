// SPDX-License-Identifier: Apache-2.0
export function joinBase(base: string, path: string): string {
  const b = base.replace(/\/$/, "");
  const clean = path.replace(/^\//, "");
  return `${b}/${clean}`;
}

export function baseUrlJoin(path: string): string {
  const base = import.meta.env.BASE_URL ?? "/";
  return joinBase(base, path);
}

export function clampSlider(v: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, v));
}
