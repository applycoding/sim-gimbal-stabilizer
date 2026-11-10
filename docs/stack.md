# Host stack usage (measured figure)

Measured on Ubuntu 24.04 with g++-13, Release build, `noheap_link` plus a 2000-step `Controller::step` bench covering both estimators, saturation, gating, and an 85° encoder sample.

| Item | Value |
| --- | --- |
| Tool | `valgrind --tool=massif` on `noheap_link` |
| Peak stack (approx.) | ~48 KiB |

This is a **host-only** measurement. It is not an MCU stack guarantee.
