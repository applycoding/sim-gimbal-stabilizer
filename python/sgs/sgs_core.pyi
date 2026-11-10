# SPDX-License-Identifier: Apache-2.0

class PidGains:
    kp: float
    ki: float
    kd: float

class Gains:
    rate: PidGains
    boresight: PidGains

def default_gains() -> Gains: ...
def default_limits() -> object: ...
def run_step_pitch_max_error(seed: int = ..., duration_s: float = ...) -> float: ...
def trace_len_after_step_pitch(seed: int = ...) -> int: ...
