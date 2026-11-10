# SPDX-License-Identifier: Apache-2.0
import math

import sgs_core


def test_import_and_step_pitch():
    assert sgs_core.default_gains().boresight.kp > 0.0
    n = sgs_core.trace_len_after_step_pitch(1)
    assert n > 500
    max_err = sgs_core.run_step_pitch_max_error(1, 3.0)
    assert math.isfinite(max_err)
    assert max_err < math.radians(0.5)
