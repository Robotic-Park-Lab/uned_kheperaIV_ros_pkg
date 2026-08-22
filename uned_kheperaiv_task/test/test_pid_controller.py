# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Tests for pid_controller.PIDController.

Extracted verbatim from gazebo_driver.py's IPC inner-loop controller.
"""

from uned_kheperaiv_task.pid_controller import PIDController


def _pid(Kp=0.0, Ki=0.0, Kd=0.0, Td=0.0, Nd=100, UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=0.0):
    return PIDController(Kp, Ki, Kd, Td, Nd, UpperLimit, LowerLimit, ai, co)


def test_proportional_only_output():
    pid = _pid(Kp=2.0)
    pid.error[0] = 3.0
    out = pid.update(dt=0.1)
    assert out == 6.0  # Kp * error, no I/D term, no saturation (UpperLimit == 0.0 disables it)


def test_saturates_at_upper_and_lower_limit():
    pid = _pid(Kp=100.0, UpperLimit=1.0, LowerLimit=-1.0)
    pid.error[0] = 5.0
    assert pid.update(dt=0.1) == 1.0

    pid2 = _pid(Kp=100.0, UpperLimit=1.0, LowerLimit=-1.0)
    pid2.error[0] = -5.0
    assert pid2.update(dt=0.1) == -1.0


def test_no_saturation_when_upper_limit_is_zero():
    pid = _pid(Kp=100.0)
    pid.error[0] = 5.0
    assert pid.update(dt=0.1) == 500.0


def test_integral_term_accumulates_the_previous_error():
    pid = _pid(Ki=1.0)
    pid.error[0] = 2.0
    first = pid.update(dt=1.0)
    assert first == 0.0  # error[1] (the previous error) is still 0.0 on the first call
    pid.error[0] = 2.0
    second = pid.update(dt=1.0)
    assert second == 2.0  # now integrates the error[1] left behind by the first call


def test_eval_threshold_true_when_delta_exceeds_threshold():
    pid = _pid(ai=0.05, co=0.01)
    assert pid.eval_threshold(error=1.0) is True  # first call: delta = 1.0 vs th ~= 0.06


def test_eval_threshold_false_when_delta_within_threshold():
    pid = _pid(ai=0.05, co=0.01)
    pid.eval_threshold(error=1.0)  # primes trigger_last_signal to 1.0
    assert pid.eval_threshold(error=1.0) is False  # no change since last trigger
