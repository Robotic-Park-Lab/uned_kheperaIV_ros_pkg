# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from uned_kheperaiv_driver.pid_controller import PIDController


def test_proportional_response():
    pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0, Td=0.0, Nd=100,
                        UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=0.0)
    pid.error[0] = 1.0
    out = pid.update(dt=0.01)
    assert out == 2.0


def test_saturation_upper_and_lower_limit():
    pid = PIDController(Kp=100.0, Ki=0.0, Kd=0.0, Td=0.0, Nd=100,
                        UpperLimit=5.0, LowerLimit=-5.0, ai=0.0, co=0.0)
    pid.error[0] = 10.0
    assert pid.update(dt=0.01) == 5.0

    pid.error[0] = -10.0
    assert pid.update(dt=0.01) == -5.0


def test_zero_upper_limit_disables_saturation():
    # A real quirk of this controller, not a bug: UpperLimit == 0.0 means
    # "no saturation", not "clamp to zero" -- verified against the actual
    # update() logic (`if not self.UpperLimit == 0.0:`), same in both
    # uned_kheperaiv_driver and uned_kheperaiv_webots before this was
    # extracted into one shared module.
    pid = PIDController(Kp=100.0, Ki=0.0, Kd=0.0, Td=0.0, Nd=100,
                        UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=0.0)
    pid.error[0] = 10.0
    assert pid.update(dt=0.01) == 1000.0


def test_integral_uses_previous_error_not_current():
    # update() integrates self.error[1] (the error from the previous call),
    # not self.error[0] (the one just set) -- real behavior, checked
    # explicitly so a future refactor doesn't silently "fix" it.
    pid = PIDController(Kp=0.0, Ki=1.0, Kd=0.0, Td=0.0, Nd=100,
                        UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=0.0)
    pid.error[0] = 5.0
    pid.error[1] = 2.0
    out = pid.update(dt=1.0)
    assert out == 2.0


def test_eval_threshold_triggers_on_large_delta():
    pid = PIDController(Kp=0.0, Ki=0.0, Kd=0.0, Td=0.0, Nd=100,
                        UpperLimit=0.0, LowerLimit=0.0, ai=0.05, co=0.01)
    # First call establishes trigger_last_signal == 0.0 (no previous signal)
    triggered_first = pid.eval_threshold(signal=0.0, ref=0.0)
    assert triggered_first is False

    # A big jump should exceed the threshold and trigger
    triggered = pid.eval_threshold(signal=10.0, ref=0.0)
    assert triggered is True


def test_eval_threshold_does_not_trigger_on_tiny_delta():
    pid = PIDController(Kp=0.0, Ki=0.0, Kd=0.0, Td=0.0, Nd=100,
                        UpperLimit=0.0, LowerLimit=0.0, ai=0.05, co=1.0)
    triggered = pid.eval_threshold(signal=0.001, ref=0.0)
    assert triggered is False
