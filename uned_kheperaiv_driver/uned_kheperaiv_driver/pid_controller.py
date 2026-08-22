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
PID controller shared by the physical Khepera IV driver
(kheperaIV_client_driver.py) and the virtual/Webots one (khepera_driver.py).

Extracted because it was byte-for-byte identical in both files (verified
with a real diff before extracting, not assumed). Pure logic, no ROS
dependency.
"""


class PIDController():
    def __init__(self, Kp, Ki, Kd, Td, Nd, UpperLimit, LowerLimit, ai, co):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Td = Td
        self.Nd = Nd
        self.UpperLimit = UpperLimit
        self.LowerLimit = LowerLimit
        self.integral = 0
        self.derivative = 0
        self.error = [0.0, 0.0]
        self.trigger_ai = ai
        self.trigger_co = co
        self.trigger_last_signal = 0.0
        self.noise = [0.0] * 20
        self.past_time = 0.0
        self.last_value = 0.0
        self.th = 0.0

    def update(self, dt):
        P = self.Kp * self.error[0]
        self.integral = self.integral + self.Ki * self.error[1] * dt
        self.derivative = (self.Td / (self.Td + self.Nd + dt)) * self.derivative + \
            (self.Kd * self.Nd / (self.Td + self.Nd * dt)) * (self.error[0] - self.error[1])
        out = P + self.integral + self.derivative

        if not self.UpperLimit == 0.0:
            if out > self.UpperLimit:
                out = self.UpperLimit
            if out < self.LowerLimit:
                out = self.LowerLimit

        self.error[1] = self.error[0]
        self.last_value = out

        return out

    def eval_threshold(self, signal, ref):
        mean = signal / len(self.noise)
        for i in range(0, len(self.noise) - 2):
            self.noise[i] = self.noise[i + 1]
            mean += self.noise[i] / len(self.noise)

        self.noise[len(self.noise) - 1] = signal

        trigger_cn = 0.0
        for i in range(0, len(self.noise) - 1):
            if abs(self.noise[i] - mean) > trigger_cn:
                trigger_cn = self.noise[i] - mean
        trigger_cn = 0.0

        a = self.trigger_ai * abs(signal - ref)
        if a > self.trigger_ai:
            a = self.trigger_ai

        self.th = self.trigger_co + a + trigger_cn
        self.inc = abs(abs(ref - signal) - self.trigger_last_signal)
        if self.inc >= abs(self.th):
            self.trigger_last_signal = abs(ref - signal)
            return True

        return False
