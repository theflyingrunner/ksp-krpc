# ========================
# PID Controller
# A simple Proportional-Integral-Derivative controller
#
# Author: Saron Bhoopathy
# Date: August 18th, 2021
# Email: sbhoopat@purdue.edu
# Version: 1.2
# ========================

import time

class PID:

    def __init__(self, proportional_gain, integral_gain, derivative_gain):
        self.Kp = proportional_gain
        self.Ki = integral_gain
        self.Kd = derivative_gain
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.u = 0.0
        self.target_value = 0.0
        self.clamp = 1.0
        self.time_prev = time.time()
        self.value_prev = 0.0
        self.change_in_time = 1.0

    def update(self, value):
        """PID loop
        P = e(t)
        I = integral(0, t) {e(t) dt}
        D = d/dt {e(t)}
        u(t) = (Kp * P) + (Ki * I) + (Kd * D)
        """
        time_curr = time.time()                                 # current time
        delta_time = time_curr - self.time_prev                 # change in time

        error = self.target_value - value                       # error
        delta_error = value - self.value_prev
        self.P = error                                          # proportional(P) block
        self.I = self.I + (error * delta_time)                  # integral(I) block
        self.I = self.integralClamp(self.I)                     # integral(I) wind-up check
        self.D = delta_error / delta_time                       # derivative(D) block

        self.value_prev = value
        self.time_prev = time_curr

        self.u = (self.Kp * self.P) + (self.Ki * self.I) - (self.Kd * self.D)
        return self.u

    def integralClamp(self, I):
        """Anti-windup for integral block using the clamping method"""
        if I > self.clamp:
            return self.clamp
        elif I < -self.clamp:
            return -self.clamp
        else:
            return I

    def setTarget(self, target):
        self.target_value = target

    def setKp(self, proportional_gain):
        """Set proportional gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Set integral gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Set derivative gain"""
        self.Kd = derivative_gain

    def setClamp(self, clamping_value):
        self.clamp = clamping_value