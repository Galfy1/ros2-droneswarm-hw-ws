#!/usr/bin/env python3


class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.proportional = 0.0
        self.integral = 0.0     # NOTE: accumulating term
        self.derivative = 0.0
        self.prev_error = 0.0
        self.first = True

    def _proportional_block(self, error: float):
        self.proportional = error * self.kp

    def _integral_block(self, error: float):
        self.integral += error * dt

    def _derivative_block(self, error: float):
        pass

    


    def update(self, error, dt):
        pass