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

    def _proportional_block(self, error: float, dt: float):
        self.proportional = error * self.kp

    def _integral_block(self, error: float, dt: float):
        self.integral += error * dt

    def _derivative_block(self, error: float, dt: float):
        if self.first:
            self.derivative = 0.0
            self.first = False
        else:
            self.derivative = (error - self.prev_error) / dt

    
    def update(self, error: float, dt: float):

        self._proportional_block(error, dt)
        self._integral_block(error, dt)
        self._derivative_block(error, dt)

        

        self.prev_error = error 
        return result