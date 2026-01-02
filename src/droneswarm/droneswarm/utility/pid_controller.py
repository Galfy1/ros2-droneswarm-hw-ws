#!/usr/bin/env python3

class PID():
    def __init__(self, kp: float, ki: float, kd: float, output_limits = (float('-inf'), float('inf'))):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_limit_min = output_limits[0]
        self.output_limit_max = output_limits[1]

        self.proportional = 0.0
        self.integral = 0.0     # NOTE: accumulating term
        self.derivative = 0.0
        self.prev_error = 0.0
        self.first = True


    def update(self, error, dt):

        # Proportional
        proportional = self.kp * error

        # Derivative
        if self.first:
            derivative = 0.0
            self.first = False
        else:
            derivative = (error - self.prev_error) / dt

        if self.ki != 0.0:

            # Proposed integral
            proposed_integral = self.integral + error * dt

            # Proposed output BEFORE clamping
            raw_output_proposed = proportional + self.ki * proposed_integral + derivative * self.kd

            # Check for saturation
            is_saturated = (
                raw_output_proposed > self.output_limit_max or
                raw_output_proposed < self.output_limit_min
            )

            # Check if input error is pushing output further into saturation? 
            pushing_further = (
                (raw_output_proposed > self.output_limit_max and error > 0) or
                (raw_output_proposed < self.output_limit_min and error < 0)
            )

            # Integrator clamping (anti-windup)
            if not (is_saturated and pushing_further):
                self.integral = proposed_integral

            # Now compute ACTUAL output using final integral
            raw_output = proportional + self.ki * self.integral + derivative * self.kd

            # Clamp output
            output = max(min(raw_output, self.output_limit_max), self.output_limit_min)

        else:
            # No integral term
            raw_output = proportional + derivative * self.kd
            output = max(min(raw_output, self.output_limit_max), self.output_limit_min)

        self.prev_error = error
        return output
