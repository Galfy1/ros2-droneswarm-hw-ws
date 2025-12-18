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



    # def _integral_block(self, error: float, dt: float):
    #     self.integral += error * dt

    # def _derivative_block(self, error: float, dt: float):
    #     if self.first:
    #         self.derivative = 0.0
    #         self.first = False
    #     else:
    #         self.derivative = (error - self.prev_error) / dt



    # def update(self, error: float, dt: float):
        
    #     if self.kp != 0.0:
    #         self._proportional_block(error, dt)
    #     if self.ki != 0.0:
    #         self._integral_block(error, dt)
    #     if self.kd != 0.0:
    #         self._derivative_block(error, dt)

    #     # Compute raw (unclamped) output
    #     raw_output = (
    #         self.kp * error +
    #         self.ki * self.integral +
    #         self.kd * self.derivative
    #     )
        
    #     clamped_output = max(min(raw_output, self.output_limit_max), self.output_limit_min)

    #     # Anti-windup: Adjust integral term if output is saturated
    #     if self.ki != 0.0:
    #         is_saturated = clamped_output != raw_output
    #         same_sign = (error * raw_output) > 0.0
    #         if is_saturated and same_sign:
    #             # Remove the last integrated term
    #             self.integral -= error * dt

    #             # Recompute 
    #             raw_output = (
    #                 self.kp * error +
    #                 self.ki * self.integral +
    #                 self.kd * self.derivative
    #             )
    #             clamped_output = max(min(raw_output, self.output_limit_max), self.output_limit_min)

    #     self.prev_error = error 
    #     return clamped_output