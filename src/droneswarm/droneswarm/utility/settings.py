#!/usr/bin/env python3

# Timer callback DTs
COPTER_CONTROL_LOOP_DT = 0.1 # 10Hz
PREARM_LOOP_DT = 2.0 # 0.5Hz
VELOCITY_STREAM_LOOP_DT = 0.05 # 20Hz

TAKEOFF_ALT = 5.0 # In meters, OSD

MAX_STALL_CYCLES_WATCHDOG = 10 # Number of cycles with no new velocity commands before watchdog triggers


BBOX_EDGE_LENGTH = 100  # NOTE: Very important. This value is what control how close or how far the UAV will fly related to the object. This is something we have to play with 