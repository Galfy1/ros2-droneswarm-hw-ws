#!/usr/bin/env python3

import droneswarm.utility.constants as const

# Timer callback DTs
COPTER_CONTROL_LOOP_DT = 0.1 # 10Hz
PREARM_LOOP_DT = 2.0 # 0.5Hz
VELOCITY_STREAM_LOOP_DT = 0.05 # 20Hz

TAKEOFF_ALT = 5.0 # In meters, OSD

MAX_STALL_CYCLES_WATCHDOG = 10 # Number of cycles with no new velocity commands before watchdog triggers


BBOX_EDGE_LENGTH = 100  # NOTE: Very important. This value is what control how close or how far the UAV will fly related to the object. This is something we have to play with 


# Settings for selecting tests or run programs
PROGRAM_SELECT = const.HOVER_TEST


# Filter settings
ERR_X_EMA_APLHA = 0.15
ERR_Y_EMA_APLHA = 0.15
BBOX_W_EMA_APLHA = 0.15
BBOX_H_EMA_APLHA = 0.15


# Detection settings
# Settings for when the drone belive a real detection has occured and starts tracking.
MIN_CONSECUTIVE_DETECTIONS = 3          # Number of consecutive detections
DETECTION_CONFIRMATION_TIME = 0.25      # Seconds from first detection
# Settings for when the drone belive the object is lost and goes back to searching.
MAX_CONSECUTIVE_MISSED_DETECTIONS = 5   # Number of consecutive missed detections
DETECTION_LOST_TIME = 0.5               # Seconds from last detection

DETECTION_PHASE_TIMEOUT = 180.0         # Seconds before before completing the detection phase