#!/usr/bin/env python3

import droneswarm.utility.constants as const
import droneswarm.utility.settings as setting
import time

"""
The mission logic here is implemented using the State Design Pattern.
https://refactoring.guru/design-patterns/state
https://refactoring.guru/design-patterns/state/python/example
"""


""" --- State Interface --- """
class State: 

    def __init__(self):
        self.context = None # States backreference to the context

    def set_context(self, context):
        self.context = context

    def run(self, node):
        #  Override in subclasses (Concrete states). 'node' is the ROS2 node giving access to commands and telemetry.
        raise NotImplementedError("State has not been implemented")

    def on_enter(self):
        #  Override in subclasses (Concrete states), but only if needed. This method is not required but optional!
        pass


""" --- Concrete states --- """
class PreArmState(State):
    def run(self, node):
        if not node.is_prearm_ok:
            return
        
        if not node.mode_switch_in_progress and (node.current_ap_status_mode != const.COPTER_MODE_GUIDED or node.internal_mode != const.INTERNAL_MODE_GUIDED_POS):
            node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_POS)
            return
        elif node.mode_switch_in_progress:
            return
        
        self.context.transition_to(ArmDroneState())

class ArmDroneState(State):
    def run(self, node):
        # Start arming once prearm succeeds and mode switch completed
        if not node.arming_in_progress and not node.current_ap_status_armed:
            node.arm()
            return
        elif node.arming_in_progress:
            return
    
        node.set_origin_alt() # Save origin altitude. We do this because no topic provide target altitude from takeoff geopose commands so its something we have to keep track of ourselves
        self.context.transition_to(TakeoffState())

class TakeoffState(State):
    def run(self, node):
        if not node.current_ap_status_flying:
            node.cmd_takeoff(alt_osd = setting.TAKEOFF_ALT)
            return
        elif node.is_takeoff_complete():
            self.context.transition_to(TestOrRunState())

class TestOrRunState(State):
    def run(self, node):
        if setting.PROGRAM_SELECT == const.RUN_PROGRAM:
            self.context.transition_to(AwaitingDetectionState())
        elif setting.PROGRAM_SELECT == const.HOVER_TEST:
            self.context.transition_to(HoverTestState())
        elif setting.PROGRAM_SELECT == const.VELEOCITY_TEST:
            self.context.transition_to(VelocityTestState())
        else:
            raise RuntimeError(
                f"Selected program does not exist"
            )

class AwaitingDetectionState(State):
    def __init__(self):
        self.detect_counter = 0
        self.first_valid_time = None

    def run(self, node):
        if not node.mode_switch_in_progress and (node.current_ap_status_mode != const.COPTER_MODE_GUIDED or node.internal_mode != const.INTERNAL_MODE_GUIDED_VEL):
            node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_VEL)
            return
        elif node.mode_switch_in_progress:
            return
        
        # Hover while waiting
        node.cmd_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) 

        # Detection logic
        if node.have_detection:

            if self.detect_counter == 0:
                self.first_valid_time = time.perf_counter()

            self.detect_counter += 1
            time_valid = time.perf_counter() - self.first_valid_time

            if self.detect_counter >= setting.MIN_CONSECUTIVE_DETECTIONS and time_valid >= setting.DETECTION_CONFIRMATION_TIME:
                self.context.transition_to(TrackingDetectionState())
                return

        else:
            # Reset confirmation on dropout
            self.detect_counter = 0
            self.first_valid_time = None

class TrackingDetectionState(State):
    def __init__(self):
        self.missed_detect_counter = 0 # Counter for lost detections
        # Timer for lost detections are handled by detections callback in copter_controller_node.py

    def run(self, node):
        if not node.mode_switch_in_progress and (node.current_ap_status_mode != const.COPTER_MODE_GUIDED or node.internal_mode != const.INTERNAL_MODE_GUIDED_VEL):
            node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_VEL)
            return
        elif node.mode_switch_in_progress:
            return

        if node.have_detection:
            self.missed_detect_counter = 0
        else:
            self.missed_detect_counter += 1
        
        elapsed_since_last_detection = time.perf_counter() - node.last_detection_time 
        if elapsed_since_last_detection >= setting.DETECTION_LOST_TIME and self.missed_detect_counter >= setting.MAX_CONSECUTIVE_MISSED_DETECTIONS: 
            self.context.transition_to(AwaitingDetectionState())

class ReturnToLaunchCoordState(State):
    def run(self, node):
        node.switch_flight_mode(const.INTERNAL_MODE_RTL)
        
        if node.has_reached_goal():
            self.context.transition_to(LandingState())

class LandingState(State):
    def run(self, node):
        node.switch_flight_mode(const.INTERNAL_MODE_LAND)
        if not node.current_ap_status_flying:
            self.context.transition_to(FinishedState())

class FinishedState(State):
    def run(self, node):
        # A state for doing nothing when landed
        pass

class HoverTestState(State):
    def __init__(self):
        self.entry_time = None
        self.hover_duration = 30.0 # seconds

    def on_enter(self):
        self.entry_time = time.perf_counter()

    def run(self, node):
        if not node.mode_switch_in_progress and (node.current_ap_status_mode != const.COPTER_MODE_GUIDED or node.internal_mode != const.INTERNAL_MODE_GUIDED_VEL):
            node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_VEL)
            return
        elif node.mode_switch_in_progress:
            return
        
        node.cmd_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) 

        current_time = time.perf_counter()
        elapsed_time = current_time - self.entry_time

        if elapsed_time >= self.hover_duration:
            self.context.transition_to(LandingState())


class VelocityTestState(State):
    def __init__(self):
        self._vel_test_phase = 0 
        self._vel_test_phase_start = None

    def start_phase(self, phase): 
        self._vel_test_phase = phase 
        self._vel_test_phase_start = time.time()

    def run(self, node):

        # On first entry into the state, initialize the test sequence
        if self._vel_test_phase_start is None:

            if not node.mode_switch_in_progress and (node.current_ap_status_mode != const.COPTER_MODE_GUIDED or node.internal_mode != const.INTERNAL_MODE_GUIDED_VEL):
                node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_VEL)
                return
            elif node.mode_switch_in_progress:
                return
            self.start_phase(1)
            return

        # Each phase lasts 2 seconds
        elapsed = time.time() - self._vel_test_phase_start
        PHASE_DURATION = 4.0

        # Phase 1: Forward
        if self._vel_test_phase == 1:
            node.cmd_velocity((1.0, 0.0, 0.0), (0.0,0.0,0.0)) 
            if elapsed > PHASE_DURATION:
                self.start_phase(2)

        # Phase 2: Backward
        elif self._vel_test_phase == 2:
            node.cmd_velocity((-1.0, 0.0, 0.0), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(3)

        # Phase 3: Right
        elif self._vel_test_phase == 3:
            node.cmd_velocity((0.0, 1.0, 0.0), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(4)

        # Phase 4: Left
        elif self._vel_test_phase == 4:
            node.cmd_velocity((0.0, -1.0, 0.0), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(5)

        # Phase 5: Up (remember vz positive = climb)
        elif self._vel_test_phase == 5:
            node.cmd_velocity((0.0, 0.0, 0.5), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(6)

        # Phase 6: Down (vz negative itive = descend)
        elif self._vel_test_phase == 6:
            node.cmd_velocity((0.0, 0.0, -0.5), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(7)

        # Phase 7: Yaw counter-clockwise
        elif self._vel_test_phase == 7:
            node.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,0.5))
            if elapsed > PHASE_DURATION:
                self.start_phase(8)

        # Phase 8: Yaw clockwise
        elif self._vel_test_phase == 8:
            node.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,-0.5))
            if elapsed > PHASE_DURATION:
                self.start_phase(9)

        # Phase 9: Diagonal forward-right
        elif self._vel_test_phase == 9:
            node.cmd_velocity((0.7, 0.7, 0.0), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(10)

        elif self._vel_test_phase == 10:
            node.cmd_velocity((10.0, 0.0, 2.0), (0.0,0.0,0.0))
            print(node._current_ap_geopose.position.altitude - node._origin_altitude)
            if elapsed > PHASE_DURATION:
                self.start_phase(11)

        # Phase 10: Complete — stop motion
        elif self._vel_test_phase == 11:
            self.context.transition_to(ReturnToLaunchCoordState())

            # node.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,0.0))
            # node.get_logger().info("Velocity test sequence complete.")

 
""" --- Context --- """
class CopterControllerFSM: 
    def __init__(self):
        initial_state = PreArmState()
        self._current_state = None
        self.transition_to(initial_state)

    def transition_to(self, state: State):
        # Change the current state, and update the state's backreference.
        self._current_state = state # Update current state
        self._current_state.set_context(self) # update the current states back reference
        self._current_state.on_enter()

    
    def step(self, node): # node passes the self reference when we call step in the copter_controller_node.
        self._current_state.run(node)




#             #     self.cmd_navigate_to(
#             #         lat = GRAYHOUND_TRACK.latitude, 
#             #         lon = GRAYHOUND_TRACK.longitude, 
#             #         alt_osd = 20.0)

#             #     if self.has_reached_goal():
#             #         self.state = STATE3

#             # elif self.state == STATE3:
#             #     if not self._mode_switch_in_progress and self._current_ap_status_mode != COPTER_MODE_RTL:
#             #         self._switch_mode_request(COPTER_MODE_RTL)
#             #         return
#             #     elif self._mode_switch_in_progress:
#             #         return
            
                