#!/usr/bin/env python3

import droneswarm.common.constants as const
import droneswarm.common.settings as setting
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
        #  Override in subclasses. 'node' is the ROS2 node giving access to commands and telemetry.
        raise NotImplementedError("State has not been implemented")


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
            self.context.transition_to(TestState())

class TestState(State):
    def __init__(self):
        self._vel_test_phase = 0 
        self._vel_test_phase_start = None

    def start_phase(self, phase): 
        self._vel_test_phase = phase 
        self._vel_test_phase_start = time.time()

    def run(self, node):

        if not node.mode_switch_in_progress and (node.current_ap_status_mode != const.COPTER_MODE_GUIDED or node.internal_mode != const.INTERNAL_MODE_GUIDED_VEL):
            node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_VEL)
            return
        elif node.mode_switch_in_progress:
            return

        # On first entry into the state, initialize the test sequence
        if self._vel_test_phase_start is None:
            node.get_logger().info("Starting velocity test sequence...")
            self.start_phase(1)
            return

        # Each phase lasts 2 seconds
        elapsed = time.time() - self._vel_test_phase_start
        PHASE_DURATION = 4.0

        # Phase 1: Forward
        if self._vel_test_phase == 1:
            node.cmd_velocity((1.0, 0.0, 0.0), (0.0,0.0,0.0))  # forward
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
            node.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,0.0))
            node.get_logger().info("Velocity test sequence complete.")

 
            
        
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

    
    def step(self, node): # node passes the self reference when we call step in the copter_controller_node.
        self._current_state.run(node)


# # Hard code some known locations
# # Note - Altitude in geopy is in km!
# GRAYHOUND_TRACK = point.Point(latitude=-35.355996, longitude=149.159017, altitude=0.584)
# # CMAC = point.Point(latitude=-35.3627010, longitude=149.1651513, altitude=0.585)


# # NOTE: temp
# self.state = STATE1
# self._vel_test_phase = 0
# self._vel_test_phase_start = None

# # NOTE: Temp
# def start_phase(phase):
#     self._vel_test_phase = phase
#     self._vel_test_phase_start = time.time()


# def copter_control_loop(node):

#     if self.state == STATE1:
#         if not self._is_prearm_ok:
#             return

#         if not self._mode_switch_in_progress and self._current_ap_status_mode != COPTER_MODE_GUIDED and self._internal_mode != INTERNAL_MODE_GUIDED_POS:
#             self.switch_flight_mode(INTERNAL_MODE_GUIDED_POS)
#             return
#         elif self._mode_switch_in_progress:
#             return

#         # Start arming once prearm succeeds and mode switch completed
#         if not self._arming_in_progress and not self._current_ap_status_armed:
#             self._arm_request()
#             return
#         elif self._arming_in_progress:
#             return

#         if not self._current_ap_status_flying and not self._takeoff_in_progress:

#             # Save origin altitude. We do this because no topic provide target altitude from takeoff geopose commands so its something we have to keep track of ourselves
#             self.set_origin_alt()

#             self.cmd_takeoff(alt_osd = TAKEOFF_ALT)

#         if self.has_reached_goal():
#             self.state = STATE2
#             self.switch_flight_mode(INTERNAL_MODE_GUIDED_VEL)



#     elif self.state == STATE2:
        
#         # On first entry into STATE2, initialize the test sequence
#         if self._vel_test_phase_start is None:
#             self.get_logger().info("Starting velocity test sequence...")
#             self.start_phase(1)
#             return

#         # Each phase lasts 2 seconds
#         elapsed = time.time() - self._vel_test_phase_start
#         PHASE_DURATION = 4.0

#         # Phase 1: Forward
#         if self._vel_test_phase == 1:
#             self.cmd_velocity((1.0, 0.0, 0.0), (0.0,0.0,0.0))  # forward
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(2)

#         # Phase 2: Backward
#         elif self._vel_test_phase == 2:
#             self.cmd_velocity((-1.0, 0.0, 0.0), (0.0,0.0,0.0))
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(3)

#         # Phase 3: Right
#         elif self._vel_test_phase == 3:
#             self.cmd_velocity((0.0, 1.0, 0.0), (0.0,0.0,0.0))
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(4)

#         # Phase 4: Left
#         elif self._vel_test_phase == 4:
#             self.cmd_velocity((0.0, -1.0, 0.0), (0.0,0.0,0.0))
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(5)

#         # Phase 5: Up (remember vz negative = climb)
#         elif self._vel_test_phase == 5:
#             self.cmd_velocity((0.0, 0.0, -0.5), (0.0,0.0,0.0))
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(6)

#         # Phase 6: Down (vz positive = descend)
#         elif self._vel_test_phase == 6:
#             self.cmd_velocity((0.0, 0.0, 0.5), (0.0,0.0,0.0))
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(7)

#         # Phase 7: Yaw clockwise
#         elif self._vel_test_phase == 7:
#             self.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,0.5))
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(8)

#         # Phase 8: Yaw counter-clockwise
#         elif self._vel_test_phase == 8:
#             self.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,-0.5))
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(9)

#         # Phase 9: Diagonal forward-right
#         elif self._vel_test_phase == 9:
#             self.cmd_velocity((0.7, 0.7, 0.0), (0.0,0.0,0.0))
#             if elapsed > PHASE_DURATION:
#                 self.start_phase(10)

#         # Phase 10: Complete — stop motion
#         elif self._vel_test_phase == 10:
#             self.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,0.0))
#             self.get_logger().info("Velocity test sequence complete.")
#             # Optionally transition to next state:
#             # self.state = STATE3




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
            
                