#!/usr/bin/env python3

import droneswarm.utility.constants as const
import droneswarm.utility.settings as setting
import time

from our_custom_interfaces.msg import ObjectData

"""
The mission logic here is implemented using the State Design Pattern.
https://refactoring.guru/design-patterns/state
https://refactoring.guru/design-patterns/state/python/example
"""

def normalize_detection(detection: ObjectData) -> tuple[float, float, float]:
    """ Returns a normalized detection tuple based on image size and BBOX_EDGE_LENGTH """

    err_x = float(detection.err_x)
    err_y = float(detection.err_y)
    bbox_w = float(detection.w)
    bbox_h = float(detection.h)

    # normalize to [-1, 1]
    n_err_x = err_x / (const.IMAGE_WIDTH / 2.0)
    n_err_y = err_y / (const.IMAGE_HEIGHT / 2.0)

    avg_edge_size = (bbox_w + bbox_h) / 2.0

    # normalise bounding box to a desired average edge length. If n_e_dist is > 0 then then UAV is to far. If < 0 then its too close.  
    n_e_dist = (setting.BBOX_EDGE_LENGTH - avg_edge_size) / setting.BBOX_EDGE_LENGTH
    n_e_dist = max(-2.0, min(2.0, n_e_dist))

    return (n_err_x, n_err_y, n_e_dist)

    


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
            self.context.transition_to(AwaitingDetectionState())

class AwaitingDetectionState(State):
    def run(self, node):
        pass

        self.context.transition_to(TrackingDetectionState())

class TrackingDetectionState(State):
    def run(self, node):
        pass

        self.context.transition_to(AwaitingDetectionState())

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
            
                