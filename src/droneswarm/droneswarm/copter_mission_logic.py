#!/usr/bin/env python3

import droneswarm.utility.constants as const
import droneswarm.utility.settings as setting
import time

from droneswarm.utility.detections_normalizer import normalize_detection 
from droneswarm.utility.pid_controller import PID

"""
The mission logic here is implemented using the State Design Pattern.
https://refactoring.guru/design-patterns/state
https://refactoring.guru/design-patterns/state/python/example
"""


""" --- State Interface --- """
class State: 

    def __init__(self):
        self.context = None  # Backreference to the context
        self.parent = None   # ONLY inside composites for substates
        self.node = None     # Reference to calling node

    def set_context(self, context):
        # ONLY by top-level states - back reference to the context
        self.context = context

    def set_parent(self, parent): 
        # ONLY inside composites for substates
        self.parent = parent

    def set_node(self, node):
        # Set reference to calling node
        self.node = node

    def run(self):
        #  Override in subclasses (Concrete states). 'node' is the ROS2 node giving access to commands and telemetry.
        raise NotImplementedError("State has not been implemented")

    def on_enter(self, ):
        #  Override in subclasses (Concrete states), but only if needed. This method is not required but optional!
        pass

""" --- Concrete states --- """
class PreArmState(State):
    def run(self):
        if not self.node.is_prearm_ok:
            return
        
        if not self.node.mode_switch_in_progress and (self.node.current_ap_status_mode != const.COPTER_MODE_GUIDED or self.node.internal_mode != const.INTERNAL_MODE_GUIDED_POS):
            self.node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_POS)
            return
        elif self.node.mode_switch_in_progress:
            return
        
        self.context.transition_to(ArmDroneState())

class ArmDroneState(State):
    def run(self):
        # Start arming once prearm succeeds and mode switch completed
        if not self.node.arming_in_progress and not self.node.current_ap_status_armed:
            self.node.arm()
            return
        elif self.node.arming_in_progress:
            return
    
        self.node.set_origin_alt() # Save origin altitude. We do this because no topic provide target altitude from takeoff geopose commands so its something we have to keep track of ourselves
        self.context.transition_to(TakeoffState())

class TakeoffState(State):
    def run(self):
        if not self.node.current_ap_status_flying:
            self.node.cmd_takeoff(alt_osd = setting.TAKEOFF_ALT)
            return
        elif self.node.is_takeoff_complete():
            self.context.transition_to(TestOrRunState())

class TestOrRunState(State):
    def run(self):
        if setting.PROGRAM_SELECT == const.RUN_PROGRAM:
            self.context.transition_to(DetectionTrackingCompositeState())
        elif setting.PROGRAM_SELECT == const.HOVER_TEST:
            self.context.transition_to(HoverTestState())
        elif setting.PROGRAM_SELECT == const.VELEOCITY_TEST:
            self.context.transition_to(VelocityTestState())
        else:
            raise RuntimeError(
                f"Selected program does not exist"
            )

class ReturnToLaunchCoordState(State):
    def run(self):
        self.node.switch_flight_mode(const.INTERNAL_MODE_RTL)
        
        if self.node.has_reached_goal():
            self.context.transition_to(LandingState())

class LandingState(State):
    def run(self):
        self.node.switch_flight_mode(const.INTERNAL_MODE_LAND)
        if not self.node.current_ap_status_flying:
            self.context.transition_to(FinishedState())

class FinishedState(State):
    def run(self):
        # A state for doing nothing when landed
        pass

class HoverTestState(State):
    def __init__(self):
        super().__init__()
        self.entry_time = None
        self.hover_duration = 30.0 # seconds

    def on_enter(self):
        self.entry_time = time.perf_counter()

    def run(self):
        if not self.node.mode_switch_in_progress and (self.node.current_ap_status_mode != const.COPTER_MODE_GUIDED or self.node.internal_mode != const.INTERNAL_MODE_GUIDED_VEL):
            self.node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_VEL)
            return
        elif self.node.mode_switch_in_progress:
            return
        
        self.node.cmd_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) 

        current_time = time.perf_counter()
        elapsed_time = current_time - self.entry_time

        if elapsed_time >= self.hover_duration:
            self.context.transition_to(LandingState())

class VelocityTestState(State):
    def __init__(self):
        super().__init__()
        self._vel_test_phase = 0 
        self._vel_test_phase_start = None

    def start_phase(self, phase): 
        self._vel_test_phase = phase 
        self._vel_test_phase_start = time.time()

    def run(self):

        # On first entry into the state, initialize the test sequence
        if self._vel_test_phase_start is None:

            if not self.node.mode_switch_in_progress and (self.node.current_ap_status_mode != const.COPTER_MODE_GUIDED or self.node.internal_mode != const.INTERNAL_MODE_GUIDED_VEL):
                self.node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_VEL)
                return
            elif self.node.mode_switch_in_progress:
                return
            self.start_phase(1)
            return

        # Each phase lasts 2 seconds
        elapsed = time.time() - self._vel_test_phase_start
        PHASE_DURATION = 4.0

        # Phase 1: Forward
        if self._vel_test_phase == 1:
            self.node.cmd_velocity((1.0, 0.0, 0.0), (0.0,0.0,0.0)) 
            if elapsed > PHASE_DURATION:
                self.start_phase(2)

        # Phase 2: Backward
        elif self._vel_test_phase == 2:
            self.node.cmd_velocity((-1.0, 0.0, 0.0), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(3)

        # Phase 3: Right
        elif self._vel_test_phase == 3:
            self.node.cmd_velocity((0.0, 1.0, 0.0), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(4)

        # Phase 4: Left
        elif self._vel_test_phase == 4:
            self.node.cmd_velocity((0.0, -1.0, 0.0), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(5)

        # Phase 5: Up (remember vz positive = climb)
        elif self._vel_test_phase == 5:
            self.node.cmd_velocity((0.0, 0.0, 0.5), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(6)

        # Phase 6: Down (vz negative itive = descend)
        elif self._vel_test_phase == 6:
            self.node.cmd_velocity((0.0, 0.0, -0.5), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(7)

        # Phase 7: Yaw counter-clockwise
        elif self._vel_test_phase == 7:
            self.node.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,0.5))
            if elapsed > PHASE_DURATION:
                self.start_phase(8)

        # Phase 8: Yaw clockwise
        elif self._vel_test_phase == 8:
            self.node.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,-0.5))
            if elapsed > PHASE_DURATION:
                self.start_phase(9)

        # Phase 9: Diagonal forward-right
        elif self._vel_test_phase == 9:
            self.node.cmd_velocity((0.7, 0.7, 0.0), (0.0,0.0,0.0))
            if elapsed > PHASE_DURATION:
                self.start_phase(10)

        elif self._vel_test_phase == 10:
            self.node.cmd_velocity((10.0, 0.0, 2.0), (0.0,0.0,0.0))
            print(self.node._current_ap_geopose.position.altitude - self.node._origin_altitude)
            if elapsed > PHASE_DURATION:
                self.start_phase(11)

        # Phase 10: Complete — stop motion
        elif self._vel_test_phase == 11:
            self.context.transition_to(ReturnToLaunchCoordState())

            # node.cmd_velocity((0.0,0.0,0.0), (0.0,0.0,0.0))
            # node.get_logger().info("Velocity test sequence complete.")

""" --- Composite substates for detection tracking --- """
class AwaitingDetectionSubstate(State):
    def __init__(self):
        super().__init__()
        self.detect_counter = 0
        self.first_valid_time = None

    def run(self):

        # Hover while waiting
        self.node.cmd_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) 

        # Detection logic
        if self.node.have_detection:

            if self.detect_counter == 0:
                self.first_valid_time = time.perf_counter()

            self.detect_counter += 1
            time_valid = time.perf_counter() - self.first_valid_time

            if self.detect_counter >= setting.MIN_CONSECUTIVE_DETECTIONS and time_valid >= setting.DETECTION_CONFIRMATION_TIME:
                self.parent.transition_to_sub(TrackingDetectionSubstate())
                return

        else:
            # Reset confirmation on dropout
            self.detect_counter = 0
            self.first_valid_time = None

class TrackingDetectionSubstate(State):
    def __init__(self):
        super().__init__()
        self.missed_detect_counter = 0 # Counter for lost detections
        # Timer for lost detections are handled by detections callback in copter_controller_node.py

        self.pid_x = PID(kp=1.0, ki=0.0, kd=0.1)
        self.pid_y = PID(kp=1.0, ki=0.0, kd=0.1)
        self.pid_z = PID(kp=1.0, ki=0.0, kd=0.1)


    def run(self):
 
        # Detection lost logic
        if self.node.have_detection:
            self.missed_detect_counter = 0
        else:
            self.missed_detect_counter += 1
        
        elapsed_since_last_detection = time.perf_counter() - self.node.last_detection_time 
        if elapsed_since_last_detection >= setting.DETECTION_LOST_TIME and self.missed_detect_counter >= setting.MAX_CONSECUTIVE_MISSED_DETECTIONS: 
            self.parent.transition_to_sub(AwaitingDetectionSubstate())
            return

        # Log filtered errors for debugging
        self.node.get_logger().info(f"Filtered Errors - X: {self.node.filtered_err_x:.3f}, Y: {self.node.filtered_err_y:.3f}, BBox W: {self.node.filtered_bbox_w:.3f}, BBox H: {self.node.filtered_bbox_h:.3f}")

        # Control logic to follow the detected object
        n_err_x, n_err_y, n_edge_dist = normalize_detection(
            self.node.filtered_err_x, 
            self.node.filtered_err_y,
            self.node.filtered_bbox_w,
            self.node.filtered_bbox_h
        )

        # Log normalized errors for debugging
        self.node.get_logger().info(f"Normalized Errors - X: {n_err_x:.3f}, Y: {n_err_y:.3f}, Edge Dist: {n_edge_dist:.3f}")

        vx = self.pid_x.update(n_err_x, setting.COPTER_CONTROL_LOOP_DT)
        vy = self.pid_y.update(n_err_y, setting.COPTER_CONTROL_LOOP_DT)
        vz = self.pid_z.update(n_edge_dist, setting.COPTER_CONTROL_LOOP_DT)

        # log computed velocities for debugging
        self.node.get_logger().info(f"Computed Velocities - VX: {vx:.3f}, VY: {vy:.3f}, VZ: {vz:.3f}")

        # self.node.cmd_velocity((vx, vy, vz), (0.0, 0.0, 0.0))


    

""" --- Composite state --- """
class DetectionTrackingCompositeState(State):
    def __init__(self):
        super().__init__()
        self._current_substate = None
        self.enter_time = None
        
        
    def on_enter(self):
        initial_substate = AwaitingDetectionSubstate()
        self.transition_to_sub(initial_substate)
        self.enter_time = time.perf_counter() # Used for the composite exit condition

    def transition_to_sub(self, new_substate):
        self.node.get_logger().info(f"Changing to substate to {type(new_substate).__name__}")
        self._current_substate = new_substate
        self._current_substate.set_parent(self) # Set the substate's backreference to this composite state
        self._current_substate.set_context(self.context) # Set the substate's backreference to the overall context
        self._current_substate.set_node(self.node)
        self._current_substate.on_enter() # Call substate on_enter method if any - Not the same as this class's on_enter!

    def run(self):

        if not self.node.mode_switch_in_progress and (self.node.current_ap_status_mode != const.COPTER_MODE_GUIDED or self.node.internal_mode != const.INTERNAL_MODE_GUIDED_VEL):
            self.node.switch_flight_mode(const.INTERNAL_MODE_GUIDED_VEL)
            return
        elif self.node.mode_switch_in_progress:
            return

        # Shared exit condition for composite state
        elapsed = time.perf_counter() - self.enter_time
        if elapsed >= setting.DETECTION_PHASE_TIMEOUT:  
            self.context.transition_to(ReturnToLaunchCoordState())
            return

        # Continue executing in the current substate 
        self._current_substate.run()

 
""" --- Context --- """
class CopterControllerFSM: 
    def __init__(self, node):
        super().__init__()
        initial_state = PreArmState()
        self._current_state = None
        self._node = node # node passes the self reference when we initialized the FSM in copter_controller_node.
        self.transition_to(initial_state)

    def transition_to(self, state: State):
        # Change the current state, and update the state's backreference.
        self._node.get_logger().info(f"Changing to state to {type(state).__name__}")
        self._current_state = state # Update current state
        self._current_state.set_context(self) # update the current states back reference
        self._current_state.set_node(self._node)
        self._current_state.on_enter()
    
    def step(self): 
        self._current_state.run()




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
            
                