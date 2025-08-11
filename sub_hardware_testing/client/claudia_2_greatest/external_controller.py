#!/usr/bin/env python3
"""
External Controller for Submarine with YOLO-based object detection and chase strategy.
Processes both camera feeds simultaneously but uses commands from selected camera only.
"""

import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple
from enum import Enum, auto

import cv2
import numpy as np
import torch
from ultralytics import YOLO


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


@dataclass
class PIDState:
    prev_error: float = 0.0
    integral: float = 0.0


@dataclass
class BoundingBox:
    x: float = 0.0
    y: float = 0.0
    width: float = 0.0
    height: float = 0.0
    
    @property
    def center(self) -> Tuple[float, float]:
        return self.x + self.width/2, self.y + self.height/2
    
    @property
    def area(self) -> float:
        return self.width * self.height
    
    @property
    def is_valid(self) -> bool:
        return self.width > 0 and self.height > 0


@dataclass
class CameraProcessingResult:
    surge: float
    strafe: float
    heave: float
    yaw_cmd: float
    processed_image: Optional[np.ndarray]
    bbox: BoundingBox
    flash_lights: bool


class ChaseStrategyState(Enum):
    IDLE = auto()
    SEARCHING = auto()
    ADVANCING = auto()
    ORBITING = auto()
    CELEBRATING = auto()


class ChaseAndCircleStrategy:
    """
    Implements a robust "Orbit and Re-advance" strategy. It includes a grace
    period for transient bounding box losses to prevent state oscillation.
    """
    def __init__(self, camera_width: int, camera_height: int):
        self.state = ChaseStrategyState.IDLE
        self.camera_center_x = camera_width / 2
        self.camera_center_y = camera_height / 2
        print(f"Chase strategy initialized for {camera_width}x{camera_height} camera.")
        
        # --- Control Gains ---
        self.centering_kp = 0.0003
        self.advance_surge = 0.8
        self.orbit_strafe_command = 0.8
        self.max_yaw_error_for_strafe = 80.0

        # --- State Thresholds ---
        self.orbit_entry_thresh_area = camera_width * camera_height * 0.05
        self.orbit_exit_thresh_area = self.orbit_entry_thresh_area * 0.75
        
        # --- Timers and Robustness ---
        self.orbit_duration_for_win_s = 10.0
        self.celebration_time_s = 5.0
        self.state_timer = 0.0
        
        # NEW: Grace period for handling flickering bounding boxes
        self.lost_target_grace_period_s = 0.5  # (in seconds)
        self.lost_target_timer = 0.0

    def update(self, bbox: BoundingBox, dt: float) -> Tuple[float, float, float, float, bool]:
        surge, strafe, heave, yaw = 0.0, 0.0, 0.0, 0.0
        flash_lights = False

        # --- Handle Target Validity with Grace Period ---
        if bbox.is_valid:
            # If we see the target, reset the lost timer.
            self.lost_target_timer = 0.0
        else:
            # If we don't see the target, increment the timer.
            self.lost_target_timer += dt

        # --- Global Target Lost Condition ---
        if self.lost_target_timer > self.lost_target_grace_period_s:
            if self.state != ChaseStrategyState.SEARCHING:
                print(f"Target lost for over {self.lost_target_grace_period_s:.1f}s. Resetting to SEARCHING mode.")
                self.state = ChaseStrategyState.SEARCHING
            # When searching, there are no commands to issue.
            return 0.0, 0.0, 0.0, 0.0, False

        # --- State Machine ---
        # If we reach here, we either have a valid bbox, or we are in the grace period.

        # State: IDLE / SEARCHING
        if self.state in [ChaseStrategyState.IDLE, ChaseStrategyState.SEARCHING]:
            if bbox.is_valid:
                print("Target acquired. Advancing.")
                self.state = ChaseStrategyState.ADVANCING
        
        # State: ADVANCING
        elif self.state == ChaseStrategyState.ADVANCING:
            # Continue advancing even if the target flickers out of view (grace period).
            surge = self.advance_surge
            strafe = 0.0
            
            # Only apply yaw/heave corrections if we can actually see the target.
            if bbox.is_valid:
                err_x = self.camera_center_x - bbox.center[0]
                err_y = self.camera_center_y - bbox.center[1]
                yaw = self.centering_kp * err_x
                heave = self.centering_kp * err_y

                # Transition condition
                if bbox.area > self.orbit_entry_thresh_area:
                    print(f"Target at optimal range. Entering dynamic ORBITING mode.")
                    self.state = ChaseStrategyState.ORBITING
                    self.state_timer = 0.0
        
        # State: ORBITING
        elif self.state == ChaseStrategyState.ORBITING:
            surge = 0.0
            
            # If we can see the target, perform dynamic orbiting.
            if bbox.is_valid:
                err_x = self.camera_center_x - bbox.center[0]
                err_y = self.camera_center_y - bbox.center[1]
                yaw = self.centering_kp * err_x
                heave = -self.centering_kp * err_y
                strafe_scale = max(0.0, 1.0 - (abs(err_x) / self.max_yaw_error_for_strafe))
                strafe = self.orbit_strafe_command * strafe_scale

                # Transition Condition: If target gets too far, re-advance.
                if bbox.area < self.orbit_exit_thresh_area:
                    print("Target has moved too far away. Re-engaging with ADVANCING.")
                    self.state = ChaseStrategyState.ADVANCING
            else:
                # Bbox is lost, but we're in the grace period.
                # Continue the last action: orbit at full speed (since we have no error data).
                strafe = self.orbit_strafe_command
            
            # Win Condition
            self.state_timer += dt
            if self.state_timer > self.orbit_duration_for_win_s:
                print("Successfully orbited target. MISSION ACCOMPLISHED!")
                self.state = ChaseStrategyState.CELEBRATING
                self.state_timer = 0.0

        # State: CELEBRATING
        elif self.state == ChaseStrategyState.CELEBRATING:
            self.state_timer += dt
            flash_lights = (int(self.state_timer * 4) % 2) == 0
            if self.state_timer > self.celebration_time_s:
                print("Celebration over. Returning to SEARCHING.")
                self.state = ChaseStrategyState.SEARCHING

        return surge, strafe, heave, yaw, flash_lights


class MyController:
    """
    External controller for submarine autonomous operation with dual YOLO model support.
    Always processes both cameras simultaneously.
    """

    def __init__(self, sim_weights_path: str = "./unity.pt", real_weights_path: str = "./best.pt"):
        """Initialize the external controller with both YOLO models."""
        self.last_time = time.time()
        self.control_counter = 0
        
        # Initialize both YOLO models
        print(f"Loading SIM YOLO model from {sim_weights_path}...")
        self.yolo_model_sim = YOLO(sim_weights_path)
        print("SIM YOLO model loaded successfully.")
        
        print(f"Loading REAL YOLO model from {real_weights_path}...")
        self.yolo_model_real = YOLO(real_weights_path)
        print("REAL YOLO model loaded successfully.")
        
        # Initialize separate chase strategies for each camera
        self.chase_strategy_real = ChaseAndCircleStrategy(camera_width=640, camera_height=480)
        self.chase_strategy_sim = ChaseAndCircleStrategy(camera_width=640, camera_height=480)
        
        # PID controllers for backup autonomous behavior
        self.pid_gains = {
            "depth": PIDGains(kp=0.02, ki=0.001, kd=0.01),
            "yaw": PIDGains(kp=0.1, ki=0.01, kd=0.05),
        }
        self.pid_states = {k: PIDState() for k in self.pid_gains}

        print(" External Controller initialized with dual YOLO models and chase capability")
        print("   - Both cameras will be processed simultaneously")
        print("   - Commands will be generated from the selected camera")

    def _process_single_camera(self, image_frame: Optional[np.ndarray], camera_type: str, 
                              chase_strategy: ChaseAndCircleStrategy, dt: float) -> CameraProcessingResult:
        """Process a single camera feed and return results - ALWAYS processes even if frame is None."""
        
        # Camera-specific settings
        if camera_type == "sim":
            camera_label = "SIM"
            camera_color = (255, 255, 0)  # Yellow for sim
            yolo_model = self.yolo_model_sim
        else:
            camera_label = "REAL"
            camera_color = (0, 255, 0)  # Green for real
            yolo_model = self.yolo_model_real
        
        # Default result
        result = CameraProcessingResult(
            surge=0.0, strafe=0.0, heave=0.0, yaw_cmd=0.0,
            processed_image=None, bbox=BoundingBox(), flash_lights=False
        )
        
        # Create base image for processing
        if image_frame is None:
            # Create a black placeholder image with status text
            base_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(base_image, f"NO {camera_label} FEED", (200, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, camera_color, 2)
            processed_image = base_image
            bbox_to_use = BoundingBox()  # Invalid bbox
        else:
            try:
                # Apply camera-specific preprocessing
                if camera_type == "sim":
                    # Mirror the sim camera frame
                    preprocessed_frame = cv2.flip(image_frame, 1)
                else:
                    # Real camera - no mirroring needed
                    preprocessed_frame = image_frame.copy()
                
                # Run YOLO detection with appropriate model
                results = yolo_model(preprocessed_frame, verbose=False)
                
                # Find the detection with highest confidence
                highest_conf = 0.0
                best_box_coords = None
                
                if results[0].boxes is not None:
                    for box in results[0].boxes:
                        if box.conf[0] > highest_conf:
                            highest_conf = box.conf[0]
                            best_box_coords = box.xywh[0]
                
                # Convert to BoundingBox format
                bbox_to_use = BoundingBox()
                if best_box_coords is not None:
                    cx, cy, w, h = best_box_coords
                    x = cx - w/2
                    y = cy - h/2
                    bbox_to_use = BoundingBox(
                        x=x.item(), 
                        y=y.item(), 
                        width=w.item(), 
                        height=h.item()
                    )
                
                # Create processed image with annotations
                processed_image = preprocessed_frame.copy()
                
                # Draw bounding box if valid
                if bbox_to_use.is_valid:
                    pt1 = (int(bbox_to_use.x), int(bbox_to_use.y))
                    pt2 = (int(bbox_to_use.x + bbox_to_use.width), 
                           int(bbox_to_use.y + bbox_to_use.height))
                    cv2.rectangle(processed_image, pt1, pt2, (0, 255, 0), 2)
                    
                    # Add confidence text if available
                    if highest_conf > 0:
                        cv2.putText(processed_image, f"{highest_conf:.2f}", 
                                  (pt1[0], pt1[1] - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
            except Exception as e:
                print(f" External Controller YOLO Error ({camera_type}): {e}")
                # Fallback to original frame with error message
                processed_image = image_frame.copy()
                cv2.putText(processed_image, f"YOLO ERROR", (200, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                bbox_to_use = BoundingBox()
        
        # ALWAYS run chase strategy (even with invalid bbox)
        try:
            s_surge, s_strafe, s_heave, s_yaw, flash_lights = chase_strategy.update(bbox_to_use, dt)
        except Exception as e:
            print(f" Chase Strategy Error ({camera_type}): {e}")
            s_surge, s_strafe, s_heave, s_yaw, flash_lights = 0.0, 0.0, 0.0, 0.0, False
        
        # Add overlays to processed image
        if processed_image is not None:
            # Draw camera type indicator
            cv2.rectangle(processed_image, (10, 10), (50, 50), camera_color, 2)
            cv2.putText(processed_image, camera_label, (15, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, camera_color, 1)
            
            # Add strategy state to image
            state_text = f"STATE: {chase_strategy.state.name}"
            cv2.putText(processed_image, state_text, (10, 80), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Add control commands to image
            cmd_text = f"S:{s_surge:.2f} St:{s_strafe:.2f} H:{s_heave:.2f} Y:{s_yaw:.2f}"
            cv2.putText(processed_image, cmd_text, (10, 110), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Add camera type to status
            cam_text = f"CAMERA: {camera_type.upper()}"
            cv2.putText(processed_image, cam_text, (10, 140), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, camera_color, 1)
            
            # Add bbox status
            bbox_text = f"TARGET: {'DETECTED' if bbox_to_use.is_valid else 'NO TARGET'}"
            bbox_color = (0, 255, 0) if bbox_to_use.is_valid else (0, 0, 255)
            cv2.putText(processed_image, bbox_text, (10, 170), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, bbox_color, 1)
        
        # Update result
        result.surge = s_surge
        result.strafe = s_strafe
        result.heave = s_heave
        result.yaw_cmd = s_yaw
        result.processed_image = processed_image
        result.bbox = bbox_to_use
        result.flash_lights = flash_lights
        
        return result

    def update_loop(
        self,
        real_frame: Optional[np.ndarray],
        sim_frame: Optional[np.ndarray],
        imu_data: Optional[Dict],
        depth_measurement: Optional[Dict],
        bounding_box=None,
        selected_camera: str = "real"
    ) -> Tuple[float, float, float, float, Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Main control loop for external controller - ALWAYS processes both cameras simultaneously.

        Args:
            real_frame: Real camera frame (BGR format)
            sim_frame: Sim camera frame (BGR format)  
            imu_data: Complete IMU data dict with orientation and depth
            depth_measurement: Depth-specific data (extracted from imu_data["depth"])
            bounding_box: Optional bounding box data from simulator
            selected_camera: "sim" or "real" - which camera's commands to use

        Returns:
            Tuple of (surge, strafe, heave, yaw_cmd, processed_real_frame, processed_sim_frame)
        """
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.control_counter += 1

        # ALWAYS process both cameras simultaneously (even if frames are None)
        real_result = self._process_single_camera(real_frame, "real", self.chase_strategy_real, dt)
        sim_result = self._process_single_camera(sim_frame, "sim", self.chase_strategy_sim, dt)

        # Use commands from the selected camera
        if selected_camera == "sim":
            surge, strafe, heave, yaw_cmd = sim_result.surge, sim_result.strafe, sim_result.heave, sim_result.yaw_cmd
            active_bbox = sim_result.bbox
            active_strategy = self.chase_strategy_sim
        else:
            surge, strafe, heave, yaw_cmd = real_result.surge, real_result.strafe, real_result.heave, real_result.yaw_cmd
            active_bbox = real_result.bbox
            active_strategy = self.chase_strategy_real

        # Debug output (every 50 iterations = ~1 second at 50Hz)
        if self.control_counter % 50 == 0:
            print(f" External Controller (YOLO Chase - ACTIVE: {selected_camera.upper()}):")
            print(f"   Active State: {active_strategy.state.name}")
            print(f"   Active Commands: surge={surge:.2f}, strafe={strafe:.2f}, heave={heave:.2f}, yaw={yaw_cmd:.2f}")
            if active_bbox.is_valid:
                print(f"   Active Target: center=({active_bbox.center[0]:.1f},{active_bbox.center[1]:.1f}), area={active_bbox.area:.0f}")
            
            # Show both camera detection status
            real_status = "DETECTED" if real_result.bbox.is_valid else "NO TARGET"
            sim_status = "DETECTED" if sim_result.bbox.is_valid else "NO TARGET" 
            print(f"   REAL Camera: {real_status} | SIM Camera: {sim_status}")
            print(f"   Both cameras processed: REAL={'✓' if real_result.processed_image is not None else '✗'}, SIM={'✓' if sim_result.processed_image is not None else '✗'}")

        return surge, strafe, heave, yaw_cmd, real_result.processed_image, sim_result.processed_image

    def _update_pid(self, pid_name: str, error: float, dt: float) -> float:
        """Update PID controller with new error value."""
        if dt <= 0:
            return 0.0

        gains = self.pid_gains[pid_name]
        state = self.pid_states[pid_name]

        # Integral term with windup protection
        state.integral += error * dt
        state.integral = np.clip(state.integral, -10.0, 10.0)

        # Derivative term
        derivative = (error - state.prev_error) / dt

        # PID output
        output = gains.kp * error + gains.ki * state.integral + gains.kd * derivative

        state.prev_error = error
        return output

    def reset_pid(self, pid_name: str):
        """Reset specific PID controller state."""
        self.pid_states[pid_name] = PIDState()

    def reset_all_pids(self):
        """Reset all PID controller states."""
        for key in self.pid_states:
            self.pid_states[key] = PIDState()

    def _get_current_depth(self, depth_data: Optional[Dict]) -> Optional[float]:
        """Extract current depth in cm from depth measurement."""
        try:
            if depth_data and "relative_cm" in depth_data:
                return depth_data["relative_cm"]
            return None
        except Exception:
            return None

    def _get_current_yaw(self, imu_data: Optional[Dict]) -> Optional[float]:
        """Extract current yaw in degrees from IMU data."""
        try:
            if (
                imu_data
                and "orientation" in imu_data
                and imu_data["orientation"]
                and "euler_deg" in imu_data["orientation"]
            ):
                return imu_data["orientation"]["euler_deg"]["yaw"]
            return None
        except Exception:
            return None

    def _angle_difference(self, target: float, current: float) -> float:
        """Calculate the shortest angular difference between two angles."""
        diff = target - current
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def process_sim_camera_frame(self, frame: np.ndarray) -> np.ndarray:
        """Legacy method for compatibility - redirects to main processing."""
        if frame is None:
            return frame
        
        # Use the main update_loop for processing (only sim frame)
        _, _, _, _, _, processed_sim_frame = self.update_loop(None, frame, None, None, None, "sim")
        
        return processed_sim_frame if processed_sim_frame is not None else frame


# Legacy function for compatibility
def process_real_camera_frame(frame: np.ndarray) -> np.ndarray:
    if frame is None:
        return frame
    
    # Add basic overlay for real camera
    processed_frame = frame.copy()
    cv2.rectangle(processed_frame, (10, 10), (50, 50), (0, 255, 0), 2)
    cv2.putText(processed_frame, "REAL", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    return processed_frame


# Example usage and testing
if __name__ == "__main__":
    controller = MyController()

    # Test with dummy data
    dummy_imu = {"orientation": {"euler_deg": {"yaw": 45.0, "pitch": 0.0, "roll": 0.0}}}
    dummy_depth = {"relative_cm": -30.0}
    dummy_real_image = np.zeros((480, 640, 3), dtype=np.uint8)
    dummy_sim_image = np.zeros((480, 640, 3), dtype=np.uint8)

    # Test both camera configurations
    for selected_cam in ["real", "sim"]:
        print(f"\nTesting with {selected_cam} camera selected:")
        surge, strafe, heave, yaw_cmd, proc_real, proc_sim = controller.update_loop(
            dummy_real_image, dummy_sim_image, dummy_imu, dummy_depth, None, selected_cam
        )
        print(f"Commands from {selected_cam}: surge={surge:.2f}, strafe={strafe:.2f}, heave={heave:.2f}, yaw={yaw_cmd:.2f}")
        print(f"Both cameras processed: real={'✓' if proc_real is not None else '✗'}, sim={'✓' if proc_sim is not None else '✗'}")
