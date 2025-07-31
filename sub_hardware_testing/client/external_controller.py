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
    """A data class to hold all results from processing a single camera frame."""
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
        
        self.lost_target_grace_period_s = 0.5
        self.lost_target_timer = 0.0

    def update(self, bbox: BoundingBox, dt: float) -> Tuple[float, float, float, float, bool]:
        surge, strafe, heave, yaw = 0.0, 0.0, 0.0, 0.0
        flash_lights = False

        if bbox.is_valid:
            self.lost_target_timer = 0.0
        else:
            self.lost_target_timer += dt

        if self.lost_target_timer > self.lost_target_grace_period_s:
            if self.state != ChaseStrategyState.SEARCHING:
                self.state = ChaseStrategyState.SEARCHING
            return 0.0, 0.0, 0.0, 0.0, False

        if self.state in [ChaseStrategyState.IDLE, ChaseStrategyState.SEARCHING]:
            if bbox.is_valid:
                print(f"[{id(self)}] Target acquired. Advancing.")
                self.state = ChaseStrategyState.ADVANCING
        
        elif self.state == ChaseStrategyState.ADVANCING:
            surge = self.advance_surge
            strafe = 0.0
            
            if bbox.is_valid:
                err_x = self.camera_center_x - bbox.center[0]
                err_y = self.camera_center_y - bbox.center[1]
                yaw = self.centering_kp * err_x
                heave = self.centering_kp * err_y

                if bbox.area > self.orbit_entry_thresh_area:
                    print(f"[{id(self)}] Target at optimal range. Entering ORBITING.")
                    self.state = ChaseStrategyState.ORBITING
                    self.state_timer = 0.0
        
        elif self.state == ChaseStrategyState.ORBITING:
            surge = 0.0
            
            if bbox.is_valid:
                err_x = self.camera_center_x - bbox.center[0]
                err_y = self.camera_center_y - bbox.center[1]
                yaw = self.centering_kp * err_x
                heave = -self.centering_kp * err_y
                strafe_scale = max(0.0, 1.0 - (abs(err_x) / self.max_yaw_error_for_strafe))
                strafe = self.orbit_strafe_command * strafe_scale

                if bbox.area < self.orbit_exit_thresh_area:
                    print(f"[{id(self)}] Target moved away. Re-ADVANCING.")
                    self.state = ChaseStrategyState.ADVANCING
            else:
                strafe = self.orbit_strafe_command
            
            self.state_timer += dt
            if self.state_timer > self.orbit_duration_for_win_s:
                print(f"[{id(self)}] Successfully orbited target. CELEBRATING.")
                self.state = ChaseStrategyState.CELEBRATING
                self.state_timer = 0.0

        elif self.state == ChaseStrategyState.CELEBRATING:
            self.state_timer += dt
            flash_lights = (int(self.state_timer * 4) % 2) == 0
            if self.state_timer > self.celebration_time_s:
                print(f"[{id(self)}] Celebration over. Returning to SEARCHING.")
                self.state = ChaseStrategyState.SEARCHING

        return surge, strafe, heave, yaw, flash_lights


class MyController:
    """
    External controller for submarine autonomous operation with dual YOLO model support.
    Always processes both cameras simultaneously and draws bounding boxes on both.
    """

    def __init__(self, sim_weights_path: str = "./unity.pt", real_weights_path: str = "./best.pt"):
        """Initialize the external controller with both YOLO models."""
        self.last_time = time.time()
        self.control_counter = 0
        
        print(f"Loading SIM YOLO model from {sim_weights_path}...")
        self.yolo_model_sim = YOLO(sim_weights_path)
        
        print(f"Loading REAL YOLO model from {real_weights_path}...")
        self.yolo_model_real = YOLO(real_weights_path)
        
        # KEY CHANGE: Initialize separate chase strategies for each camera feed
        print("Initializing separate chase strategies for REAL and SIM cameras.")
        self.chase_strategy_real = ChaseAndCircleStrategy(camera_width=640, camera_height=480)
        self.chase_strategy_sim = ChaseAndCircleStrategy(camera_width=640, camera_height=480)

        print("\nExternal Controller Initialized:")
        print("  - Both REAL and SIM cameras will be processed with YOLO simultaneously.")
        print("  - The GUI checkbox determines which camera's commands are used.")

# In external_controller.py, inside class MyController

    # In external_controller.py, inside class MyController

# In external_controller.py, inside class MyController

    def _process_single_camera(
        self,
        image_frame: Optional[np.ndarray],
        camera_type: str,
        chase_strategy: ChaseAndCircleStrategy,
        dt: float
    ) -> CameraProcessingResult:
        """
        Processes a single camera feed with corrected logic for flipping.
        It performs all calculations on the original image and only flips the
        final canvas for display purposes.
        """
        is_sim = (camera_type == "sim")
        camera_label = "SIM" if is_sim else "REAL"
        camera_color = (255, 255, 0) if is_sim else (100, 255, 100)
        yolo_model = self.yolo_model_sim if is_sim else self.yolo_model_real
        
        bbox_to_use = BoundingBox()
        has_frame = image_frame is not None

        # --- 1. Prepare the Canvas (NO FLIPPING HERE) ---
        if has_frame:
            # If we have a frame, use a copy of it as the base canvas
            canvas = image_frame.copy()
        else:
            # Otherwise, create a black placeholder canvas
            canvas = np.zeros((480, 640, 3), dtype=np.uint8)

        # --- 2. Run YOLO detection on the ORIGINAL, UNFLIPPED canvas ---
        if has_frame:
            try:
                results = yolo_model(canvas, verbose=False)
                highest_conf = 0.0
                best_box_coords = None
                if results and results[0].boxes:
                    for box in results[0].boxes:
                        if box.conf and box.conf[0] > highest_conf:
                            highest_conf = box.conf[0]
                            best_box_coords = box.xywh[0]
                if best_box_coords is not None:
                    cx, cy, w, h = best_box_coords
                    bbox_to_use = BoundingBox(x=(cx - w/2).item(), y=(cy - h/2).item(),
                                              width=w.item(), height=h.item())
            except Exception as e:
                print(f"!!! YOLO Error on {camera_label} camera: {e}")
                cv2.putText(canvas, "YOLO ERROR", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # --- 3. Get Control Commands from the Chase Strategy (using correct coordinates) ---
        s_surge, s_strafe, s_heave, s_yaw, flash_lights = chase_strategy.update(bbox_to_use, dt)
        
        # --- 4. Draw All Overlays on the UNFLIPPED canvas ---
        if not has_frame:
            cv2.putText(canvas, f"NO {camera_label} FEED", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2)

        if bbox_to_use.is_valid:
            pt1 = (int(bbox_to_use.x), int(bbox_to_use.y))
            pt2 = (int(bbox_to_use.x + bbox_to_use.width), int(bbox_to_use.y + bbox_to_use.height))
            cv2.rectangle(canvas, pt1, pt2, (0, 255, 0), 2)

        state_text = f"STATE: {chase_strategy.state.name}"
        cv2.putText(canvas, state_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, camera_color, 2)
        
        # --- 5. Flip the FINAL canvas just for display, if it's the sim camera ---
        final_processed_image = cv2.flip(canvas, 1) if is_sim else canvas
        
        return CameraProcessingResult(
            surge=s_surge, strafe=s_strafe, heave=s_heave, yaw_cmd=s_yaw,
            processed_image=final_processed_image, # Return the correctly oriented image for display
            bbox=bbox_to_use, 
            flash_lights=flash_lights
        )

    def update_loop(
        self,
        real_frame: Optional[np.ndarray],
        sim_frame: Optional[np.ndarray],
    ) -> Tuple[CameraProcessingResult, CameraProcessingResult]:
        """
        Main processing loop. ALWAYS processes both camera frames.
        This function is now decoupled from control selection.

        Returns:
            A tuple containing the full processing results for the real and sim cameras.
        """
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: dt = 1/50.0 # Avoid division by zero on first frame
        self.last_time = current_time
        self.control_counter += 1

        # --- STEP 1: Always process both cameras ---
        real_result = self._process_single_camera(real_frame, "real", self.chase_strategy_real, dt)
        sim_result = self._process_single_camera(sim_frame, "sim", self.chase_strategy_sim, dt)

        # --- STEP 2: Return the complete results for both ---
        if self.control_counter % 100 == 0: # Print status periodically
             print(f"VisionProc: REAL State={real_result.bbox.is_valid} | SIM State={sim_result.bbox.is_valid}")

        return real_result, sim_result

# Legacy function for compatibility if called directly elsewhere
def process_real_camera_frame(frame: np.ndarray) -> np.ndarray:
    if frame is None: return frame
    processed_frame = frame.copy()
    cv2.putText(processed_frame, "REAL (Legacy)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return processed_frame
