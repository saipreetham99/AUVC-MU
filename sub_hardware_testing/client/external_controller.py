#!/usr/bin/env python3
"""
External Controller for Submarine
This is an example implementation that can be customized for AI/autonomous control.
"""

import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


@dataclass
class PIDState:
    prev_error: float = 0.0
    integral: float = 0.0


class MyController:
    """
    Example external controller for submarine autonomous operation.
    Replace this implementation with your own AI/control logic.
    """

    def __init__(self):
        """Initialize the external controller."""
        self.last_time = time.time()
        self.control_counter = 0
        self.target_depth = -50.0  # Target 50cm deep
        self.target_yaw = 0.0  # Target heading

        # PID controllers for autonomous behavior
        self.pid_gains = {
            "depth": PIDGains(kp=0.02, ki=0.001, kd=0.01),
            "yaw": PIDGains(kp=0.1, ki=0.01, kd=0.05),
        }
        self.pid_states = {k: PIDState() for k in self.pid_gains}

        print(" External Controller initialized")
        print("   - Target depth: 50cm")
        print("   - Will perform simple autonomous circle pattern")

    def update_loop(
        self,
        image_frame: Optional[np.ndarray],
        imu_data: Optional[Dict],
        depth_measurement: Optional[Dict],
        bounding_box=None
    ) -> Tuple[float, float, float, float]:
        """
        Main control loop for external controller.

        Args:
            image_frame: Current camera frame (BGR format)
            imu_data: Complete IMU data dict with orientation and depth
            depth_measurement: Depth-specific data (extracted from imu_data["depth"])

        Returns:
            Tuple of (surge, strafe, heave, yaw_cmd) in range [-1.0, 1.0]
        """
        if bounding_box:
            x, y, w, h = bounding_box["x"], bounding_box["y"], bounding_box["width"], bounding_box["height"]


        # Default to neutral
        surge, strafe, heave, yaw_cmd = 0.0, 0.0, 0.0, 0.0

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.control_counter += 1

        try:
            # Extract current state
            current_depth = self._get_current_depth(depth_measurement)
            current_yaw = self._get_current_yaw(imu_data)

            # Simple autonomous behavior: Circle pattern while maintaining depth

            # 1. Depth control (heave)
            if current_depth is not None:
                depth_error = self.target_depth - current_depth
                heave = self._update_pid("depth", depth_error, dt)
                heave = np.clip(heave, -0.5, 0.5)  # Limit vertical speed

            # 2. Yaw control - slow circle pattern
            if current_yaw is not None:
                # Create a slowly changing target yaw for circle pattern
                circle_rate = 10.0  # degrees per second
                self.target_yaw += circle_rate * dt
                self.target_yaw = self.target_yaw % 360.0

                yaw_error = self._angle_difference(self.target_yaw, current_yaw)
                yaw_cmd = self._update_pid("yaw", yaw_error, dt)
                yaw_cmd = np.clip(yaw_cmd, -0.3, 0.3)  # Limit turn rate

            # 3. Forward motion - gentle cruise
            surge = 0.2  # Slow forward motion

            # 4. Image processing (example)
            if image_frame is not None:
                # Example: Simple obstacle avoidance based on image
                obstacle_detected = self._detect_obstacles(image_frame)
                if obstacle_detected:
                    surge = 0.0  # Stop if obstacle detected
                    print(" Obstacle detected - stopping forward motion")

            # Debug output (every 50 iterations = ~1 second at 50Hz)
            if self.control_counter % 50 == 0:
                print(f" External Controller:")
                print(
                    f"   Depth: {current_depth:.1f}cm (target: {self.target_depth:.1f}cm)"
                )
                print(f"   Yaw: {current_yaw:.1f}° (target: {self.target_yaw:.1f}°)")
                print(
                    f"   Commands: surge={surge:.2f}, heave={heave:.2f}, yaw={yaw_cmd:.2f}"
                )

        except Exception as e:
            print(f" External Controller Error: {e}")
            # Return neutral commands on error
            surge, strafe, heave, yaw_cmd = 0.0, 0.0, 0.0, 0.0

        return surge, strafe, heave, yaw_cmd

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

    def _detect_obstacles(self, image: np.ndarray) -> bool:
        """
        Simple obstacle detection using image processing.
        Replace with your own computer vision logic.
        """
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Simple edge detection
            edges = cv2.Canny(gray, 100, 200)

            # Count edge pixels in the center region
            h, w = edges.shape
            center_region = edges[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]
            edge_pixels = np.sum(center_region > 0)

            # Threshold for obstacle detection
            obstacle_threshold = 1000  # Adjust based on your environment

            return edge_pixels > obstacle_threshold

        except Exception:
            return False


# Example usage and testing
if __name__ == "__main__":
    controller = MyController()

    # Test with dummy data
    dummy_imu = {"orientation": {"euler_deg": {"yaw": 45.0, "pitch": 0.0, "roll": 0.0}}}

    dummy_depth = {"relative_cm": -30.0}
    dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)

    surge, strafe, heave, yaw_cmd = controller.update_loop(
        dummy_image, dummy_imu, dummy_depth
    )

    print(
        f"Test output: surge={surge:.2f}, strafe={strafe:.2f}, heave={heave:.2f}, yaw={yaw_cmd:.2f}"
    )
