#!/usr/bin/env python3
"""
Async BlueROV2 Controller with Sync/Async Mode and significant networking
performance improvements.

Key improvements:
- Async UDP socket operations for minimal latency.
- Non-blocking network calls that don't stall the control loop.
- Concurrent command execution for real robot and simulator.
- Selectable sync/async communication with the simulator via CLI and gamepad.
"""

import argparse
import asyncio
import json
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import cv2
# Use a non-interactive backend for Matplotlib
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pygame
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    from external_controller import MyController

    EXTERNAL_CONTROLLER_AVAILABLE = True
    print(" External controller loaded successfully")
except ImportError:
    EXTERNAL_CONTROLLER_AVAILABLE = False
    print(" External controller not found - only joystick control available")


matplotlib.use("Agg")


class MsgHeader(Enum):
    ERROR = 0
    NO_ERROR = 1
    HEARTBEAT = 2
    GET_MODEL_INFO = 3
    GET_SENSORDATA = 4
    GET_RGB_IMAGE = 5
    GET_MASKED_IMAGE = 6
    APPLY_CTRL = 7
    STEP_SIM = 8
    RESET = 9


# Expected bytes from server for a response containing sensor data
SENSOR_DATA_RECV_BYTES = 40  # 10 floats * 4 bytes/float (4 sensor + 1 time + 1 depth + 4 bbox)
ACK_WITH_SENSOR_DATA_RECV_BYTES = 41  # 1 byte status + 40 bytes sensor+time+depth+bbox data


class SubmarineDataReceiverThread(threading.Thread):
    def __init__(self, video_port=10001, imu_port=10002):
        super().__init__(daemon=True, name="SubmarineDataReceiver")
        self.video_listen_addr = ("", video_port)
        self.imu_listen_addr = ("", imu_port)
        self.latest_frame = None
        self.latest_imu_data = None
        self.frame_lock = threading.Lock()
        self.stop_event = threading.Event()

    def _video_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(self.video_listen_addr)
        s.settimeout(1.0)
        b = {}

        while not self.stop_event.is_set():
            try:
                d, _ = s.recvfrom(65536)
                if len(d) < 6:
                    continue

                fid, cs, cid = struct.unpack("!HHH", d[:6])
                if fid not in b:
                    b[fid] = {}

                b[fid][cid] = d[6:]
                if len(b[fid]) == cs:
                    jpeg = b"".join(v for k, v in sorted(b[fid].items()))
                    fr = cv2.imdecode(
                        np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                    )
                    with self.frame_lock:
                        self.latest_frame = fr
                    del b[fid]

            except socket.timeout:
                continue
            except Exception:
                pass

        s.close()

    def _imu_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(self.imu_listen_addr)
        s.settimeout(1.0)

        while not self.stop_event.is_set():
            try:
                d, _ = s.recvfrom(1024)
                self.latest_imu_data = json.loads(d.decode("utf-8"))
            except socket.timeout:
                continue
            except Exception:
                self.latest_imu_data = None

        s.close()

    def run(self):
        vt = threading.Thread(target=self._video_loop, daemon=True)
        it = threading.Thread(target=self._imu_loop, daemon=True)
        vt.start()
        it.start()
        vt.join()
        it.join()

    def stop(self):
        self.stop_event.set()


class CombinedVisualizer:
    """Manages all GUI elements (Pygame and Matplotlib) in a single window."""

    def __init__(self, data_receiver):
        self.data_receiver = data_receiver
        self.screen_width = 1920  # Increase width for third panel
        self.camera_width = 640
        self.screen_height = 480
        self.plot_width = (self.screen_width - self.camera_width) // 2  # Split remaining space
        self.comparison_width = self.plot_width

        self.active_input = None  # Which input box is active: 'steps' or 'delay'
        self.steps_text = "5"  # Current steps input text
        self.delay_text = "0"  # Current delay input text
        self.input_values = {  # Return values for controller
            "num_steps": 5,
            "delay_ms": 0.0,
        }

        # Input box rectangles
        # self.steps_box = pygame.Rect(self.camera_width + 10, 350, 80, 25)
        # self.delay_box = pygame.Rect(self.camera_width + 10, 385, 80, 25)
        self.steps_box = pygame.Rect(160, 350, 80, 25)
        self.delay_box = pygame.Rect(160, 385, 80, 25)

        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Submarine Control Dashboard")
        self.font = pygame.font.SysFont("monospace", 16)
        self.font_bold = pygame.font.SysFont("monospace", 18, bold=True)

        # Pyramid vertices for 3D visualization
        self.pyramid_verts_base = np.array(
            [
                [-0.6, 0, 0],
                [0.2, -0.4, -0.4],
                [0.2, 0.4, -0.4],
                [0.2, 0.4, 0.4],
                [0.2, -0.4, 0.4],
            ]
        )

        self.pyramid_faces_indices = [
            [0, 1, 2],
            [0, 2, 3],
            [0, 3, 4],
            [0, 4, 1],
            [1, 4, 3, 2],
        ]
        self.pyramid_face_colors = ["cyan", "cyan", "cyan", "cyan", "red"]

        # Comparison plot data storage
        self.time_window_size = 30.0  # 30 second window
        history_length = int(self.time_window_size * 50)  # 30s at 50Hz
        self.start_time = time.time()

        # History deques for real submarine
        self.real_times = deque(maxlen=history_length)
        self.real_roll = deque(maxlen=history_length)
        self.real_pitch = deque(maxlen=history_length)
        self.real_yaw = deque(maxlen=history_length)

        # History deques for simulation
        self.sim_times = deque(maxlen=history_length)
        self.sim_roll = deque(maxlen=history_length)
        self.sim_pitch = deque(maxlen=history_length)
        self.sim_yaw = deque(maxlen=history_length)

        # Setup matplotlib plot
        plot_dpi = 100
        fig_size = (self.plot_width / plot_dpi, self.screen_height / plot_dpi)
        self.fig = plt.figure(figsize=fig_size, dpi=plot_dpi)
        self.ax_3d = self.fig.add_subplot(1, 1, 1, projection="3d")

        self.pyramid_collection = Poly3DCollection(
            [],
            facecolors=self.pyramid_face_colors,
            linewidths=0.8,
            edgecolors="k",
            alpha=0.6,
        )

        self.setup_3d_plot()
        self.rotation_fix = np.eye(3)
        # Live Sliding Window Plot
        self.setup_comparison_plot()

        self.hud_data = {
            "armed": False,
            "light_on": False,
            "stabilization": True,
            "sync_mode": 1,
            "yaw_kp": 0.0,
            "trans_scale": 0.8,
        }

    def setup_3d_plot(self):
        self.ax_3d.set_xlabel("X (Fwd)")
        self.ax_3d.set_ylabel("Y (Right)")
        self.ax_3d.set_zlabel("Z (Up)")

        self.ax_3d.set_xlim([-1, 1])
        self.ax_3d.set_ylim([-1, 1])
        self.ax_3d.set_zlim([-1, 1])

        try:
            self.ax_3d.set_box_aspect([1, 1, 1])
        except AttributeError:
            pass

        self.ax_3d.view_init(elev=25, azim=35)

        # Create frame lines for coordinate axes
        colors = ["red", "green", "blue"]
        labels = ["X (Roll)", "Y (Pitch)", "Z (Yaw)"]
        self.frame_lines = [
            self.ax_3d.plot([], [], [], c=c, lw=3, label=l)[0]
            for c, l in zip(colors, labels)
        ]

        self.ax_3d.legend()
        self.ax_3d.add_collection3d(self.pyramid_collection)
        self.fig.tight_layout(pad=0)

    def _quaternion_to_rotation_matrix(self, q):
        q0, q1, q2, q3 = q
        return np.array(
            [
                [
                    q0**2 + q1**2 - q2**2 - q3**2,
                    2 * (q1 * q2 - q0 * q3),
                    2 * (q1 * q3 + q0 * q2),
                ],
                [
                    2 * (q1 * q2 + q0 * q3),
                    q0**2 - q1**2 + q2**2 - q3**2,
                    2 * (q2 * q3 - q0 * q1),
                ],
                [
                    2 * (q1 * q3 - q0 * q2),
                    2 * (q2 * q3 + q0 * q1),
                    q0**2 - q1**2 - q2**2 + q3**2,
                ],
            ]
        )

    def handle_input_events(self, events):
        """Handle text input events. Call this from your main event loop."""
        updated_values = {}

        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                # --- Check for checkbox clicks ---
                for label, rect in self.checkboxes.items():
                    if rect.collidepoint(event.pos):
                        self.plot_visibility[label] = not self.plot_visibility[label]
                        # Prevent activating input boxes if a checkbox was clicked
                        self.active_input = None 
                        return {} # Exit early as we've handled the event

                # Check if clicked on input boxes
                if self.steps_box.collidepoint(event.pos):
                    self.active_input = "steps"
                elif self.delay_box.collidepoint(event.pos):
                    self.active_input = "delay"
                else:
                    self.active_input = None

            elif event.type == pygame.KEYDOWN and self.active_input:
                if event.key == pygame.K_RETURN or event.key == pygame.K_TAB:
                    # Submit current input
                    if self.active_input == "steps":
                        try:
                            new_steps = int(self.steps_text)
                            if 1 <= new_steps <= 100:
                                self.input_values["num_steps"] = new_steps
                                updated_values["num_steps"] = new_steps
                                print(f" Updated steps: {new_steps}")
                        except ValueError:
                            self.steps_text = str(self.input_values["num_steps"])

                    elif self.active_input == "delay":
                        try:
                            new_delay = float(self.delay_text)
                            if 0 <= new_delay <= 1000:
                                self.input_values["delay_ms"] = new_delay
                                updated_values["delay_ms"] = new_delay
                                print(f" Updated delay: {new_delay}ms")
                        except ValueError:
                            self.delay_text = str(self.input_values["delay_ms"])

                    self.active_input = None

                elif event.key == pygame.K_ESCAPE:
                    self.active_input = None

                elif event.key == pygame.K_BACKSPACE:
                    # Remove last character
                    if self.active_input == "steps":
                        self.steps_text = self.steps_text[:-1]
                    elif self.active_input == "delay":
                        self.delay_text = self.delay_text[:-1]

                else:
                    # Add character
                    char = event.unicode
                    if char.isprintable():
                        if self.active_input == "steps" and (
                            char.isdigit() or char == "."
                        ):
                            if len(self.steps_text) < 6:  # Limit length
                                self.steps_text += char
                        elif self.active_input == "delay" and (
                            char.isdigit() or char == "."
                        ):
                            if len(self.delay_text) < 8:  # Limit length
                                self.delay_text += char

        return updated_values

    def _draw_plot_checkboxes(self):
        """Draws the plot toggle checkboxes onto the screen."""
        for label, rect in self.checkboxes.items():
            # Draw the box
            pygame.draw.rect(self.screen, (200, 200, 200), rect, 1)
            
            # Draw the checkmark if visible
            if self.plot_visibility[label]:
                pygame.draw.line(self.screen, (0, 255, 128), (rect.x + 3, rect.y + 10), (rect.x + 8, rect.y + 15), 2)
                pygame.draw.line(self.screen, (0, 255, 128), (rect.x + 8, rect.y + 15), (rect.x + 15, rect.y + 5), 2)
            
            # Draw the label
            label_surface = self.font.render(label, 1, (0, 0, 0))
            self.screen.blit(label_surface, (rect.right + 10, rect.y + 2))

    def setup_comparison_plot(self):
        """Setup the RPY comparison plot."""
        # Position for comparison plot (rightmost panel)
        plot_dpi = 100
        fig_size = (self.comparison_width / plot_dpi, self.screen_height / plot_dpi)
        
        self.comparison_fig = plt.figure(figsize=fig_size, dpi=plot_dpi)
        self.comparison_ax = self.comparison_fig.add_subplot(1, 1, 1)
        
        # Plot lines for real submarine (solid lines)
        self.real_roll_line, = self.comparison_ax.plot([], [], 'r-', lw=2, label='Real Roll', alpha=0.8)
        self.real_pitch_line, = self.comparison_ax.plot([], [], 'g-', lw=2, label='Real Pitch', alpha=0.8)
        self.real_yaw_line, = self.comparison_ax.plot([], [], 'b-', lw=2, label='Real Yaw', alpha=0.8)
        
        # Plot lines for simulation (dashed lines)
        self.sim_roll_line, = self.comparison_ax.plot([], [], 'r--', lw=2, label='Sim Roll', alpha=0.6)
        self.sim_pitch_line, = self.comparison_ax.plot([], [], 'g--', lw=2, label='Sim Pitch', alpha=0.6)
        self.sim_yaw_line, = self.comparison_ax.plot([], [], 'b--', lw=2, label='Sim Yaw', alpha=0.6)
        
        self.comparison_ax.set_xlabel('Time (s)')
        self.comparison_ax.set_ylabel('Angle (degrees)')
        self.comparison_ax.set_title('Real vs Sim RPY Comparison')
        self.comparison_ax.legend(loc='upper left', fontsize=8)
        self.comparison_ax.set_ylim(-180, 180)
        self.comparison_ax.grid(True, alpha=0.3)
        self.comparison_ax.set_xlim(0, self.time_window_size)
        
        self.comparison_fig.tight_layout(pad=0.5)

        # --- Checkbox setup for plot visibility ---
        self.plot_visibility = {
            'Roll': True, 
            'Pitch': True, 
            'Yaw': True,
        }
        
        # Define checkbox rectangles and labels for Roll, Pitch, Yaw
        self.checkboxes = {}
        checkbox_x_start = self.camera_width + self.plot_width - 50
        # Adjust vertical start position for better centering
        checkbox_y_start = self.screen_height - 150 
        y_offset = 0
        for label in self.plot_visibility.keys():
            rect = pygame.Rect(checkbox_x_start, checkbox_y_start + y_offset, 20, 20)
            self.checkboxes[label] = rect
            y_offset += 35 # Increase vertical spacing between checkboxes

    def update_comparison_data(self, real_imu_data, sim_quaternion):
        """Update comparison plot data with latest readings."""
        current_time = time.time() - self.start_time
        
        # Process real submarine data
        if (real_imu_data and "orientation" in real_imu_data and 
            real_imu_data["orientation"] and "euler_deg" in real_imu_data["orientation"]):
            
            euler = real_imu_data["orientation"]["euler_deg"]
            self.real_times.append(current_time)
            self.real_roll.append(euler["roll"])
            self.real_pitch.append(euler["pitch"])
            self.real_yaw.append(euler["yaw"])
        
        # Process simulation data
        if sim_quaternion is not None and len(sim_quaternion) == 4:
            # Convert quaternion to euler (same as old code)
            w, x, y, z = sim_quaternion
            roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
            sinp = 2 * (w * y - z * x)
            pitch = np.arcsin(np.clip(sinp, -1, 1))
            yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
            
            self.sim_times.append(current_time)
            self.sim_roll.append(np.degrees(roll))
            self.sim_pitch.append(np.degrees(pitch))
            self.sim_yaw.append(np.degrees(yaw))

    def update_comparison_plot(self):
        """Update the comparison plot lines based on visibility for each axis."""
        
        # Update Roll plots (Real and Sim) based on a single 'Roll' checkbox
        if self.plot_visibility['Roll']:
            if len(self.real_times) > 1:
                self.real_roll_line.set_data(list(self.real_times), list(self.real_roll))
            if len(self.sim_times) > 1:
                self.sim_roll_line.set_data(list(self.sim_times), list(self.sim_roll))
        else:
            self.real_roll_line.set_data([], [])
            self.sim_roll_line.set_data([], [])

        # Update Pitch plots (Real and Sim) based on a single 'Pitch' checkbox
        if self.plot_visibility['Pitch']:
            if len(self.real_times) > 1:
                self.real_pitch_line.set_data(list(self.real_times), list(self.real_pitch))
            if len(self.sim_times) > 1:
                self.sim_pitch_line.set_data(list(self.sim_times), list(self.sim_pitch))
        else:
            self.real_pitch_line.set_data([], [])
            self.sim_pitch_line.set_data([], [])

        # Update Yaw plots (Real and Sim) based on a single 'Yaw' checkbox
        if self.plot_visibility['Yaw']:
            if len(self.real_times) > 1:
                self.real_yaw_line.set_data(list(self.real_times), list(self.real_yaw))
            if len(self.sim_times) > 1:
                self.sim_yaw_line.set_data(list(self.sim_times), list(self.sim_yaw))
        else:
            self.real_yaw_line.set_data([], [])
            self.sim_yaw_line.set_data([], [])
            
        # Update time window (this part remains the same)
        if len(self.real_times) > 1:
            times = list(self.real_times)
            if times and times[-1] > self.time_window_size:
                self.comparison_ax.set_xlim(times[-1] - self.time_window_size, times[-1])

    def update(self, imu_override: Optional[np.ndarray] = None):
        self.screen.fill((25, 25, 25))

        # Display camera feed (unchanged)
        with self.data_receiver.frame_lock:
            frame = self.data_receiver.latest_frame

        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            camera_surface = pygame.surfarray.make_surface(np.rot90(frame_rgb))
            self.screen.blit(camera_surface, (0, 0))

        # --- Enhanced HUD display ---
        sync_mode = self.hud_data.get("sync_mode", 1)
        sync_text = "SYNC" if sync_mode == 1 else "ASYNC"

        # ENHANCED: More prominent sync/async indicator with background
        sync_color = (
            (0, 255, 128) if sync_mode == 1 else (255, 165, 0)
        )  # Green for sync, orange for async
        bg_color = (0, 50, 25) if sync_mode == 1 else (50, 25, 0)  # Dark background

        # Draw background rectangle for mode indicator
        mode_rect = pygame.Rect(5, 5, 150, 30)
        pygame.draw.rect(self.screen, bg_color, mode_rect)
        pygame.draw.rect(self.screen, sync_color, mode_rect, 2)

        mode_surface = self.font_bold.render(f"MODE: {sync_text}", 1, sync_color)
        self.screen.blit(mode_surface, (10, 10))

        # NEW: Add explanation text
        explanation = "Waits for sim" if sync_mode == 1 else "Immediate response"
        exp_surface = self.font.render(explanation, 1, (180, 180, 180))
        self.screen.blit(exp_surface, (170, 15))

        # Armed Status (unchanged)
        armed_color = (255, 0, 0) if self.hud_data["armed"] else (0, 255, 0)
        self.screen.blit(
            self.font.render(f"ARMED: {self.hud_data['armed']}", 1, armed_color),
            (10, 45),
        )

        # IMPROVED: Show actual pending operations indicator
        pending_commands = self.hud_data.get("pending_commands", 0)
        if sync_mode == 0:  # Async mode
            if pending_commands > 0:
                # Show pending operations with count
                pending_indicator = f"⚡ ASYNC: {pending_commands} pending"
                color = (
                    (255, 255, 0) if pending_commands < 3 else (255, 128, 0)
                )  # Orange if many pending
            else:
                # Async mode but no pending operations
                pending_indicator = "⚡ ASYNC: Ready"
                color = (128, 255, 128)  # Light green

            pending_surface = self.font.render(pending_indicator, 1, color)
            self.screen.blit(pending_surface, (10, 420))
        else:
            # Sync mode indicator
            sync_indicator = " SYNC: Waiting for sim"
            sync_surface = self.font.render(sync_indicator, 1, (128, 200, 255))
            self.screen.blit(sync_surface, (10, 420))

        # Rest of HUD display (unchanged)
        light_text = "ON" if self.hud_data["light_on"] else "OFF"
        self.screen.blit(
            self.font.render(f"LIGHT: {light_text}", 1, (255, 255, 0)), (10, 65)
        )

        stab_text = "ON" if self.hud_data.get("stabilization", False) else "OFF"
        self.screen.blit(
            self.font.render(f"STABILIZE: {stab_text}", 1, (0, 255, 255)), (10, 85)
        )

        # Tuning Info
        self.screen.blit(
            self.font.render(
                f"Sim Kp: {self.hud_data['yaw_kp']:.2f}", 1, (255, 255, 255)
            ),
            (10, 110),  # Changed from 95
        )
        real_kp = self.hud_data.get("yaw_kp_real", 0.0)
        self.screen.blit(
            self.font.render(f"Real Kp: {real_kp:.2f}", 1, (255, 255, 255)), (10, 130)  # Changed from 115
        )
        trans_scale = self.hud_data.get("trans_scale", 0.8)
        self.screen.blit(
            self.font.render(f"Move Scale: {trans_scale:.2f}", 1, (255, 255, 255)),
            (10, 150),  # Changed from 135
        )

        controller_mode = self.hud_data.get("controller_mode", "JOYSTICK")
        mode_color = (0, 255, 0) if controller_mode == "JOYSTICK" else (255, 165, 0)
        self.screen.blit(
            self.font.render(f"CTRL: {controller_mode}", 1, mode_color), (10, 170)  # Changed from 155
        )

        # Get IMU data from real sub or use override from simulator
        imu_data = self.data_receiver.latest_imu_data
        if imu_data is None and imu_override is not None:
            w, x, y, z = imu_override
            r = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
            p = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
            y_angle = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

            imu_data = {
                "orientation": {
                    "quaternion": imu_override,
                    "euler_deg": {
                        "roll": np.degrees(r),
                        "pitch": np.degrees(p),
                        "yaw": np.degrees(y_angle),
                    },
                },
                "depth": None,
            }

        # Display attitude information
        # if imu_data and "euler_deg" in imu_data:
        if (
            imu_data
            and "orientation" in imu_data
            and imu_data["orientation"]
            and "euler_deg" in imu_data["orientation"]
        ):

            pygame.draw.line(self.screen, (100, 100, 100), (10, 195), (200, 195), 1) # Changed from 160
            roll_text = f"Roll:  {imu_data['orientation']['euler_deg']['roll']:6.1f}°"
            self.screen.blit(self.font.render(roll_text, 1, (255, 255, 255)), (10, 205)) # Changed from 170

            pitch_text = f"Pitch: {imu_data['orientation']['euler_deg']['pitch']:6.1f}°"
            self.screen.blit(
                self.font.render(pitch_text, 1, (255, 255, 255)), (10, 225) # Changed from 190
            )

            yaw_text = f"Yaw:   {imu_data['orientation']['euler_deg']['yaw']:6.1f}°"
            self.screen.blit(self.font.render(yaw_text, 1, (255, 255, 255)), (10, 245)) # Changed from 210

        # Add depth display section:
        if imu_data and "depth" in imu_data and imu_data["depth"]:
            depth_data = imu_data["depth"]
            depth_text = f"Depth: {depth_data['depth']['relative_cm']:6.1f}cm"
            self.screen.blit(self.font.render(depth_text, 1, (0, 255, 255)), (10, 265)) # Changed from 230

            pressure_text = f"Press: {depth_data['pressure']['value']:6.1f}{depth_data['pressure']['units']}"
            self.screen.blit(
                self.font.render(pressure_text, 1, (255, 255, 0)), (10, 285) # Changed from 250
            )

            temp_text = f"Temp:  {depth_data['temperature']['value']:6.1f}°C"
            self.screen.blit(self.font.render(temp_text, 1, (255, 255, 0)), (10, 305)) # Changed from 270

        # Update 3D visualization
        if (
            imu_data
            and "orientation" in imu_data
            and imu_data["orientation"]
            and "quaternion" in imu_data["orientation"]
            and not np.any(np.isnan(imu_data["orientation"]["quaternion"]))
        ):

            # q = np.array(imu_data["quaternion"])
            q = np.array(imu_data["orientation"]["quaternion"])
            R = self.rotation_fix @ self._quaternion_to_rotation_matrix(q)
            x_ax, y_ax, z_ax = R[:, 0], R[:, 1], R[:, 2]

            # Update coordinate frame lines
            self.frame_lines[0].set_data([0, x_ax[0]], [0, x_ax[1]])
            self.frame_lines[0].set_3d_properties([0, x_ax[2]])

            self.frame_lines[1].set_data([0, y_ax[0]], [0, y_ax[1]])
            self.frame_lines[1].set_3d_properties([0, y_ax[2]])

            self.frame_lines[2].set_data([0, z_ax[0]], [0, z_ax[1]])
            self.frame_lines[2].set_3d_properties([0, z_ax[2]])

            # Update pyramid
            rotated_verts = self.pyramid_verts_base @ R.T
            pyramid_faces = [
                [rotated_verts[i] for i in face] for face in self.pyramid_faces_indices
            ]
            self.pyramid_collection.set_verts(pyramid_faces)
            self.ax_3d.set_title("3D Orientation (Live)")
        else:
            self.pyramid_collection.set_verts([])
            self.ax_3d.set_title("3D Orientation (Waiting...)")

        # Update comparison data
        real_data = self.data_receiver.latest_imu_data
        self.update_comparison_data(real_data, imu_override)
        self.update_comparison_plot()


        # ADD: Live tuning input boxes (insert after existing HUD elements)
        # Draw input boxes
        steps_color = (
            (255, 255, 255) if self.active_input == "steps" else (100, 100, 100)
        )
        delay_color = (
            (255, 255, 255) if self.active_input == "delay" else (100, 100, 100)
        )

        # Steps input box
        pygame.draw.rect(self.screen, (50, 50, 50), self.steps_box)
        pygame.draw.rect(self.screen, steps_color, self.steps_box, 2)

        # Delay input box
        pygame.draw.rect(self.screen, (50, 50, 50), self.delay_box)
        pygame.draw.rect(self.screen, delay_color, self.delay_box, 2)

        # Labels
        self.screen.blit(self.font.render("Sim Steps:", 1, (255, 255, 255)), (10, 355))
        self.screen.blit(
            self.font.render("Sub Delay (ms):", 1, (255, 255, 255)), (10, 390)
        )

        # Input text
        steps_surface = self.font.render(self.steps_text, 1, (255, 255, 255))
        delay_surface = self.font.render(self.delay_text, 1, (255, 255, 255))

        self.screen.blit(steps_surface, (self.steps_box.x + 5, self.steps_box.y + 5))
        self.screen.blit(delay_surface, (self.delay_box.x + 5, self.delay_box.y + 5))

        # Cursor for active input
        if self.active_input == "steps":
            cursor_x = self.steps_box.x + 5 + steps_surface.get_width()
            pygame.draw.line(
                self.screen,
                (255, 255, 255),
                (cursor_x, self.steps_box.y + 5),
                (cursor_x, self.steps_box.y + 20),
                2,
            )
        elif self.active_input == "delay":
            cursor_x = self.delay_box.x + 5 + delay_surface.get_width()
            pygame.draw.line(
                self.screen,
                (255, 255, 255),
                (cursor_x, self.delay_box.y + 5),
                (cursor_x, self.delay_box.y + 20),
                2,
            )

        # Current values display
        current_steps = self.input_values["num_steps"]
        current_delay = self.input_values["delay_ms"]

        self.screen.blit(
            self.font.render(
                f"Active: {current_steps} steps, {current_delay:.0f}ms delay",
                1,
                (0, 255, 255),
            ),
            (self.camera_width + 10, 420),
        )

        # Instructions
        if self.active_input:
            self.screen.blit(
                self.font.render(
                    "Press ENTER to apply, ESC to cancel", 1, (255, 255, 0)
                ),
                (self.camera_width + 10, 440),
            )
        else:
            self.screen.blit(
                self.font.render("Click boxes to edit values", 1, (128, 128, 128)),
                (self.camera_width + 10, 440),
            )

        # Render matplotlib plot to pygame surface
        self.fig.canvas.draw()
        plot_surface = pygame.image.frombuffer(
            self.fig.canvas.buffer_rgba(), self.fig.canvas.get_width_height(), "RGBA"
        )
        self.screen.blit(plot_surface, (self.camera_width, 0))

        # Render comparison plot
        self.comparison_fig.canvas.draw()
        comparison_surface = pygame.image.frombuffer(
            self.comparison_fig.canvas.buffer_rgba(), 
            self.comparison_fig.canvas.get_width_height(), "RGBA"
        )
        self.screen.blit(comparison_surface, (self.camera_width + self.plot_width, 0))

        self._draw_plot_checkboxes()

        pygame.display.flip()


class UDPClientProtocol(asyncio.DatagramProtocol):
    def __init__(self, on_con_lost: asyncio.Future):
        self.transport = None
        self.on_con_lost = on_con_lost
        self.recv_queue = asyncio.Queue()

    def connection_made(self, transport: asyncio.DatagramTransport):
        self.transport = transport

    def datagram_received(self, data: bytes, addr: Tuple[str, int]):
        self.recv_queue.put_nowait(data)

    def error_received(self, exc: Exception):
        print(f"UDP connection error: {exc}")

    def connection_lost(self, exc: Exception):
        if not self.on_con_lost.done():
            self.on_con_lost.set_result(True)


class AsyncAUV:
    def __init__(self, server_ip="127.0.0.1", server_port=60001):
        self.server_ip = server_ip
        self.server_port = server_port
        self.transport = None
        self.protocol = None
        self.connection_lock = asyncio.Lock()
        self.connected = False

    async def _ensure_transport(self):
        async with self.connection_lock:
            if self.transport and not self.transport.is_closing():
                return
            try:
                if self.transport:
                    self.transport.close()
                loop = asyncio.get_running_loop()
                self.protocol = UDPClientProtocol(loop.create_future())
                self.transport, _ = await loop.create_datagram_endpoint(
                    lambda: self.protocol,
                    remote_addr=(self.server_ip, self.server_port),
                )
                self.connected = True
                print(
                    f"AsyncAUV: UDP transport ready for {self.server_ip}:{self.server_port}"
                )
            except Exception as e:
                self.connected = False
                print(f"AsyncAUV: UDP transport creation failed: {e}")
                raise

    async def _send_command(self, data: bytes) -> Optional[bytes]:
        for attempt in range(3):
            try:
                await self._ensure_transport()
                while not self.protocol.recv_queue.empty():
                    self.protocol.recv_queue.get_nowait()
                self.transport.sendto(data)
                return await asyncio.wait_for(
                    self.protocol.recv_queue.get(), timeout=2.0
                )
            except asyncio.TimeoutError:
                if attempt == 2:
                    print(f"AsyncAUV: Command timed out after 3 attempts")
            except Exception as e:
                print(f"AsyncAUV: Unexpected error: {e}")
                self.connected = False
                await asyncio.sleep(0.1)
        return None

    def _parse_sensor_response(self, response: bytes) -> Optional[dict]:
        """Parses a server response that contains sensor data."""
        if not response:
            return None

        # Check for status byte
        if len(response) == ACK_WITH_SENSOR_DATA_RECV_BYTES and response[0] == 0x00:
            payload = response[1:]
        elif len(response) == SENSOR_DATA_RECV_BYTES:
            payload = response
        else:
            return None

        try:
            values = struct.unpack("<10f", payload)
            return {
                "imu_quaternion": np.array(values[:4]),
                "time": values[4],
                "depth": values[5],  # ADD DEPTH VALUE
                "bounding_box": {
                    "x": values[6],      # Updated indices
                    "y": values[7], 
                    "width": values[8],
                    "height": values[9]
                }
            }
        except struct.error:
            print(f"Error unpacking sensor response: expected 40 bytes, got {len(payload)} bytes")
            return None

    async def get_sensor_data(self) -> Optional[dict]:
        data = struct.pack("<f", float(MsgHeader.GET_SENSORDATA.value))
        response = await self._send_command(data)
        return self._parse_sensor_response(response)

    async def apply_ctrl(
        self, forces: np.ndarray, num_steps: int = 1, sync_flag: int = 1
    ) -> Optional[dict]:
        if len(forces) != 6:
            raise ValueError("Forces array must have 6 elements")
        data = struct.pack(
            "<9f",
            float(MsgHeader.APPLY_CTRL.value),
            float(num_steps),
            float(sync_flag),
            *forces,
        )
        response = await self._send_command(data)
        return self._parse_sensor_response(response)

    async def step_sim(self, num_steps: int = 1, sync_flag: int = 1) -> Optional[dict]:
        """Commands the simulator to step forward without new forces."""
        # Format: [header, num_steps, sync_flag] = 3 floats
        data = struct.pack(
            "<3f", float(MsgHeader.STEP_SIM.value), float(num_steps), float(sync_flag)
        )
        response = await self._send_command(data)
        return self._parse_sensor_response(response)

    async def reset(self, num_steps: int = 0, sync_flag: int = 1) -> Optional[dict]:
        pose = np.array([10.25, 6.71, 0, 1, 0, 0, 0])
        vel = np.zeros(3)
        ang_vel = np.zeros(3)
        # FIX: Correct format string to pack 16 floats
        # Header (1) + nsteps (1) + sync (1) + pose (7) + vel (3) + ang_vel (3) = 16
        data = struct.pack(
            "<16f",
            float(MsgHeader.RESET.value),
            float(num_steps),
            float(sync_flag),
            *pose,
            *vel,
            *ang_vel,
        )
        response = await self._send_command(data)
        return self._parse_sensor_response(response)

    async def close(self):
        if self.transport:
            self.transport.close()
        self.connected = False


class RealSubmarineClient:
    def __init__(self, server_ip, control_port=10000):
        self.server_address = (server_ip, control_port)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
        print(f"RealSubClient: Ready for {self.server_address}")
        self.NEUTRAL_PULSE, self.LIGHT_OFF, self.LIGHT_ON = 1500, 1100, 1900
        self.AMP, self.LOOP_DURATION = 300, 0.02

    def _to_pwm(self, nf):
        return int(self.NEUTRAL_PULSE + nf * self.AMP)

    def _send_packet(self, cmd_dict):
        try:
            self.control_socket.sendto(
                (json.dumps(cmd_dict) + "\n").encode("utf-8"), self.server_address
            )
        except Exception as e:
            print(f"RealSubClient Error: {e}", file=sys.stderr)

    def send_control(self, armed, light_on, forces):
        forces = np.zeros(6) if not armed else np.clip(forces / 100.0, -1.0, 1.0)
        cmd = {
            "command": "control",
            "thruster_pulses": [self._to_pwm(v) for v in forces],
            "light_pulse": self.LIGHT_ON if light_on else self.LIGHT_OFF,
            "duration": self.LOOP_DURATION,
        }
        self._send_packet(cmd)

    def send_reset_command(self):
        print(f"Sending 'reset_orientation' command to {self.server_address}...")
        self._send_packet({"command": "reset_orientation"})

    def shutdown(self):
        print("RealSubClient: Shutting down.")
        self.send_control(False, False, np.zeros(6))
        time.sleep(0.1)
        self.control_socket.close()


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


@dataclass
class PIDState:
    prev_error: float = 0.0
    integral: float = 0.0


class BlueROVController:
    def __init__(self, auv_ip, auv_port, sub_ip, initial_sync_mode: int):
        self.auv = AsyncAUV(auv_ip, auv_port)
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad connected.")
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print(f"Gamepad: {self.controller.get_name()}")

        self.real_sub_client = RealSubmarineClient(sub_ip)
        self.data_receiver_thread = SubmarineDataReceiverThread()

        # External controller state
        self.external_controller_active = False
        self.external_controller = None

        if EXTERNAL_CONTROLLER_AVAILABLE:
            try:
                self.external_controller = MyController()
                print(" External controller initialized and ready")
            except Exception as e:
                print(f" Failed to initialize external controller: {e}")
                self.external_controller = None

        # Control state
        self.armed = False
        self.light_on = False
        self.stabilization_enabled = True
        self.sync_mode = initial_sync_mode  # 1 for Sync, 0 for Async
        self.saved_sync_mode = self.sync_mode  # Save original sync mode
        print(f"Starting in {'SYNC' if self.sync_mode == 1 else 'ASYNC'} mode.")

        self.minForce, self.maxForce = np.full(6, -100), np.full(6, 100)
        self.manual_force_scale, self.translation_scale, self.rotation_scale = (
            100.0,
            0.8,
            0.4,
        )

        self.pid_gains = {
            "roll": PIDGains(0, 0, 0),
            "pitch": PIDGains(0, 0, 0),
            "yaw": PIDGains(0, 0, 0),
            "yaw_sim": PIDGains(0.1, 0, 0),
            "yaw_real": PIDGains(10550.0, 0, 0),
        }
        self.pid_states = {k: PIDState() for k in self.pid_gains}
        self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])

        self.kp_step, self.translation_scale_step = 0.1, 0.02
        self.translation_scale_min, self.translation_scale_max = 0.05, 0.8

        # Controller button/axis mappings
        self.LEFT_STICK_X, self.LEFT_STICK_Y, self.RIGHT_STICK_X = 0, 1, 2
        self.LEFT_TRIGGER, self.RIGHT_TRIGGER = 4, 5
        self.BTN_A, self.BTN_B, self.BTN_X, self.BTN_Y = 0, 1, 2, 3
        self.BTN_BACK, self.BTN_MENU, self.BTN_LEFT_STICK, self.BTN_LB, self.BTN_RB = (
            4,
            6,
            7,
            9,
            10,
        )
        self.BTN_HAT_UP, self.BTN_HAT_DOWN, self.BTN_HAT_LEFT, self.BTN_HAT_RIGHT = (
            11,
            12,
            13,
            14,
        )
        self.BTN_MIDDLE = 15

        self.DEADZONE, self.dt, self.last_tuning_time = 0.1, 0.02, 0.0
        self.loop_times, self.network_times = [], []

        # ADD: Live tuning variables
        self.live_num_steps = 5
        self.live_delay_ms = 0.0
        print("CONTROLS:")
        print("   MENU    - Arm/Disarm")
        print("   BACK    - Toggle External Controller")
        print("   A       - Reset Orientation & Depth")
        print("   B       - Toggle Light")
        print("   X       - Toggle Stabilization")
        print("   LB      - Toggle Sync/Async Mode")
        if EXTERNAL_CONTROLLER_AVAILABLE:
            print("~~External Controller Available!")
        print("")

    def apply_deadzone(self, v: float) -> float:
        if abs(v) < self.DEADZONE:
            return 0.0
        return np.sign(v) * (abs(v) - self.DEADZONE) / (1.0 - self.DEADZONE)

    def quaternion_to_euler(self, q: np.ndarray) -> Tuple[float, float, float]:
        w, x, y, z = q
        r = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        p = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
        y_angle = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return r, p, y_angle

    def get_controller_input(self) -> Tuple[float, float, float, float]:
        if not self.armed:
            return 0.0, 0.0, 0.0, 0.0
        surge = (
            -self.apply_deadzone(self.controller.get_axis(self.LEFT_STICK_Y))
            * self.translation_scale
        )
        strafe = (
            self.apply_deadzone(self.controller.get_axis(self.LEFT_STICK_X))
            * self.translation_scale
        )
        yaw_cmd = (
            -self.apply_deadzone(self.controller.get_axis(self.RIGHT_STICK_X))
            * self.rotation_scale
        )
        heave = (
            (
                self.controller.get_axis(self.RIGHT_TRIGGER)
                - self.controller.get_axis(self.LEFT_TRIGGER)
            )
            * 0.5
            * self.translation_scale
        )
        return surge, strafe, heave, yaw_cmd

    def get_external_controller_input(self) -> Tuple[float, float, float, float]:
        """Get control input from external controller."""
        if not self.external_controller:
            return 0.0, 0.0, 0.0, 0.0
        try:
            # Get current sensor data
            real_data = self.data_receiver_thread.latest_imu_data
            
            # Don't try to get sim data synchronously from within async context
            # Instead, use the last known sim data from the main loop
            sim_data = getattr(self, '_last_sim_data', None)
            
            image_frame = None
            with self.data_receiver_thread.frame_lock:
                if self.data_receiver_thread.latest_frame is not None:
                    image_frame = self.data_receiver_thread.latest_frame.copy()
            
            # Extract depth measurement (prioritize real submarine depth over sim depth)
            depth_measurement = None
            if (
                real_data
                and "depth" in real_data
                and real_data["depth"]
                and "depth" in real_data["depth"]
            ):
                depth_measurement = real_data["depth"]["depth"]
            elif sim_data and "depth" in sim_data:
                depth_measurement = sim_data["depth"]
            
            # Extract bounding box data from simulator
            bbox_data = None
            if sim_data and "bounding_box" in sim_data:
                bbox_data = sim_data["bounding_box"]
            
            surge, strafe, heave, yaw_cmd = self.external_controller.update_loop(
                image_frame, real_data, depth_measurement, bbox_data
            )
            
            # Apply scaling
            surge *= self.translation_scale
            strafe *= self.translation_scale
            heave *= self.translation_scale
            yaw_cmd *= self.rotation_scale
            
            return surge, strafe, heave, yaw_cmd
            
        except Exception as e:
            print(f" External controller error: {e}")
            return 0.0, 0.0, 0.0, 0.0

    def calculate_stabilization_forces(
        self, current_attitude: np.ndarray, system_type: str
    ) -> np.ndarray:
        if current_attitude is None or np.any(np.isnan(current_attitude)):
            return np.zeros(6)
        _, _, current_yaw = self.quaternion_to_euler(current_attitude)
        _, _, target_yaw = self.quaternion_to_euler(self.target_attitude)
        if np.isnan(current_yaw) or np.isnan(target_yaw):
            return np.zeros(6)

        yaw_error_deg = (np.degrees(target_yaw - current_yaw) + 180) % 360 - 180

        key = "yaw_sim" if system_type == "sim" else "yaw_real"
        g, s = self.pid_gains[key], self.pid_states[key]

        s.integral = np.clip(s.integral + yaw_error_deg * self.dt, -10.0, 10.0)
        derivative = (yaw_error_deg - s.prev_error) / self.dt
        yaw_correction = g.kp * yaw_error_deg + g.ki * s.integral + g.kd * derivative
        s.prev_error = yaw_error_deg
        if np.isnan(yaw_correction):
            yaw_correction = 0.0

        # Thruster mapping for yaw correction
        mapping = [1, -1, -1, 1, 0, 0] if system_type == "sim" else [1, -1, 1, -1, 0, 0]
        stab_forces = np.array(mapping) * yaw_correction
        return np.clip(stab_forces * 0.1, -15.0, 15.0)

    def calculate_thruster_forces(
        self, surge, strafe, heave, yaw_cmd, system_type: str, stab_forces: np.ndarray
    ) -> np.ndarray:
        if system_type == "sim":
            base_forces = np.array(
                [
                    surge - strafe + yaw_cmd,
                    surge + strafe - yaw_cmd,
                    -surge - strafe - yaw_cmd,
                    -surge + strafe + yaw_cmd,
                    heave,
                    heave,
                ]
            )
        else:  # 'real'
            base_forces = np.array(
                [
                    surge - strafe + yaw_cmd,
                    surge + strafe - yaw_cmd,
                    surge + strafe + yaw_cmd,
                    surge - strafe - yaw_cmd,
                    heave,
                    -heave,
                ]
            )
        total_forces = base_forces * self.manual_force_scale + (
            stab_forces if self.stabilization_enabled else 0
        )
        return np.clip(total_forces, self.minForce, self.maxForce)

    def get_external_controller_input(self) -> Tuple[float, float, float, float]:
        """Get control input from external controller."""
        if not self.external_controller:
            return 0.0, 0.0, 0.0, 0.0
        try:
            # Get current sensor data
            real_data = self.data_receiver_thread.latest_imu_data
            sim_data = None
            try:
                # Create a new event loop task to get sim data
                loop = asyncio.get_event_loop()
                sim_data = loop.run_until_complete(
                    asyncio.wait_for(self.auv.get_sensor_data(), timeout=0.01)
                )
            except (asyncio.TimeoutError, Exception):
                sim_data = None
                
            image_frame = None
            with self.data_receiver_thread.frame_lock:
                if self.data_receiver_thread.latest_frame is not None:
                    image_frame = self.data_receiver_thread.latest_frame.copy()
            
            # Extract depth measurement (prioritize real submarine depth over sim depth)
            depth_measurement = None
            if (
                real_data
                and "depth" in real_data
                and real_data["depth"]
                and "depth" in real_data["depth"]
            ):
                depth_measurement = real_data["depth"]["depth"]
            elif sim_data and "depth" in sim_data:
                depth_measurement = sim_data["depth"]
            
            # Extract bounding box data from simulator
            bbox_data = None
            if sim_data and "bounding_box" in sim_data:
                bbox_data = sim_data["bounding_box"]
            
            surge, strafe, heave, yaw_cmd = self.external_controller.update_loop(
                image_frame, real_data, depth_measurement, bbox_data
            )
            
            # Apply scaling (external controller returns -1 to 1, scale to our system)
            surge *= self.translation_scale
            strafe *= self.translation_scale
            heave *= self.translation_scale
            yaw_cmd *= self.rotation_scale
            
            return surge, strafe, heave, yaw_cmd
            
        except Exception as e:
            print(f" External controller error: {e}")
            return 0.0, 0.0, 0.0, 0.0

    def toggle_external_controller(self):
        """Toggle between joystick and external controller."""
        if not EXTERNAL_CONTROLLER_AVAILABLE or not self.external_controller:
            print(" External controller not available")
            return

        self.external_controller_active = not self.external_controller_active

        if self.external_controller_active:
            # Switch to external controller
            print(" EXTERNAL CONTROLLER ACTIVATED")
            print("   - Controller will run in SYNC mode for safety")
            print("   - Joystick still active for emergency override")

            # Force sync mode for external controller
            self.saved_sync_mode = self.sync_mode
            self.sync_mode = 1

        else:
            # Switch back to joystick
            print(" JOYSTICK CONTROLLER ACTIVATED")

            # Restore original sync mode
            self.sync_mode = self.saved_sync_mode

    async def send_real_sub_with_delay(self, armed, light_on, forces, delay_ms):
        """Send real submarine command with live-tunable delay"""
        # Apply live delay
        if delay_ms > 0:
            await asyncio.sleep(delay_ms / 1000.0)

        # Send command
        await asyncio.get_event_loop().run_in_executor(
            None, self.real_sub_client.send_control, armed, light_on, forces
        )

    async def run_controller_async(self, num_steps, fwcmd, runtime):
        print("🚀 Starting ASYNC controller loop. Press MENU to arm.")
        self.visualizer = CombinedVisualizer(self.data_receiver_thread)
        self.data_receiver_thread.start()

        shutdown = False
        loop_count = 0
        cmdstarttime = time.time()

        try:
            while not shutdown:
                loop_start = time.time()
                if time.time() - cmdstarttime > runtime + 0.1:
                    shutdown = True

                events = pygame.event.get()
                for e in events:
                    if e.type == pygame.QUIT:
                        shutdown = True
                        break
                    if e.type == pygame.JOYBUTTONDOWN:
                        # --- Handle Button Presses ---
                        if e.button == self.BTN_MENU:
                            self.armed = not self.armed
                            print(f"\n[{'ARMED' if self.armed else 'DISARMED'}]")
                        elif e.button == self.BTN_BACK:
                            print("\n Toggling controller mode...")
                            self.toggle_external_controller()
                        elif e.button == self.BTN_X:
                            self.stabilization_enabled = not self.stabilization_enabled
                            print(
                                f"\nStabilization: {'ON' if self.stabilization_enabled else 'OFF'}"
                            )
                        elif e.button == self.BTN_B:
                            self.light_on = not self.light_on
                            print(f"\nLIGHT: {'ON' if self.light_on else 'OFF'}")
                        elif e.button == self.BTN_LB:
                            self.sync_mode = 1 - self.sync_mode
                            print(
                                f"\nMode switched to {'SYNC' if self.sync_mode else 'ASYNC'}"
                            )
                        elif e.button == self.BTN_MIDDLE:
                            raise KeyboardInterrupt("Emergency stop")
                        elif e.button == self.BTN_LEFT_STICK:
                            print("\nResetting Orientation (Target Only)...")
                            self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])
                        elif e.button == self.BTN_A:
                            print("\nSending RESET signal to Sub & Sim...")
                            self.target_attitude = np.array([1.0, 0, 0, 0])
                            if self.real_sub_client:
                                self.real_sub_client.send_reset_command()
                            asyncio.create_task(
                                self.auv.reset(sync_flag=self.sync_mode)
                            )

                        # --- D-pad Tuning ---
                        tuned = False
                        if e.button == self.BTN_HAT_UP:
                            self.pid_gains["yaw_sim"].kp += self.kp_step
                            tuned = True
                        if e.button == self.BTN_HAT_DOWN:
                            self.pid_gains["yaw_sim"].kp = max(
                                0.0, self.pid_gains["yaw_sim"].kp - self.kp_step
                            )
                            tuned = True
                        if e.button == self.BTN_HAT_LEFT:
                            self.pid_gains["yaw_real"].kp = max(
                                0.0, self.pid_gains["yaw_real"].kp - self.kp_step
                            )
                            tuned = True
                        if e.button == self.BTN_HAT_RIGHT:
                            self.pid_gains["yaw_real"].kp += self.kp_step
                            tuned = True
                        if tuned:
                            print(
                                f"\n🔧 Tuned: SimKp={self.pid_gains['yaw_sim'].kp:.2f}, RealKp={self.pid_gains['yaw_real'].kp:.2f}"
                            )

                updated_values = self.visualizer.handle_input_events(events)

                # Handle values from input boxes
                if "num_steps" in updated_values:
                    self.live_num_steps = updated_values["num_steps"]
                if "delay_ms" in updated_values:
                    self.live_delay_ms = updated_values["delay_ms"]

                if shutdown:
                    break

                if self.external_controller_active:
                    # External controller (always armed when active)
                    surge, strafe, heave, yaw_cmd = self.get_external_controller_input()

                    # Safety: Allow joystick override if significant input detected
                    joystick_surge, joystick_strafe, joystick_heave, joystick_yaw = (
                        self.get_controller_input()
                    )

                    # Check for emergency override (any significant joystick input)
                    joystick_magnitude = (
                        abs(joystick_surge)
                        + abs(joystick_strafe)
                        + abs(joystick_heave)
                        + abs(joystick_yaw)
                    )
                    if joystick_magnitude > 0.1:  # Threshold for override
                        print(" JOYSTICK OVERRIDE DETECTED - Using manual control")
                        surge, strafe, heave, yaw_cmd = (
                            joystick_surge,
                            joystick_strafe,
                            joystick_heave,
                            joystick_yaw,
                        )
                        self.external_controller_active = False  # Permanent TakeOver for safety
                        self.stabilization_enabled = False

                else:
                    # Normal joystick control
                    surge, strafe, heave, yaw_cmd = self.get_controller_input()

                real_data = self.data_receiver_thread.latest_imu_data

                current_attitude_real = (
                    np.array(real_data["orientation"]["quaternion"])
                    if real_data
                    and "orientation" in real_data
                    and real_data["orientation"]
                    and "quaternion" in real_data["orientation"]
                    else None
                )

                sim_data, current_attitude_sim = None, None
                try:  # Prioritize getting simulator data for the main loop
                    sim_data = await asyncio.wait_for(
                        self.auv.get_sensor_data(), timeout=0.005
                    )
                    if sim_data and "imu_quaternion" in sim_data:
                        current_attitude_sim = sim_data["imu_quaternion"]
                except asyncio.TimeoutError:
                    pass
                if current_attitude_sim is None:
                    await asyncio.sleep(0.001)
                    continue

                # Calculate forces and send commands
                sim_stab_f = self.calculate_stabilization_forces(
                    current_attitude_sim, "sim"
                )
                real_stab_f = self.calculate_stabilization_forces(
                    current_attitude_real, "real"
                )

                sim_f = self.calculate_thruster_forces(
                    surge, strafe, heave, yaw_cmd, "sim", sim_stab_f
                )
                real_f = self.calculate_thruster_forces(
                    surge, strafe, heave, yaw_cmd, "real", real_stab_f
                )

                tasks = [
                    self.auv.apply_ctrl(
                        sim_f if self.armed else np.zeros(6),
                        self.live_num_steps,
                        self.sync_mode,
                    )
                ]
                if self.real_sub_client:
                    tasks.append(
                        self.send_real_sub_with_delay(
                            self.armed, self.light_on, real_f, self.live_delay_ms
                        )
                    )

                network_start = time.time()

                try:  # SYNC/ASYNC
                    if self.sync_mode == 1:
                        print(f" SYNC: Waiting for {num_steps} steps to complete...")
                        # No timeout - let it take as long as needed
                        await asyncio.gather(*tasks, return_exceptions=True)
                    else:
                        print(" ASYNC: Fire and forget mode")
                        # Short timeout for async
                        await asyncio.wait_for(
                            asyncio.gather(*tasks, return_exceptions=True), timeout=0.1
                        )
                except asyncio.TimeoutError:
                    print(" ASYNC timeout (expected)")
                except Exception as e:
                    print(f"Network error: {e}")
                finally:
                    pass

                network_time = (time.time() - network_start) * 1000  # Convert to ms
                # print(f"Network time: {network_time:.1f}ms")
                self.network_times.append(network_time / 1000)  # Store in seconds

                ###################################

                # Update visualizer
                controller_mode = (
                    "EXTERNAL" if self.external_controller_active else "JOYSTICK"
                )
                self.visualizer.hud_data.update(
                    {
                        "armed": self.armed,
                        "light_on": self.light_on,
                        "stabilization": self.stabilization_enabled,
                        "sync_mode": self.sync_mode,
                        "controller_mode": controller_mode,
                        "yaw_kp": self.pid_gains["yaw_sim"].kp,
                        "yaw_kp_real": self.pid_gains["yaw_real"].kp,
                        "trans_scale": self.translation_scale,
                    }
                )

                self.visualizer.update(imu_override=current_attitude_sim)

                loop_time = time.time() - loop_start
                self.loop_times.append(loop_time)
                if loop_count % 100 == 0 and self.loop_times:
                    print(
                        f"Perf: Loop={np.mean(self.loop_times[-100:])*1000:.1f}ms, Net={np.mean(self.network_times[-100:])*1000:.1f}ms"
                    )
                loop_count += 1
                await asyncio.sleep(max(0, self.dt - loop_time))
        finally:
            print("Disarming and stopping...")
            self.data_receiver_thread.stop()
            self.data_receiver_thread.join(timeout=1)
            plt.close("all")
            try:
                # FIX: Send a final disarm in sync mode with the correct number of arguments
                await self.auv.apply_ctrl(np.zeros(6), 1, 1)
            except Exception as e:
                print(f"Could not cleanly close AUV connection: {e}")
            if self.real_sub_client:
                self.real_sub_client.shutdown()
            pygame.quit()
            if self.loop_times:
                print(
                    f"Final Perf - Loop: {np.mean(self.loop_times)*1000:.1f}ms, Net: {np.mean(self.network_times)*1000:.1f}ms"
                )

    def run_controller(self, *args):
        asyncio.run(self.run_controller_async(*args))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Async BlueROV2 Controller with High Performance Networking"
    )
    parser.add_argument(
        "--ip", type=str, default="127.0.0.1", help="IP of the MuJoCo/Unity simulator."
    )
    parser.add_argument(
        "--port", type=int, default=60001, help="Port of the MuJoCo/Unity server."
    )
    parser.add_argument(
        "--num_steps", type=int, default=1, help="Simulator physics steps per cycle."
    )
    parser.add_argument(
        "--sub_ip", type=str, default="192.168.2.11", help="IP of real submarine."
    )
    parser.add_argument(
        "--fwcmd", type=float, default=2.0, help="Time duration to apply forward force."
    )
    parser.add_argument(
        "--runtime", type=float, default=5.0, help="Time duration to send commands."
    )
    parser.add_argument(
        "--sync",
        type=int,
        default=1,
        choices=[0, 1],
        help="Run in synchronous (1) or asynchronous (0) mode. Default: 1",
    )
    args = parser.parse_args()

    try:
        controller = BlueROVController(args.ip, args.port, args.sub_ip, args.sync)
        controller.run_controller(args.num_steps, args.fwcmd, args.runtime)
    except Exception as e:
        print(f"\n[FATAL ERROR] Could not start: {e}", file=sys.stderr)
        pygame.quit()
        sys.exit(1)
