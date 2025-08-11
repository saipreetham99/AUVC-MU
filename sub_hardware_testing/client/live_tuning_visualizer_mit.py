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
    from external_controller import MyController, process_real_camera_frame

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
SENSOR_DATA_RECV_BYTES = (
    40  # 10 floats * 4 bytes/float (4 sensor + 1 time + 1 depth + 4 bbox)
)
ACK_WITH_SENSOR_DATA_RECV_BYTES = (
    41  # 1 byte status + 40 bytes sensor+time+depth+bbox data
)

# In live_tuning_visualizer.py


class SubmarineDataReceiverThread(threading.Thread):
    def __init__(self, video_port=10001, sim_video_port=60000, imu_port=10002):
        super().__init__(daemon=True, name="SubmarineDataReceiver")
        self.video_listen_addr = ("", video_port)
        self.sim_video_listen_addr = ("", sim_video_port)
        self.imu_listen_addr = ("", imu_port)
        self.latest_sim_frame = None
        self.latest_frame = None
        self.latest_imu_data = None
        self.frame_lock = threading.Lock()
        self.sim_frame_lock = threading.Lock()
        self.stop_event = threading.Event()

    def _sim_video_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(self.sim_video_listen_addr)
        s.settimeout(1.0)

        b = {}
        packet_counter = 0
        # Maximum number of incomplete frames to keep in memory
        MAX_BUFFERED_FRAMES = 50

        print(
            f"[DEBUG] SIM VIDEO: Listening for UDP packets on {self.sim_video_listen_addr}..."
        )

        while not self.stop_event.is_set():
            try:
                d, _ = s.recvfrom(65536)
                packet_counter += 1

                if len(d) < 6:
                    continue

                # Periodically clean up old, incomplete frames to prevent memory leak
                if packet_counter > 1000:
                    packet_counter = 0
                    if len(b) > MAX_BUFFERED_FRAMES:
                        # Find the oldest frame IDs and remove them
                        oldest_fids = sorted(b.keys())[:-MAX_BUFFERED_FRAMES]
                        for fid_to_del in oldest_fids:
                            del b[fid_to_del]

                fid, cs, cid = struct.unpack("!HHH", d[:6])
                if fid not in b:
                    b[fid] = {}

                b[fid][cid] = d[6:]
                if len(b[fid]) == cs:
                    jpeg = b"".join(v for k, v in sorted(b[fid].items()))
                    fr = cv2.imdecode(
                        np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                    )
                    with self.sim_frame_lock:
                        self.latest_sim_frame = fr
                    del b[fid]

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[ERROR] Sim video loop exception: {e}")
        s.close()

    def _video_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(self.video_listen_addr)
        s.settimeout(1.0)

        b = {}
        packet_counter = 0
        # Maximum number of incomplete frames to keep in memory
        MAX_BUFFERED_FRAMES = 50

        while not self.stop_event.is_set():
            try:
                d, _ = s.recvfrom(65536)
                packet_counter += 1

                if len(d) < 6:
                    continue

                # Periodically clean up old, incomplete frames
                if packet_counter > 1000:
                    packet_counter = 0
                    if len(b) > MAX_BUFFERED_FRAMES:
                        oldest_fids = sorted(b.keys())[:-MAX_BUFFERED_FRAMES]
                        for fid_to_del in oldest_fids:
                            del b[fid_to_del]

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
            except Exception as e:
                print(f"[ERROR] Real video loop exception: {e}")
        s.close()

    def _imu_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(self.imu_listen_addr)
        s.settimeout(1.0)
        while not self.stop_event.is_set():
            try:
                d, _ = s.recvfrom(1024)
                self.latest_imu_data = json.loads(d.decode("utf-8"))
            except (socket.timeout, json.JSONDecodeError):
                continue
            except Exception as e:
                print(f"[ERROR] IMU loop exception: {e}")
                self.latest_imu_data = None
        s.close()

    def run(self):
        vt = threading.Thread(target=self._video_loop, daemon=True)
        sim_vt = threading.Thread(target=self._sim_video_loop, daemon=True)
        it = threading.Thread(target=self._imu_loop, daemon=True)
        vt.start()
        sim_vt.start()
        it.start()
        vt.join()
        sim_vt.join()
        it.join()

    def stop(self):
        self.stop_event.set()


class CombinedVisualizer:
    """Manages all GUI elements (Pygame and Matplotlib) in a single window."""

    def __init__(self, data_receiver):
        self.data_receiver = data_receiver

        # --- FINAL, REFINED LAYOUT DEFINITION ---
        self.screen_width = 1280
        self.screen_height = 1080  # Increased height for taller graph

        # Define non-overlapping areas for each panel
        self.camera_area = pygame.Rect(0, 0, 640, 480)
        # Move status area to the right and make it narrower
        self.status_area = pygame.Rect(
            740, 0, 540, 240
        )  # Moved right by 100px, reduced width by 100px
        self.control_area = pygame.Rect(0, 480, 900, 280)  # Keep as is
        # Move 3D plot area to the right and make it narrower
        self.plot_3d_area = pygame.Rect(
            740, 240, 540, 540
        )  # Moved right by 100px, reduced width by 100px
        # Keep comparison plot area as is
        self.comparison_plot_area = pygame.Rect(0, 760, 1280, 320)

        self.active_input = None

        # --- INPUT BOX POSITIONS (Realigned and Resized) ---
        box_width, box_height = 75, 35
        row_1_y = self.control_area.y + 70
        row_2_y = self.control_area.y + 150
        row_3_y = self.control_area.y + 230

        col_1_x = self.control_area.x + 30
        col_2_x = self.control_area.x + 120
        col_3_x = self.control_area.x + 210

        # Multiplier columns after separator
        col_4_x = self.control_area.x + 320  # Surge
        col_5_x = self.control_area.x + 410  # Strafe
        col_6_x = self.control_area.x + 500  # Yaw
        col_7_x = self.control_area.x + 590  # Heave

        self.steps_box = pygame.Rect(col_1_x, row_1_y, box_width, box_height)
        self.delay_box = pygame.Rect(col_2_x, row_1_y, box_width, box_height)

        self.sim_kp_box = pygame.Rect(col_1_x, row_2_y, box_width, box_height)
        self.sim_ki_box = pygame.Rect(col_2_x, row_2_y, box_width, box_height)
        self.sim_kd_box = pygame.Rect(col_3_x, row_2_y, box_width, box_height)

        self.real_kp_box = pygame.Rect(col_1_x, row_3_y, box_width, box_height)
        self.real_ki_box = pygame.Rect(col_2_x, row_3_y, box_width, box_height)
        self.real_kd_box = pygame.Rect(col_3_x, row_3_y, box_width, box_height)

        # Sim multipliers row
        self.sim_surge_box = pygame.Rect(col_4_x, row_2_y, box_width, box_height)
        self.sim_strafe_box = pygame.Rect(col_5_x, row_2_y, box_width, box_height)
        self.sim_yaw_box = pygame.Rect(col_6_x, row_2_y, box_width, box_height)
        self.sim_heave_box = pygame.Rect(col_7_x, row_2_y, box_width, box_height)

        # Real multipliers row
        self.real_surge_box = pygame.Rect(col_4_x, row_3_y, box_width, box_height)
        self.real_strafe_box = pygame.Rect(col_5_x, row_3_y, box_width, box_height)
        self.real_yaw_box = pygame.Rect(col_6_x, row_3_y, box_width, box_height)
        self.real_heave_box = pygame.Rect(col_7_x, row_3_y, box_width, box_height)

        self.steps_text, self.delay_text = "7", "0"
        self.sim_kp_text, self.sim_ki_text, self.sim_kd_text = "10.0", "0.2", "0.1"
        self.real_kp_text, self.real_ki_text, self.real_kd_text = "1.0", "0.2", "0.1"
        
        # Depth PID text values
        self.sim_depth_kp_text = "0.5"
        self.sim_depth_ki_text = "0.01"
        self.sim_depth_kd_text = "0.2"
        self.real_depth_kp_text = "0.02"
        self.real_depth_ki_text = "0.001"
        self.real_depth_kd_text = "0.05"


        (
            self.sim_surge_text,
            self.sim_strafe_text,
            self.sim_yaw_text,
            self.sim_heave_text,
        ) = ("1.0", "1.0", "1.0", "1.0")
        (
            self.real_surge_text,
            self.real_strafe_text,
            self.real_yaw_text,
            self.real_heave_text,
        ) = ("1.0", "1.0", "1.0", "1.0")

        self.input_values = {
            "num_steps": 5,
            "delay_ms": 0.0,
            "depth_target": -1.0,  # Default to surface depth
            "sim_kp": 0.1,
            "sim_ki": 0.0,
            "sim_kd": 0.0,
            "real_kp": 10550.0,
            "real_ki": 0.0,
            "real_kd": 0.0,
            "sim_depth_kp": 0.5,     # Conservative defaults
            "sim_depth_ki": 0.01,
            "sim_depth_kd": 0.2,
            "real_depth_kp": 0.02,   # Very conservative for real sub
            "real_depth_ki": 0.001,
            "real_depth_kd": 0.05,
            "sim_surge_mult": 1.0,
            "sim_strafe_mult": 1.0,
            "sim_yaw_mult": 1.0,
            "sim_heave_mult": 1.0,
            "real_surge_mult": 1.0,
            "real_strafe_mult": 1.0,
            "real_yaw_mult": 1.0,
            "real_heave_mult": 1.0,
        }

        self.real_depth, self.sim_depth = None, None
        # Add depth target input box (after existing boxes)
        self.depth_target_box = pygame.Rect(col_3_x, row_1_y, box_width, box_height)
        self.depth_target_text = "-1.0"  # Default to surface
        
        # Depth PID boxes (new positions)
        depth_row_y = self.control_area.y + 110  # Between simulation control and PID
        self.sim_depth_kp_box = pygame.Rect(col_1_x, depth_row_y, box_width, box_height)
        self.sim_depth_ki_box = pygame.Rect(col_2_x, depth_row_y, box_width, box_height)
        self.sim_depth_kd_box = pygame.Rect(col_3_x, depth_row_y, box_width, box_height)
        self.real_depth_kp_box = pygame.Rect(col_1_x, depth_row_y + 40, box_width, box_height)
        self.real_depth_ki_box = pygame.Rect(col_2_x, depth_row_y + 40, box_width, box_height)
        self.real_depth_kd_box = pygame.Rect(col_3_x, depth_row_y + 40, box_width, box_height)

        # Rest of __init__ remains the same...
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Submarine Control Dashboard")

        # Fonts (Input font is now smaller)
        self.font = pygame.font.SysFont("monospace", 16)
        self.font_bold = pygame.font.SysFont("monospace", 18, bold=True)
        self.font_large = pygame.font.SysFont("monospace", 22, bold=True)
        self.font_input = pygame.font.SysFont("monospace", 18, bold=True)

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
        self.pyramid_face_colors = [
            "#00FFFF",
            "#00FFFF",
            "#00FFFF",
            "#00FFFF",
            "#FF4500",
        ]  # Cyan and Orange-Red
        self.rotation_fix = np.eye(3)

        self.time_window_size = 30.0
        history_length = int(self.time_window_size * 50)
        self.start_time = time.time()
        self.real_times, self.real_roll, self.real_pitch, self.real_yaw = [
            deque(maxlen=history_length) for _ in range(4)
        ]
        self.sim_times, self.sim_roll, self.sim_pitch, self.sim_yaw = [
            deque(maxlen=history_length) for _ in range(4)
        ]
        self.last_processed_real = None
        self.last_processed_sim = None

        self.setup_3d_plot()
        self.setup_comparison_plot()

        self.hud_data = {
            "armed": False,
            "light_on": False,
            "stabilization": True,
            "sync_mode": 1,
            "yaw_kp": 0.0,
            "trans_scale": 0.8,
        }

        self.show_sim_camera = False  # False = real camera, True = sim camera
        self.camera_toggle_checkbox = pygame.Rect(10, 10, 20, 20)  # Top-left checkbox

    def quaternion_to_euler(self, q: np.ndarray) -> Tuple[float, float, float]:
        w, x, y, z = q
        roll_rad = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch_rad = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
        yaw_rad = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return np.degrees(roll_rad), np.degrees(pitch_rad), np.degrees(yaw_rad)

    def update_3d_plot(self, imu_data: Optional[dict]):
        if (
            imu_data
            and "orientation" in imu_data
            and imu_data["orientation"]
            and "quaternion" in imu_data["orientation"]
            and not np.any(np.isnan(imu_data["orientation"]["quaternion"]))
        ):
            q = np.array(imu_data["orientation"]["quaternion"])
            R = self.rotation_fix @ self._quaternion_to_rotation_matrix(q)
            for i, axis_vec in enumerate(R.T):
                self.frame_lines[i].set_data([0, axis_vec[0]], [0, axis_vec[1]])
                self.frame_lines[i].set_3d_properties([0, axis_vec[2]])
            rotated_verts = self.pyramid_verts_base @ R.T
            self.pyramid_collection.set_verts(
                [
                    [rotated_verts[i] for i in face]
                    for face in self.pyramid_faces_indices
                ]
            )
            self.ax_3d.set_title("3D Orientation (Live)", color="white")
        else:
            self.pyramid_collection.set_verts([])
            self.ax_3d.set_title("3D Orientation (Waiting...)", color="gray")

    def update(self, imu_override: Optional[np.ndarray] = None):
        self.screen.fill((25, 25, 25))
        self.draw_camera_panel()
        self.draw_status_panel()
        self.draw_control_panel()

        imu_data = self.data_receiver.latest_imu_data
        if imu_data is None and imu_override is not None:
            roll, pitch, yaw = self.quaternion_to_euler(np.array(imu_override))
            imu_data = {
                "orientation": {
                    "quaternion": imu_override,
                    "euler_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
                },
                "depth": None,
            }

        self.update_3d_plot(imu_data)
        self.fig.canvas.draw()
        plot_surface = pygame.image.frombuffer(
            self.fig.canvas.buffer_rgba(), self.fig.canvas.get_width_height(), "RGBA"
        )
        self.screen.blit(plot_surface, self.plot_3d_area.topleft)

        self.update_comparison_data(imu_data, imu_override)
        self.update_comparison_plot()
        self.comparison_fig.canvas.draw()
        comparison_surface = pygame.image.frombuffer(
            self.comparison_fig.canvas.buffer_rgba(),
            self.comparison_fig.canvas.get_width_height(),
            "RGBA",
        )
        self.screen.blit(comparison_surface, self.comparison_plot_area.topleft)

        self._draw_plot_checkboxes()
        pygame.display.flip()

    def _draw_camera_panel(self):
        pygame.draw.rect(self.screen, (0, 0, 0), self.camera_area)

        frame_to_display = None

        # This logic is now fed by get_external_controller_input, which correctly
        # populates these variables with the processed frames from MyController.
        if self.show_sim_camera:
            # If the SIM camera checkbox is ticked, try to show the processed SIM frame.
            if hasattr(self, "last_processed_sim"):
                frame_to_display = self.last_processed_sim
        else:
            # Otherwise, try to show the processed REAL frame.
            if hasattr(self, "last_processed_real"):
                frame_to_display = self.last_processed_real

        # Display the selected frame
        if (
            frame_to_display is not None
            and frame_to_display.shape[0] > 0
            and frame_to_display.shape[1] > 0
        ):
            try:
                # Ensure the frame is in a displayable format (RGB)
                if len(frame_to_display.shape) == 3:
                    frame_rgb = cv2.cvtColor(frame_to_display, cv2.COLOR_BGR2RGB)
                    frame_resized = cv2.resize(frame_rgb, self.camera_area.size)
                    camera_surface = pygame.surfarray.make_surface(
                        np.rot90(frame_resized)
                    )
                    self.screen.blit(camera_surface, self.camera_area.topleft)
                else:
                    # Handle grayscale or other unexpected formats if necessary
                    raise ValueError("Frame is not in BGR format")
            except Exception as e:
                # Catch any conversion or drawing errors
                source_text = "SIM CAMERA" if self.show_sim_camera else "REAL CAMERA"
                error_text = self.font.render(
                    f"ERROR DISPLAYING {source_text}", 1, (255, 0, 0)
                )
                self.screen.blit(
                    error_text, error_text.get_rect(center=self.camera_area.center)
                )
                print(f"Error drawing camera panel: {e}")

        else:
            # If frame_to_display is None, show a waiting message
            source_text = "SIM CAMERA" if self.show_sim_camera else "REAL CAMERA"
            waiting_text = self.font_large.render(
                f"WAITING FOR {source_text}", 1, (128, 128, 128)
            )
            self.screen.blit(
                waiting_text, waiting_text.get_rect(center=self.camera_area.center)
            )

        # --- Draw the camera toggle checkbox and its label (no changes here) ---
        pygame.draw.rect(self.screen, (200, 200, 200), self.camera_toggle_checkbox, 1)
        if self.show_sim_camera:
            pygame.draw.line(
                self.screen,
                "#32CD32",
                (self.camera_toggle_checkbox.x + 3, self.camera_toggle_checkbox.y + 10),
                (self.camera_toggle_checkbox.x + 8, self.camera_toggle_checkbox.y + 15),
                3,
            )
            pygame.draw.line(
                self.screen,
                "#32CD32",
                (self.camera_toggle_checkbox.x + 8, self.camera_toggle_checkbox.y + 15),
                (self.camera_toggle_checkbox.x + 15, self.camera_toggle_checkbox.y + 5),
                3,
            )

        label_text = "USE SIM" if self.show_sim_camera else "USE REAL"
        label_color = (255, 100, 100) if self.show_sim_camera else (100, 255, 100)
        self.screen.blit(
            self.font.render(label_text, 1, label_color),
            (self.camera_toggle_checkbox.right + 8, self.camera_toggle_checkbox.y),
        )

    def draw_camera_panel(self):
        pygame.draw.rect(self.screen, (0, 0, 0), self.camera_area)

        frame_to_display = None

        if self.show_sim_camera:
            if hasattr(self, "last_processed_sim"):
                frame_to_display = self.last_processed_sim
        else:
            if hasattr(self, "last_processed_real"):
                frame_to_display = self.last_processed_real

        if (
            frame_to_display is not None
            and frame_to_display.shape[0] > 0
            and frame_to_display.shape[1] > 0
        ):
            try:
                if len(frame_to_display.shape) == 3:
                    frame_rgb = cv2.cvtColor(frame_to_display, cv2.COLOR_BGR2RGB)
                    frame_resized = cv2.resize(frame_rgb, self.camera_area.size)

                    # --- THE FIX IS HERE ---
                    # Replace np.rot90 with np.transpose to correctly swap the axes for Pygame
                    # without visually rotating or distorting the image content.
                    camera_surface = pygame.surfarray.make_surface(
                        np.transpose(frame_resized, (1, 0, 2))
                    )

                    self.screen.blit(camera_surface, self.camera_area.topleft)
                else:
                    raise ValueError("Frame is not in BGR format")
            except Exception as e:
                source_text = "SIM CAMERA" if self.show_sim_camera else "REAL CAMERA"
                error_text = self.font.render(
                    f"ERROR DISPLAYING {source_text}", 1, (255, 0, 0)
                )
                self.screen.blit(
                    error_text, error_text.get_rect(center=self.camera_area.center)
                )
                print(f"Error drawing camera panel: {e}")

        else:
            source_text = "SIM CAMERA" if self.show_sim_camera else "REAL CAMERA"
            waiting_text = self.font_large.render(
                f"WAITING FOR {source_text}", 1, (128, 128, 128)
            )
            self.screen.blit(
                waiting_text, waiting_text.get_rect(center=self.camera_area.center)
            )

        # --- (The rest of the function remains unchanged) ---
        pygame.draw.rect(self.screen, (200, 200, 200), self.camera_toggle_checkbox, 1)
        if self.show_sim_camera:
            pygame.draw.line(
                self.screen,
                "#32CD32",
                (self.camera_toggle_checkbox.x + 3, self.camera_toggle_checkbox.y + 10),
                (self.camera_toggle_checkbox.x + 8, self.camera_toggle_checkbox.y + 15),
                3,
            )
            pygame.draw.line(
                self.screen,
                "#32CD32",
                (self.camera_toggle_checkbox.x + 8, self.camera_toggle_checkbox.y + 15),
                (self.camera_toggle_checkbox.x + 15, self.camera_toggle_checkbox.y + 5),
                3,
            )

        label_text = "USE SIM" if self.show_sim_camera else "USE REAL"
        label_color = (255, 100, 100) if self.show_sim_camera else (100, 255, 100)
        self.screen.blit(
            self.font.render(label_text, 1, label_color),
            (self.camera_toggle_checkbox.right + 8, self.camera_toggle_checkbox.y),
        )

    def draw_status_panel(self):
        pygame.draw.rect(self.screen, (35, 35, 35), self.status_area)
        x, y = self.status_area.left + 20, self.status_area.top + 20
        sync_mode, sync_text, sync_color = (
            self.hud_data.get("sync_mode", 1),
            "SYNC",
            ("#00FF80"),
        )
        if sync_mode != 1:
            sync_text, sync_color = "ASYNC", ("#FFA500")
        self.screen.blit(
            self.font_large.render(f"MODE: {sync_text}", 1, sync_color), (x, y)
        )

        col1, col2, col3 = x, x + 200, x + 400
        row_y = y + 50
        self.screen.blit(self.font_bold.render("SYSTEM", 1, "#A0A0FF"), (col1, row_y))
        self.screen.blit(self.font_bold.render("CONTROL", 1, "#A0FFA0"), (col2, row_y))
        self.screen.blit(self.font_bold.render("DEPTH", 1, "#FFA0A0"), (col3, row_y))

        armed_t, armed_c = (
            ("ARMED", "#FF6347") if self.hud_data["armed"] else ("DISARMED", "#90EE90")
        )
        self.screen.blit(self.font.render(armed_t, 1, armed_c), (col1, row_y + 30))
        self.screen.blit(
            self.font.render(
                f"LIGHT: {'ON' if self.hud_data['light_on'] else 'OFF'}", 1, "#FFFF87"
            ),
            (col1, row_y + 50),
        )
        self.screen.blit(
            self.font.render(
                f"STAB: {'ON' if self.hud_data.get('stabilization') else 'OFF'}",
                1,
                "#87CEEB",
            ),
            (col1, row_y + 70),
        )

        self.screen.blit(
            self.font.render(
                self.hud_data.get("controller_mode", "JOYSTICK"), 1, "#90EE90"
            ),
            (col2, row_y + 30),
        )
        self.screen.blit(
            self.font.render(
                f"SIM KP: {self.input_values['sim_kp']:.2f}", 1, "#FFFFFF"
            ),
            (col2, row_y + 50),
        )
        self.screen.blit(
            self.font.render(
                f"REAL KP: {self.input_values['real_kp']:.0f}", 1, "#FFFFFF"
            ),
            (col2, row_y + 70),
        )

        # Updated depth display for negative values
        real_d = f"{self.real_depth:.1f}" if self.real_depth is not None else "N/A"
        sim_d = f"{self.sim_depth:.1f}" if self.sim_depth is not None else "N/A"
        self.screen.blit(
            self.font.render(f"REAL: {real_d}", 1, "#87CEEB"), (col3, row_y + 30)
        )
        self.screen.blit(
            self.font.render(f"SIM:  {sim_d}", 1, "#FFFF87"), (col3, row_y + 50)
        )

        target_d = (
            f"{self.input_values['depth_target']:.1f}"
            if self.input_values["depth_target"] != -1.0
            else "SURFACE"
        )
        self.screen.blit(
            self.font.render(f"TARGET: {target_d}", 1, "#FFA500"), (col3, row_y + 70)
        )

    def draw_control_panel(self):
        pygame.draw.rect(self.screen, (45, 45, 45), self.control_area)
        x, y = self.control_area.left + 30, self.control_area.top + 30
        label_y_offset = -25
        col_1_x = self.control_area.x + 30
        col_2_x = self.control_area.x + 120
        col_3_x = self.control_area.x + 210
        depth_row_y = self.control_area.y + 110

        self.screen.blit(
            self.font_large.render("SIMULATION CONTROL", 1, "#FFFF87"), (x, y)
        )
        
        # Depth PID section labels
        self.screen.blit(self.font_bold.render("Depth PID", 1, "#87CEEB"), (col_1_x, depth_row_y - 25))
        self.screen.blit(self.font_bold.render("Sim", 1, "#90EE90"), (col_1_x, depth_row_y - 10))
        self.screen.blit(self.font_bold.render("Real", 1, "#FFB6C1"), (col_1_x, depth_row_y + 30))
        
        self.screen.blit(
            self.font_bold.render("Steps", 1, (220, 220, 220)),
            (self.steps_box.x, self.steps_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Delay (ms)", 1, (220, 220, 220)),
            (self.delay_box.x, self.delay_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Depth", 1, "#87CEEB"),
            (self.depth_target_box.x, self.depth_target_box.y + label_y_offset),
        )

        # Green color for simulator labels
        sim_color = "#90EE90"
        self.screen.blit(
            self.font_large.render("SIMULATOR PID", 1, sim_color), (x, y + 80)
        )
        self.screen.blit(
            self.font_bold.render("Kp", 1, sim_color),
            (self.sim_kp_box.x, self.sim_kp_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Ki", 1, sim_color),
            (self.sim_ki_box.x, self.sim_ki_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Kd", 1, sim_color),
            (self.sim_kd_box.x, self.sim_kd_box.y + label_y_offset),
        )

        # Vertical separator
        sep_x = self.sim_kd_box.right + 20
        pygame.draw.line(
            self.screen, (100, 100, 100), (sep_x, y + 60), (sep_x, y + 240), 2
        )

        # Sim multiplier labels (green)
        self.screen.blit(
            self.font_bold.render("Surge", 1, sim_color),
            (self.sim_surge_box.x, self.sim_surge_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Strafe", 1, sim_color),
            (self.sim_strafe_box.x, self.sim_strafe_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Yaw", 1, sim_color),
            (self.sim_yaw_box.x, self.sim_yaw_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Heave", 1, sim_color),
            (self.sim_heave_box.x, self.sim_heave_box.y + label_y_offset),
        )

        # Depth PID labels for sim (green)
        self.screen.blit(
            self.font_bold.render("Kp", 1, sim_color),
            (self.sim_depth_kp_box.x, self.sim_depth_kp_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Ki", 1, sim_color),
            (self.sim_depth_ki_box.x, self.sim_depth_ki_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Kd", 1, sim_color),
            (self.sim_depth_kd_box.x, self.sim_depth_kd_box.y + label_y_offset),
        )

        # Pink color for real sub labels
        real_color = "#FFB6C1"
        self.screen.blit(
            self.font_large.render("REAL SUB PID", 1, real_color), (x, y + 160)
        )
        self.screen.blit(
            self.font_bold.render("Kp", 1, real_color),
            (self.real_kp_box.x, self.real_kp_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Ki", 1, real_color),
            (self.real_ki_box.x, self.real_ki_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Kd", 1, real_color),
            (self.real_kd_box.x, self.real_kd_box.y + label_y_offset),
        )

        # Same for real sub labels
        self.screen.blit(
            self.font_bold.render("Surge", 1, real_color),
            (self.real_surge_box.x, self.real_surge_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Strafe", 1, real_color),
            (self.real_strafe_box.x, self.real_strafe_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Yaw", 1, real_color),
            (self.real_yaw_box.x, self.real_yaw_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Heave", 1, real_color),
            (self.real_heave_box.x, self.real_heave_box.y + label_y_offset),
        )

        # Depth PID labels for real (pink)
        self.screen.blit(
            self.font_bold.render("Kp", 1, real_color),
            (self.real_depth_kp_box.x, self.real_depth_kp_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Ki", 1, real_color),
            (self.real_depth_ki_box.x, self.real_depth_ki_box.y + label_y_offset),
        )
        self.screen.blit(
            self.font_bold.render("Kd", 1, real_color),
            (self.real_depth_kd_box.x, self.real_depth_kd_box.y + label_y_offset),
        )

        input_configs = [
            (self.steps_box, self.steps_text, "steps"),
            (self.delay_box, self.delay_text, "delay"),
            (self.depth_target_box, self.depth_target_text, "depth_target"),
            (self.sim_kp_box, self.sim_kp_text, "sim_kp"),
            (self.sim_ki_box, self.sim_ki_text, "sim_ki"),
            (self.sim_kd_box, self.sim_kd_text, "sim_kd"),
            (self.sim_surge_box, self.sim_surge_text, "sim_surge"),
            (self.sim_strafe_box, self.sim_strafe_text, "sim_strafe"),
            (self.sim_yaw_box, self.sim_yaw_text, "sim_yaw"),
            (self.sim_heave_box, self.sim_heave_text, "sim_heave"),
            # Depth PID input boxes
            (self.sim_depth_kp_box, self.sim_depth_kp_text, "sim_depth_kp"),
            (self.sim_depth_ki_box, self.sim_depth_ki_text, "sim_depth_ki"),
            (self.sim_depth_kd_box, self.sim_depth_kd_text, "sim_depth_kd"),
            (self.real_kp_box, self.real_kp_text, "real_kp"),
            (self.real_ki_box, self.real_ki_text, "real_ki"),
            (self.real_kd_box, self.real_kd_text, "real_kd"),
            (self.real_surge_box, self.real_surge_text, "real_surge"),
            (self.real_strafe_box, self.real_strafe_text, "real_strafe"),
            (self.real_yaw_box, self.real_yaw_text, "real_yaw"),
            (self.real_heave_box, self.real_heave_text, "real_heave"),
            (self.real_depth_kp_box, self.real_depth_kp_text, "real_depth_kp"),
            (self.real_depth_ki_box, self.real_depth_ki_text, "real_depth_ki"),
            (self.real_depth_kd_box, self.real_depth_kd_text, "real_depth_kd"),
        ]
        for box, text, input_type in input_configs:
            self._draw_large_input_box(box, text, input_type)

    def _draw_large_input_box(self, box, text, input_type):
        is_active = self.active_input == input_type
        box_color = (255, 255, 100) if is_active else (180, 180, 180)
        bg_color = (70, 70, 70) if is_active else (50, 50, 50)
        border = 3 if is_active else 2
        pygame.draw.rect(self.screen, bg_color, box)
        pygame.draw.rect(self.screen, box_color, box, border)
        text_surf = self.font_input.render(text, 1, (255, 255, 255))
        text_rect = text_surf.get_rect(center=box.center)
        self.screen.blit(text_surf, text_rect)
        if is_active and int(time.time() * 2) % 2:
            cursor_x = text_rect.right + 3
            pygame.draw.line(
                self.screen,
                (255, 255, 100),
                (cursor_x, box.y + 6),
                (cursor_x, box.y + box.height - 6),
                2,
            )

    def _draw_plot_checkboxes(self):
        y_pos = self.comparison_plot_area.top + 15
        x_start = self.comparison_plot_area.left + 20
        for i, label in enumerate(self.plot_visibility.keys()):
            rect = pygame.Rect(x_start + i * 110, y_pos, 20, 20)
            self.checkboxes[label] = rect
            pygame.draw.rect(self.screen, (200, 200, 200), rect, 1)
            if self.plot_visibility[label]:
                pygame.draw.line(
                    self.screen,
                    "#32CD32",
                    (rect.x + 3, rect.y + 10),
                    (rect.x + 8, rect.y + 15),
                    3,
                )
                pygame.draw.line(
                    self.screen,
                    "#32CD32",
                    (rect.x + 8, rect.y + 15),
                    (rect.x + 15, rect.y + 5),
                    3,
                )
            self.screen.blit(
                self.font.render(label, 1, (220, 220, 220)), (rect.right + 8, rect.y)
            )

    def handle_input_events(self, events):
        updated_values = {}
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                # Check camera toggle checkbox first
                if self.camera_toggle_checkbox.collidepoint(event.pos):
                    self.show_sim_camera = not self.show_sim_camera
                    return {}  # Return early

                clicked_on_ui = False
                for label, rect in self.checkboxes.items():
                    if rect.collidepoint(event.pos):
                        self.plot_visibility[label] = not self.plot_visibility[label]
                        clicked_on_ui = True
                        break
                if clicked_on_ui:
                    self.active_input = None
                    continue
                    
                box_map = {
                    "steps": self.steps_box,
                    "delay": self.delay_box,
                    "depth_target": self.depth_target_box,
                    "sim_kp": self.sim_kp_box,
                    "sim_ki": self.sim_ki_box,
                    "sim_kd": self.sim_kd_box,
                    "sim_depth_kp": self.sim_depth_kp_box,
                    "sim_depth_ki": self.sim_depth_ki_box,
                    "sim_depth_kd": self.sim_depth_kd_box,
                    "sim_surge": self.sim_surge_box,
                    "sim_strafe": self.sim_strafe_box,
                    "sim_yaw": self.sim_yaw_box,
                    "sim_heave": self.sim_heave_box,
                    "real_kp": self.real_kp_box,
                    "real_ki": self.real_ki_box,
                    "real_kd": self.real_kd_box,
                    "real_depth_kp": self.real_depth_kp_box,
                    "real_depth_ki": self.real_depth_ki_box,
                    "real_depth_kd": self.real_depth_kd_box,
                    "real_surge": self.real_surge_box,
                    "real_strafe": self.real_strafe_box,
                    "real_yaw": self.real_yaw_box,
                    "real_heave": self.real_heave_box,
                }

                for name, rect in box_map.items():
                    if rect.collidepoint(event.pos):
                        self.active_input = name
                        clicked_on_ui = True
                        break
                if not clicked_on_ui:
                    self.active_input = None

            elif event.type == pygame.KEYDOWN and self.active_input:
                attr, text = f"{self.active_input}_text", getattr(
                    self, f"{self.active_input}_text"
                )
                if event.key == pygame.K_RETURN or event.key == pygame.K_TAB:
                    try:
                        v_type = int if self.active_input == "steps" else float
                        key_mapping = {
                            "steps": "num_steps",
                            "delay": "delay_ms",
                            "depth_target": "depth_target",
                            "sim_surge": "sim_surge_mult",
                            "sim_strafe": "sim_strafe_mult",
                            "sim_yaw": "sim_yaw_mult",
                            "sim_heave": "sim_heave_mult",
                            "real_surge": "real_surge_mult",
                            "real_strafe": "real_strafe_mult",
                            "real_yaw": "real_yaw_mult",
                            "real_heave": "real_heave_mult",
                        }
                        v_key = key_mapping.get(self.active_input, self.active_input)
                        new_val = v_type(text)
                        self.input_values[v_key] = new_val
                        updated_values[v_key] = new_val
                        print(f"Updated {v_key}: {new_val}")
                    except ValueError:
                        setattr(self, attr, str(self.input_values[v_key]))
                    self.active_input = None
                elif event.key == pygame.K_ESCAPE:
                    self.active_input = None
                elif event.key == pygame.K_BACKSPACE:
                    setattr(self, attr, text[:-1])
                elif event.unicode.isprintable():
                    setattr(self, attr, text + event.unicode)
        return updated_values

    def setup_3d_plot(self):
        plot_dpi = 100
        fig_size = (
            self.plot_3d_area.width / plot_dpi,
            self.plot_3d_area.height / plot_dpi,
        )
        self.fig = plt.figure(figsize=fig_size, dpi=plot_dpi, facecolor="#2d2d2d")
        self.ax_3d = self.fig.add_subplot(1, 1, 1, projection="3d", facecolor="#191919")
        self.pyramid_collection = Poly3DCollection(
            [],
            facecolors=self.pyramid_face_colors,
            linewidths=0.8,
            edgecolors="k",
            alpha=0.7,
        )
        for ax_name in ["x", "y", "z"]:
            getattr(self.ax_3d, f"set_{ax_name}label")(
                ax_name.upper(), color="white", fontsize=10
            )
            getattr(self.ax_3d, f"set_{ax_name}lim")([-1, 1])
            self.ax_3d.tick_params(axis=ax_name, colors="white")
        self.ax_3d.xaxis.pane.fill = False
        self.ax_3d.yaxis.pane.fill = False
        self.ax_3d.zaxis.pane.fill = False
        self.ax_3d.grid(color="gray", linestyle="--", linewidth=0.5, alpha=0.5)
        try:
            self.ax_3d.set_box_aspect([1, 1, 1])
        except AttributeError:
            pass
        self.ax_3d.view_init(elev=25, azim=35)
        colors, labels = ["#FF4500", "#32CD32", "#1E90FF"], [
            "X (Roll)",
            "Y (Pitch)",
            "Z (Yaw)",
        ]
        self.frame_lines = [
            self.ax_3d.plot([], [], [], c=c, lw=3, label=l)[0]
            for c, l in zip(colors, labels)
        ]
        legend = self.ax_3d.legend(facecolor="#333333", edgecolor="gray")
        for text in legend.get_texts():
            text.set_color("white")
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

    def update_depth_data(self, real_imu_data, sim_data):
        if (
            real_imu_data
            and "depth" in real_imu_data
            and real_imu_data["depth"]
            and "depth" in real_imu_data["depth"]
        ):
            self.real_depth = real_imu_data["depth"]["depth"]["relative_cm"]
        if sim_data and "depth" in sim_data:
            self.sim_depth = sim_data["depth"]

    def setup_comparison_plot(self):
        plot_dpi = 100
        fig_size = (
            self.comparison_plot_area.width / plot_dpi,
            self.comparison_plot_area.height / plot_dpi,
        )
        self.comparison_fig = plt.figure(
            figsize=fig_size, dpi=plot_dpi, facecolor="#191919"
        )
        self.comparison_ax = self.comparison_fig.add_subplot(
            1, 1, 1, facecolor="#252525"
        )
        # Lighter, more vibrant colors for dark mode
        (self.real_roll_line,) = self.comparison_ax.plot(
            [], [], color="#FF6347", linestyle="-", lw=2, label="Real Roll"
        )
        (self.real_pitch_line,) = self.comparison_ax.plot(
            [], [], color="#32CD32", linestyle="-", lw=2, label="Real Pitch"
        )
        (self.real_yaw_line,) = self.comparison_ax.plot(
            [], [], color="#1E90FF", linestyle="-", lw=2, label="Real Yaw"
        )
        (self.sim_roll_line,) = self.comparison_ax.plot(
            [], [], color="#FFB6C1", linestyle="--", lw=2, label="Sim Roll"
        )
        (self.sim_pitch_line,) = self.comparison_ax.plot(
            [], [], color="#98FB98", linestyle="--", lw=2, label="Sim Pitch"
        )
        (self.sim_yaw_line,) = self.comparison_ax.plot(
            [], [], color="#ADD8E6", linestyle="--", lw=2, label="Sim Yaw"
        )

        self.comparison_ax.set_xlabel("Time (s)", color="white")
        self.comparison_ax.set_ylabel("Angle (degrees)", color="white")
        self.comparison_ax.set_title(
            "Real vs Sim RPY Comparison", color="white", fontweight="bold"
        )
        legend = self.comparison_ax.legend(
            loc="upper right", fontsize=10, facecolor="#333333"
        )
        for text in legend.get_texts():
            text.set_color("white")
        self.comparison_ax.set_ylim(-180, 180)
        self.comparison_ax.grid(True, alpha=0.2, color="gray")
        for spine in self.comparison_ax.spines.values():
            spine.set_color("gray")
        self.comparison_ax.tick_params(colors="white")
        self.comparison_fig.tight_layout(pad=1.2)
        self.plot_visibility = {"Roll": True, "Pitch": True, "Yaw": True}
        self.checkboxes = {}

    def update_comparison_data(self, real_imu_data, sim_quaternion):
        current_time = time.time() - self.start_time
        if (
            real_imu_data
            and "orientation" in real_imu_data
            and real_imu_data["orientation"]
            and "euler_deg" in real_imu_data["orientation"]
        ):
            euler = real_imu_data["orientation"]["euler_deg"]
            self.real_times.append(current_time)
            self.real_roll.append(euler["roll"])
            self.real_pitch.append(euler["pitch"])
            self.real_yaw.append(euler["yaw"])
        if sim_quaternion is not None:
            roll, pitch, yaw = self.quaternion_to_euler(sim_quaternion)
            self.sim_times.append(current_time)
            self.sim_roll.append(roll)
            self.sim_pitch.append(pitch)
            self.sim_yaw.append(yaw)

    def update_comparison_plot(self):
        lines = {
            "Roll": (
                self.real_roll_line,
                self.sim_roll_line,
                self.real_roll,
                self.sim_roll,
            ),
            "Pitch": (
                self.real_pitch_line,
                self.sim_pitch_line,
                self.real_pitch,
                self.sim_pitch,
            ),
            "Yaw": (self.real_yaw_line, self.sim_yaw_line, self.real_yaw, self.sim_yaw),
        }
        for label, (real_line, sim_line, real_data, sim_data) in lines.items():
            if self.plot_visibility[label]:
                if len(self.real_times) > 1:
                    real_line.set_data(list(self.real_times), list(real_data))
                if len(self.sim_times) > 1:
                    sim_line.set_data(list(self.sim_times), list(sim_data))
            else:
                real_line.set_data([], [])
                sim_line.set_data([], [])
        all_times = list(self.real_times) + list(self.sim_times)
        if all_times:
            max_t = max(all_times)
            self.comparison_ax.set_xlim(
                max(0, max_t - self.time_window_size), max(max_t, self.time_window_size)
            )


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
                    "x": values[6],  # Updated indices
                    "y": values[7],
                    "width": values[8],
                    "height": values[9],
                },
            }
        except struct.error:
            print(
                f"Error unpacking sensor response: expected 40 bytes, got {len(payload)} bytes"
            )
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
        pose = np.array([9.25, -5.71, 0, 1, 0, 0, 0])
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
        self.data_receiver_thread = SubmarineDataReceiverThread(
            video_port=10001,  # Real submarine video
            sim_video_port=60000,  # Unity simulation video
        )

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
            "yaw_real": PIDGains(0.6, 0, 0),
            "depth_sim": PIDGains(0.5, 0.01, 0.2),    # Much more conservative
            "depth_real": PIDGains(0.02, 0.001, 0.05), # Much more conservative for real sub
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

    def calculate_depth_control(self, current_depth: Optional[float], target_depth: float, system_type: str) -> float:
        """Calculate depth control command (heave force)
        Depth values: -1.0 is surface, more negative is deeper.
        Target depth from the UI is in CENTIMETERS.
        Real depth is in cm, sim depth is in meters.
        """
        if current_depth is None or np.isnan(current_depth) or target_depth == -1.0:
            return 0.0

        unit = ""
        # Convert units to be consistent. UI input is now in cm.
        if system_type == "real":
            # Target (UI) is in cm, Current (sensor) is in cm. No conversion needed.
            target_depth_converted = target_depth
            current_depth_converted = current_depth
            unit = "cm"
        else:  # system_type == "sim"
            # Target (UI) is in cm, Current (sensor) is in m. Convert target to meters.
            target_depth_converted = target_depth / 100.0
            current_depth_converted = current_depth
            unit = "m"

        # Calculate depth error (negative means need to go deeper)
        depth_error = target_depth_converted - current_depth_converted
        
        # Debug print to see what's happening
        if abs(depth_error) > 0.5:  # Threshold for printing
            print(f"DEPTH CONTROL [{system_type}]: Current={current_depth_converted:.2f}{unit}, Target={target_depth_converted:.2f}{unit}, Error={depth_error:.2f}{unit}")

        key = "depth_sim" if system_type == "sim" else "depth_real"
        g, s = self.pid_gains[key], self.pid_states[key]

        # PID calculation
        s.integral = np.clip(s.integral + depth_error * self.dt, -5.0, 5.0)
        derivative = (depth_error - s.prev_error) / self.dt
        depth_command = g.kp * depth_error + g.ki * s.integral + g.kd * derivative
        s.prev_error = depth_error

        if np.isnan(depth_command):
            depth_command = 0.0

        max_output = 20.0 if system_type == "real" else 30.0
        return np.clip(depth_command, -max_output, max_output)

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
            sim_data = None  # This data is currently unused by the external controller but fetched

            # Get both camera frames
            real_frame = None
            sim_frame = None

            with self.data_receiver_thread.frame_lock:
                if self.data_receiver_thread.latest_frame is not None:
                    real_frame = self.data_receiver_thread.latest_frame.copy()

            with self.data_receiver_thread.sim_frame_lock:
                if self.data_receiver_thread.latest_sim_frame is not None:
                    sim_frame = self.data_receiver_thread.latest_sim_frame.copy()

            # Extract depth measurement (currently unused, but available)
            depth_measurement = None
            if (
                real_data
                and "depth" in real_data
                and real_data["depth"]
                and "depth" in real_data["depth"]
            ):
                depth_measurement = real_data["depth"]["depth"]

            # Extract bounding box data from simulator (currently unused)
            bbox_data = None

            # KEY CHANGE: Determine which camera is selected in the UI
            # The checkbox now determines which camera's commands will be used.
            selected_camera = "sim" if self.visualizer.show_sim_camera else "real"

            # KEY CHANGE: Call the updated external controller loop.
            # It processes both frames but returns commands from the selected camera.
            # It now returns SIX values.
            surge, strafe, heave, yaw_cmd, processed_real, processed_sim = (
                self.external_controller.update_loop(
                    real_frame,
                    sim_frame,
                    real_data,
                    depth_measurement,
                    bbox_data,
                    selected_camera,
                )
            )

            # BUG FIX: Store the processed frames for the visualizer to display.
            # This is the crucial link that was missing.
            self.visualizer.last_processed_real = processed_real
            self.visualizer.last_processed_sim = processed_sim

            # Apply scaling (external controller returns -1 to 1, scale to our system)
            surge *= self.translation_scale
            strafe *= self.translation_scale
            heave *= self.translation_scale
            yaw_cmd *= self.rotation_scale

            return surge, strafe, heave, yaw_cmd

        except Exception as e:
            print(f" External controller error: {e}")
            # Ensure visualizer frames are cleared on error to prevent stale images
            self.visualizer.last_processed_real = None
            self.visualizer.last_processed_sim = None
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
        # self.visualizer._external_controller_ref = self.external_controller # This is no longer needed
        self.data_receiver_thread.start()

        shutdown = False
        loop_count = 0
        cmdstarttime = time.time()

        # --- Initialize vision results to None ---
        real_vision_result, sim_vision_result = None, None

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
                        elif e.button == self.BTN_A:
                            print("\nSending RESET signal to Sub & Sim...")
                            self.target_attitude = np.array([1.0, 0, 0, 0])
                            if self.real_sub_client:
                                self.real_sub_client.send_reset_command()
                            asyncio.create_task(
                                self.auv.reset(sync_flag=self.sync_mode)
                            )

                # Handle GUI input boxes
                updated_values = self.visualizer.handle_input_events(events)
                if "sim_kp" in updated_values:
                    self.pid_gains["yaw_sim"].kp = updated_values["sim_kp"]
                if "real_kp" in updated_values:
                    self.pid_gains["yaw_real"].kp = updated_values["real_kp"]
                if "sim_depth_kp" in updated_values:
                    self.pid_gains["depth_sim"].kp = updated_values["sim_depth_kp"]
                if "real_depth_kp" in updated_values:
                    self.pid_gains["depth_real"].kp = updated_values["real_depth_kp"]
                if "num_steps" in updated_values:
                    self.live_num_steps = updated_values["num_steps"]
                if "delay_ms" in updated_values:
                    self.live_delay_ms = updated_values["delay_ms"]
                if "depth_target" in updated_values:
                    print(f"Depth target set to: {updated_values['depth_target']} cm")
                if "sim_depth_ki" in updated_values:
                    self.pid_gains["depth_sim"].ki = updated_values["sim_depth_ki"]
                if "sim_depth_kd" in updated_values:
                    self.pid_gains["depth_sim"].kd = updated_values["sim_depth_kd"]
                if "real_depth_ki" in updated_values:
                    self.pid_gains["depth_real"].ki = updated_values["real_depth_ki"]
                if "real_depth_kd" in updated_values:
                    self.pid_gains["depth_real"].kd = updated_values["real_depth_kd"]

                if shutdown:
                    break

                # ==================================================================
                # STEP 1: VISION PROCESSING (ALWAYS RUNS)
                # ==================================================================
                if self.external_controller:
                    # Get the latest raw camera frames
                    real_frame, sim_frame = None, None
                    with self.data_receiver_thread.frame_lock:
                        if self.data_receiver_thread.latest_frame is not None:
                            real_frame = self.data_receiver_thread.latest_frame.copy()
                    with self.data_receiver_thread.sim_frame_lock:
                        if self.data_receiver_thread.latest_sim_frame is not None:
                            sim_frame = (
                                self.data_receiver_thread.latest_sim_frame.copy()
                            )

                    # Process frames ONCE per loop and get all results back
                    real_vision_result, sim_vision_result = (
                        self.external_controller.update_loop(real_frame, sim_frame)
                    )

                    # ALWAYS update the visualizer with the processed frames
                    self.visualizer.last_processed_real = (
                        real_vision_result.processed_image
                    )
                    self.visualizer.last_processed_sim = (
                        sim_vision_result.processed_image
                    )
                else:
                    # If no external controller, just pass raw frames for display
                    with self.data_receiver_thread.frame_lock:
                        self.visualizer.last_processed_real = (
                            self.data_receiver_thread.latest_frame
                        )
                    with self.data_receiver_thread.sim_frame_lock:
                        self.visualizer.last_processed_sim = (
                            self.data_receiver_thread.latest_sim_frame
                        )

                # ==================================================================
                # STEP 2: CONTROL INPUT SELECTION (DECISION LOGIC)
                # ==================================================================
                if self.external_controller_active and real_vision_result is not None:
                    # --- Use External Controller Commands ---
                    selected_camera = (
                        "sim" if self.visualizer.show_sim_camera else "real"
                    )
                    active_result = (
                        sim_vision_result
                        if selected_camera == "sim"
                        else real_vision_result
                    )

                    surge, strafe, heave, yaw_cmd = (
                        active_result.surge,
                        active_result.strafe,
                        active_result.heave,
                        active_result.yaw_cmd,
                    )

                    # Joystick override logic
                    joystick_surge, joystick_strafe, joystick_heave, joystick_yaw = (
                        self.get_controller_input()
                    )
                    joystick_magnitude = (
                        abs(joystick_surge)
                        + abs(joystick_strafe)
                        + abs(joystick_heave)
                        + abs(joystick_yaw)
                    )
                    if joystick_magnitude > 0.1:
                        print(" JOYSTICK OVERRIDE DETECTED")
                        surge, strafe, heave, yaw_cmd = (
                            joystick_surge,
                            joystick_strafe,
                            joystick_heave,
                            joystick_yaw,
                        )
                        self.external_controller_active = False  # Disable for safety
                        self.stabilization_enabled = False
                else:
                    # --- Use Joystick Commands ---
                    surge, strafe, heave, yaw_cmd = self.get_controller_input()

                # ==================================================================
                # STEP 3: FORCE CALCULATION & DISPATCH (No changes here)
                # ==================================================================
                real_data = self.data_receiver_thread.latest_imu_data
                current_attitude_real = (
                    np.array(real_data["orientation"]["quaternion"])
                    if real_data
                    and "orientation" in real_data
                    and real_data.get("orientation")
                    else None
                )

                sim_data, current_attitude_sim = None, None
                try:
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

                # Extract current depths
                current_depth_real = None
                current_depth_sim = None

                if (
                    real_data
                    and "depth" in real_data
                    and real_data["depth"]
                    and "depth" in real_data["depth"]
                ):
                    current_depth_real = real_data["depth"]["depth"]["relative_cm"]

                if sim_data and "depth" in sim_data:
                    current_depth_sim = sim_data["depth"]

                # Calculate depth control commands
                depth_target = self.visualizer.input_values["depth_target"]

                sim_depth_control = self.calculate_depth_control(
                    current_depth_sim, depth_target, "sim"
                )
                real_depth_control = self.calculate_depth_control(
                    current_depth_real, depth_target, "real"
                )

                sim_stab_f = self.calculate_stabilization_forces(
                    current_attitude_sim, "sim"
                )
                real_stab_f = self.calculate_stabilization_forces(
                    current_attitude_real, "real"
                )

                sim_surge, sim_strafe, sim_heave, sim_yaw_cmd = (
                    surge * self.visualizer.input_values["sim_surge_mult"],
                    strafe * self.visualizer.input_values["sim_strafe_mult"],
                    (heave * self.visualizer.input_values["sim_heave_mult"])
                    + sim_depth_control,
                    yaw_cmd * self.visualizer.input_values["sim_yaw_mult"],
                )
                real_surge, real_strafe, real_heave, real_yaw_cmd = (
                    surge * self.visualizer.input_values["real_surge_mult"],
                    strafe * self.visualizer.input_values["real_strafe_mult"],
                    (heave * self.visualizer.input_values["real_heave_mult"])
                    + real_depth_control,
                    yaw_cmd * self.visualizer.input_values["real_yaw_mult"],
                )

                sim_f = self.calculate_thruster_forces(
                    sim_surge, sim_strafe, sim_heave, sim_yaw_cmd, "sim", sim_stab_f
                )
                real_f = self.calculate_thruster_forces(
                    real_surge,
                    real_strafe,
                    real_heave,
                    real_yaw_cmd,
                    "real",
                    real_stab_f,
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

                await asyncio.gather(*tasks, return_exceptions=True)

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
                    }
                )
                self.visualizer.update_depth_data(real_data, sim_data)
                self.visualizer.update(imu_override=current_attitude_sim)

                loop_time = time.time() - loop_start
                self.loop_times.append(loop_time)
                await asyncio.sleep(max(0, self.dt - loop_time))
        finally:
            print("Disarming and stopping...")
            self.data_receiver_thread.stop()
            self.data_receiver_thread.join(timeout=1)
            plt.close("all")
            if self.real_sub_client:
                self.real_sub_client.shutdown()
            pygame.quit()

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
