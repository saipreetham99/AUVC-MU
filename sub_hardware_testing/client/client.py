#!/usr/bin/env python3
"""
Merged BlueROV2 Controller with correct joystick input logic and IMU data
handling.

This version combines the correct components from both source files:
- CombinedVisualizer, SubmarineDataReceiverThread, MsgHeader from file 1
- AUV, RealSubmarineClient, BlueROVController from file 2
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import matplotlib.pyplot as plt
# import os
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# from typing import List
import pygame
import numpy as np
import time
import sys
import socket
import struct
import argparse
import threading
import json
import cv2
# import selectors

# Use a non-interactive backend for Matplotlib so it
# doesn't create its own window
import matplotlib
matplotlib.use("Agg")


class MsgHeader(Enum):  # CORRECT
    ERROR = 0,
    NO_ERROR = 1,
    HEARTBEAT = 2,
    GET_MODEL_INFO = 3,
    GET_SENSORDATA = 4
    GET_RGB_IMAGE = 5,
    GET_MASKED_IMAGE = 6,
    APPLY_CTRL = 7
    STEP_SIM = 8,
    RESET = 9


class SubmarineDataReceiverThread(threading.Thread):  # CORRECT
    def __init__(self, video_port=10001, imu_port=10002):
        super().__init__(daemon=True, name="SubmarineDataReceiver")
        self.video_listen_addr = ('', video_port)
        self.imu_listen_addr = ('', imu_port)
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
                fid, cs, cid = struct.unpack('!HHH', d[:6])
                if fid not in b:
                    b[fid] = {}
                b[fid][cid] = d[6:]
                if len(b[fid]) == cs:
                    jpeg = b''.join(v for k, v in sorted(b[fid].items()))
                    fr = cv2.imdecode(np.frombuffer(
                        jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    with self.frame_lock:
                        self.latest_frame = fr
                    del b[fid]
            except socket.timeout:
                continue
            except Exception:
                pass
        s.close()
        print("SubDataReceiver: Video loop stopped.")

    def _imu_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(self.imu_listen_addr)
        s.settimeout(1.0)
        while not self.stop_event.is_set():
            try:
                d, _ = s.recvfrom(1024)
                self.latest_imu_data = json.loads(d.decode('utf-8'))
            except socket.timeout:
                continue
            except Exception:
                self.latest_imu_data = None
        s.close()
        print("SubDataReceiver: IMU loop stopped.")

    def run(self):
        print("SubDataReceiver: Starting.")
        vt = threading.Thread(target=self._video_loop, daemon=True)
        it = threading.Thread(target=self._imu_loop, daemon=True)
        vt.start()
        it.start()
        vt.join()
        it.join()

    def stop(self): self.stop_event.set()


class CombinedVisualizer:  # CORRECT
    """Manages all GUI elements (Pygame and Matplotlib) in a single window."""

    def __init__(self, data_receiver):
        self.data_receiver = data_receiver
        self.screen_width = 1280
        self.camera_width = 640
        self.plot_width = self.screen_width - self.camera_width
        self.screen_height = 480
        self.screen = pygame.display.set_mode(
            (self.screen_width, self.screen_height))
        pygame.display.set_caption("Submarine Control Dashboard")
        self.font = pygame.font.SysFont("monospace", 16)

        # Store base pyramid vertices that will be rotated (using exact structure from working file 2, but rotated to point towards +Z)
        self.pyramid_verts_base = np.array([
            [-0.6, 0, 0],       # Apex pointing towards -X
            [0.2, -0.4, -0.4],  # Base vertex 1
            [0.2, 0.4, -0.4],   # Base vertex 2
            [0.2, 0.4, 0.4],    # Base vertex 3
            [0.2, -0.4, 0.4]    # Base vertex 4
        ])

        # Use exact same face indices as working file 2
        self.pyramid_faces_indices = [[0, 1, 2], [
            0, 2, 3], [0, 3, 4], [0, 4, 1], [1, 4, 3, 2]]

        # Use exact same colors as working file 2
        # self.pyramid_face_colors = ['red','red','red','red','cyan']
        self.pyramid_face_colors = ['cyan', 'cyan', 'cyan', 'cyan', 'red']

        plot_dpi = 100
        self.fig = plt.figure(figsize=(
            self.plot_width / plot_dpi, self.screen_height / plot_dpi), dpi=plot_dpi)
        self.ax_3d = self.fig.add_subplot(1, 1, 1, projection='3d')

        # Create empty pyramid collection that will be updated dynamically
        self.pyramid_collection = Poly3DCollection([], facecolors=self.pyramid_face_colors,
                                                   linewidths=0.8, edgecolors='k', alpha=0.6)

        self.setup_3d_plot()

        # self.rotation_fix = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        # self.rotation_fix = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., 1.]])
        self.rotation_fix = np.eye(3)

        self.hud_data = {'armed': False, 'light_on': False,
                         'stabilization': True, 'yaw_kp': 0.0, 'trans_scale': 0.8}

    def setup_3d_plot(self):
        """Initializes the Matplotlib 3D plot properties."""
        self.ax_3d.set_xlabel('X (Fwd)')
        self.ax_3d.set_ylabel('Y (Right)')
        self.ax_3d.set_zlabel('Z (Up)')

        self.ax_3d.set_xlim([-1, 1])
        self.ax_3d.set_ylim([-1, 1])
        self.ax_3d.set_zlim([-1, 1])
        try:
            self.ax_3d.set_box_aspect([1, 1, 1])
        except AttributeError:
            pass
        # Set view angle to match the perspective in the images
        self.ax_3d.view_init(elev=15, azim=90)

        colors = ['red', 'green', 'blue']
        labels = ['X (Roll)', 'Y (Pitch)', 'Z (Yaw)']

        self.frame_lines = [self.ax_3d.plot([], [], [], c=c, lw=3, label=l)[
            0] for c, l in zip(colors, labels)]

        self.ax_3d.legend()

        # Add the dynamic pyramid collection to the plot
        self.ax_3d.add_collection3d(self.pyramid_collection)
        self.fig.tight_layout(pad=0)

    def _quaternion_to_rotation_matrix(self, q):
        q0, q1, q2, q3 = q
        return np.array([
            [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])

    def update(self, imu_override: Optional[np.ndarray] = None):
        """Update and draw all UI elements to the single Pygame window."""
        self.screen.fill((25, 25, 25))
        with self.data_receiver.frame_lock:
            frame = self.data_receiver.latest_frame
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            camera_surface = pygame.surfarray.make_surface(np.rot90(frame_rgb))
            self.screen.blit(camera_surface, (0, 0))

        # --- Existing HUD ---
        arm_text = self.font.render(f"ARMED: {
                                    self.hud_data['armed']}", 1, (255, 0, 0) if self.hud_data['armed'] else (0, 255, 0))
        light_text = self.font.render(
            f"LIGHT: {'ON' if self.hud_data['light_on'] else 'OFF'}", 1, (255, 255, 0))
        stab_text = self.font.render(
            f"STABILIZE: {'ON' if self.hud_data.get('stabilization', False) else 'OFF'}", 1, (0, 255, 255))
        kp_text = self.font.render(
            f"Yaw Kp: {self.hud_data['yaw_kp']:.2f}", 1, (255, 255, 255))
        scale_text = self.font.render(
            f"Move Scale: {self.hud_data.get('trans_scale', 0.8):.2f}", 1, (255, 255, 255))

        self.screen.blit(arm_text, (10, 10))
        self.screen.blit(light_text, (10, 30))
        self.screen.blit(stab_text, (10, 50))
        self.screen.blit(kp_text, (10, 70))
        self.screen.blit(scale_text, (10, 90))

        # --- MODIFICATION START: Draw IMU Euler Angles ---
        imu_data = self.data_receiver.latest_imu_data

        # Handle IMU override (from simulator when real data unavailable)
        if imu_data is None and imu_override is not None:
            w, x, y, z = imu_override
            r = np.arctan2(2*(w*x+y*z), 1-2*(x**2+y**2))
            p = np.arcsin(np.clip(2*(w*y-z*x), -1, 1))
            y_angle = np.arctan2(2*(w*z+x*y), 1-2*(y**2+z**2))
            roll, pitch, yaw = np.degrees(
                r), np.degrees(p), np.degrees(y_angle)
            imu_data = {'quaternion': imu_override, 'euler_deg': {
                'roll': roll, 'pitch': pitch, 'yaw': yaw}}

        if imu_data and 'euler_deg' in imu_data:
            # Add a small separator line
            pygame.draw.line(self.screen, (100, 100, 100),
                             (10, 115), (200, 115), 1)

            # Extract roll, pitch, and yaw
            roll = imu_data['euler_deg']['roll']
            pitch = imu_data['euler_deg']['pitch']
            yaw = imu_data['euler_deg']['yaw']

            # Render the text for each angle
            roll_text = self.font.render(
                f"Roll:  {roll:6.1f}°", 1, (255, 255, 255))
            pitch_text = self.font.render(
                f"Pitch: {pitch:6.1f}°", 1, (255, 255, 255))
            yaw_text = self.font.render(
                f"Yaw:   {yaw:6.1f}°", 1, (255, 255, 255))

            # Draw the text on the screen below the other HUD elements
            self.screen.blit(roll_text, (10, 125))
            self.screen.blit(pitch_text, (10, 145))
            self.screen.blit(yaw_text, (10, 165))
        # --- MODIFICATION END ---

        # --- 3D Plot Logic: Update both axes and pyramid ---
        if imu_data and 'quaternion' in imu_data and not np.any(np.isnan(imu_data['quaternion'])):
            q = np.array(imu_data['quaternion'])
            R = self.rotation_fix @ self._quaternion_to_rotation_matrix(q)

            # Update the IMU axes
            x_ax, y_ax, z_ax = R[:, 0], R[:, 1], R[:, 2]

            self.frame_lines[0].set_data([0, x_ax[0]], [0, x_ax[1]])
            self.frame_lines[0].set_3d_properties([0, x_ax[2]])
            self.frame_lines[1].set_data([0, y_ax[0]], [0, y_ax[1]])
            self.frame_lines[1].set_3d_properties([0, y_ax[2]])
            self.frame_lines[2].set_data([0, z_ax[0]], [0, z_ax[1]])
            self.frame_lines[2].set_3d_properties([0, z_ax[2]])

            # Update the pyramid using the exact same rotation matrix (child of IMU axes)
            rotated_verts = self.pyramid_verts_base @ R.T
            new_faces = [[rotated_verts[i] for i in face]
                         for face in self.pyramid_faces_indices]
            self.pyramid_collection.set_verts(new_faces)

            self.ax_3d.set_title('3D Orientation (Live)')
        else:
            # Clear the pyramid when no data is available
            self.pyramid_collection.set_verts([])
            self.ax_3d.set_title('3D Orientation (Waiting...)')

        self.fig.canvas.draw()
        size = self.fig.canvas.get_width_height()
        buffer = self.fig.canvas.buffer_rgba()
        plot_surface = pygame.image.frombuffer(buffer, size, "RGBA")
        self.screen.blit(plot_surface, (self.camera_width, 0))
        pygame.display.flip()


class AUV:  # CORRECT
    """
    This is the advanced AUV (simulator driver) class from the second script.
    It uses a more robust, multi-step communication protocol.
    """

    def __init__(self, server_ip="127.0.0.1", server_port=60001):
        self.server_ip, self.server_port = server_ip, server_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.settimeout(15.0)
        self.socket.connect((self.server_ip, self.server_port))
        print(f"AUV: Connected to MuJoCo server at {
              self.server_ip}:{self.server_port}")

    def _send(self, data: bytes, expected_bytes: int) -> bytes:
        if not self.socket:
            raise RuntimeError("Not connected to server")
        try:
            self.socket.sendall(data)
            response = b''
            start_time = time.time()
            while len(response) < expected_bytes and time.time() - start_time < 10.0:
                chunk = self.socket.recv(expected_bytes - len(response))
                if not chunk:
                    raise ConnectionError("Server closed connection")
                response += chunk
            if len(response) < expected_bytes:
                raise TimeoutError("Command timed out")
            return response
        except Exception as e:
            print(f"AUV: Communication error: {e}")
            raise

    def get_sensor_data(self) -> Optional[dict]:
        data = struct.pack('<f', float(MsgHeader.GET_SENSORDATA.value))
        response = self._send(data, 20)
        if len(response) >= 20:
            values = struct.unpack('<5f', response)
            return {'imu_quaternion': np.array(values[:4]), 'time': values[4]}
        return None

    def apply_ctrl(self, forces: np.ndarray, num_steps: int = 1):
        if len(forces) != 6:
            raise ValueError("Forces array must have 6 elements")
        data = struct.pack('<8f', float(
            MsgHeader.APPLY_CTRL.value), float(num_steps), *forces)
        response = self._send(data, 20)
        if len(response) >= 20:
            values = struct.unpack('<5f', response)
            return {'imu_quaternion': np.array(values[:4]), 'time': values[4]}
        return None

    def reset(self) -> bool:
        pose = np.array([0, 0, 0, 1, 0, 0, 0])
        vel = np.zeros(3)
        ang_vel = np.zeros(3)
        data = struct.pack('<15f', float(
            MsgHeader.RESET.value), 0.0, *pose, *vel, *ang_vel)
        response = self._send(data, 1)
        return len(response) == 1 and response[0] == 0

    def close(self):
        if self.socket:
            self.socket.close()


class RealSubmarineClient:  # CORRECT
    def __init__(self, server_ip, control_port=10000):
        self.server_address = (server_ip, control_port)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"RealSubClient: Ready for {self.server_address}")
        self.NEUTRAL_PULSE = 1500
        self.LIGHT_OFF = 1100
        self.LIGHT_ON = 1900
        self.AMP = 300
        self.LOOP_DURATION = 0.02

    def _to_pwm(self, nf):
        return int(self.NEUTRAL_PULSE+nf*self.AMP)

    def _send_packet(self, cmd_dict):
        try:
            payload = (json.dumps(cmd_dict)+'\n').encode('utf-8')
            self.control_socket.sendto(payload, self.server_address)
        except Exception as e:
            print(f"RealSubClient Error: {e}", file=sys.stderr)

    def send_control(self, armed, light_on, forces):
        if not armed:
            forces = np.zeros(6)
        nf = np.clip(forces/100.0, -1.0, 1.0)
        tp = [self._to_pwm(v) for v in nf]
        light = self.LIGHT_ON if light_on else self.LIGHT_OFF
        cmd = {'command': 'control', 'thruster_pulses': tp,
               'light_pulse': light, 'duration': self.LOOP_DURATION}
        self._send_packet(cmd)

    def send_reset_command(self):
        print(f"Sending 'reset_orientation' command to {
              self.server_address}...")
        cmd = {'command': 'reset_orientation'}
        self._send_packet(cmd)

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


class BlueROVController:  # CORRECT
    def __init__(self, auv_ip, auv_port, sub_ip):
        # AUV (Simulator) and Pygame Initialization
        self.auv = AUV(auv_ip, auv_port)
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad connected.")
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print(f"Gamepad: {self.controller.get_name()}")

        # Real Submarine and Visualizer Initialization
        self.real_sub_client = None
        self.data_receiver_thread = None
        self.visualizer = None
        if sub_ip:
            self.real_sub_client = RealSubmarineClient(sub_ip)
        # Data receiver is always needed for the visualizer
        self.data_receiver_thread = SubmarineDataReceiverThread()

        # System State
        self.armed = False
        self.light_on = False
        self.stabilization_enabled = True

        # Force and Control Limits
        self.minForce = np.full(6, -100)
        self.maxForce = np.full(6, 100)
        self.manual_force_scale = 100.0
        self.translation_scale = 0.8
        self.rotation_scale = 0.4

        # PID Configuration
        self.pid_gains = {
            'roll': PIDGains(kp=0.0, ki=0.0, kd=0.0),
            'pitch': PIDGains(kp=0.0, ki=0.0, kd=0.0),
            'yaw': PIDGains(kp=0.0, ki=0.0, kd=0.0)
        }
        self.pid_states = {
            'roll': PIDState(), 'pitch': PIDState(), 'yaw': PIDState()}
        self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.angle_deadzone = {'roll': 3.0, 'pitch': 3.0, 'yaw': 5.0}
        self.stabilization_force_multiplier = 0.1
        self.stabilization_force_max = 15.0

        # Tuning Parameters
        self.kp_step = 0.1
        self.translation_scale_step = 0.02
        self.translation_scale_min, self.translation_scale_max = 0.05, 0.8

        # Controller Mapping
        self.LEFT_STICK_X = 0
        self.LEFT_STICK_Y = 1
        self.RIGHT_STICK_X = 2
        self.LEFT_TRIGGER = 4
        self.RIGHT_TRIGGER = 5
        self.BTN_A = 0
        self.BTN_B = 1
        self.BTN_X = 2
        self.BTN_Y = 3
        self.BTN_VIEW = 4
        self.BTN_MENU = 6
        self.BTN_LEFT_STICK = 7
        self.BTN_LB = 9
        self.BTN_RB = 10
        self.BTN_HAT_UP = 11
        self.BTN_HAT_DOWN = 12
        self.BTN_HAT_LEFT = 13
        self.BTN_HAT_RIGHT = 14
        self.BTN_MIDDLE = 15
        self.DEADZONE = 0.1

        # Loop Timing
        self.dt = 0.02
        self.last_time = time.time()
        self.last_tuning_time = 0.0
        self.tuning_cooldown = 0.2

    def apply_deadzone(self, value: float) -> float:
        if abs(value) < self.DEADZONE:
            return 0.0
        else:
            sign = np.sign(value)
            magnitude = abs(value) - self.DEADZONE
            scale = 1.0 - self.DEADZONE
            return sign * (magnitude / scale)

    def quaternion_to_euler(self, q: np.ndarray) -> Tuple[float, float, float]:
        w, x, y, z = q
        r = np.arctan2(2*(w*x+y*z), 1-2*(x**2+y**2))
        p = np.arcsin(np.clip(2*(w*y-z*x), -1, 1))
        y_angle = np.arctan2(2*(w*z+x*y), 1-2*(y**2+z**2))
        return r, p, y_angle

    def euler_to_quaternion(self, r: float, p: float, y: float) -> np.ndarray:
        cr, cp, cy = np.cos(r*0.5), np.cos(p*0.5), np.cos(y*0.5)
        sr, sp, sy = np.sin(r*0.5), np.sin(p*0.5), np.sin(y*0.5)
        return np.array(
            [cr*cp*cy+sr*sp*sy,
             sr*cp*cy-cr*sp*sy,
             cr*sp*cy+sr*cp*sy,
             cr*cp*sy-sr*sp*cy
             ]
        )

    def pid_control(self, axis: str, error: float) -> float:
        gains, state = self.pid_gains[axis], self.pid_states[axis]
        state.integral = np.clip(state.integral+error*self.dt, -10.0, 10.0)
        output = gains.kp*error+gains.ki*state.integral + \
            gains.kd*(error-state.prev_error)/self.dt
        state.prev_error = error
        return 0.0 if np.isnan(output) else output

    def handle_button_events(self, event):
        if event.type != pygame.JOYBUTTONDOWN:
            return

        # State changes
        if event.button == self.BTN_MENU:
            self.armed = not self.armed
            print(f"\n[{'ARMED' if self.armed else 'DISARMED'}]")
        elif event.button == self.BTN_X:
            self.stabilization_enabled = not self.stabilization_enabled
            print(f"\nStabilization: {
                  'ON' if self.stabilization_enabled else 'OFF'}")
        elif event.button == self.BTN_B:
            self.light_on = not self.light_on
            print(f"\nLIGHT: {'ON' if self.light_on else 'OFF'}")
        elif event.button == self.BTN_VIEW:
            self.use_rl_policy is not self.use_rl_policy
            if (self.use_rl_policy):
                print("Code Enabled!\n")
                self.use_rl_policy = False
            else:
                print("Code Disabled!\n")
                self.use_rl_policy = True
        elif event.button == self.BTN_MIDDLE:
            raise KeyboardInterrupt("Emergency stop button pressed")
        elif event.button == self.BTN_LEFT_STICK:
            print("\nResetting Orientation...")
            self.target_attitude = np.array([1., 0., 0., 0.])
            if self.real_sub_client:
                self.real_sub_client.send_reset_command()
            self.auv.reset()

        # Target attitude changes
        elif event.button == self.BTN_Y or event.button == self.BTN_A:
            r, p, y = self.quaternion_to_euler(self.target_attitude)
            if event.button == self.BTN_Y:
                y += np.radians(10)
            else:
                y += np.radians(-10)

            self.target_attitude = self.euler_to_quaternion(r, p, y)
            print(f"\n🎯 Target Yaw: {np.degrees(y):.1f}°")

        # Real-time tuning
        if time.time() - self.last_tuning_time > self.tuning_cooldown:
            tuned = False
            if event.button == self.BTN_HAT_UP:
                self.translation_scale = min(
                    self.translation_scale+self.translation_scale_step,
                    self.translation_scale_max
                )
                tuned = True
            elif event.button == self.BTN_HAT_DOWN:
                self.translation_scale = max(
                    self.translation_scale-self.translation_scale_step,
                    self.translation_scale_min
                )
                tuned = True
            elif event.button == self.BTN_HAT_RIGHT:
                self.pid_gains['yaw'].kp += self.kp_step
                tuned = True
            elif event.button == self.BTN_HAT_LEFT:
                self.pid_gains['yaw'].kp = max(
                    0.0, self.pid_gains['yaw'].kp-self.kp_step)
                tuned = True
            elif event.button == self.BTN_RB:
                self.pid_gains['roll'].kp += self.kp_step
                self.pid_gains['pitch'].kp += self.kp_step
                tuned = True
            elif event.button == self.BTN_LB:
                self.pid_gains['roll'].kp = max(
                    0.0, self.pid_gains['roll'].kp-self.kp_step)
                self.pid_gains['pitch'].kp = max(
                    0.0, self.pid_gains['pitch'].kp-self.kp_step)
                tuned = True
            if tuned:
                scale = self.translation_scale
                yaw_kp = self.pid_gains['yaw'].kp
                rp_kp = self.pid_gains['roll'].kp
                print(
                    f"\n🔧 Tuned: Scale={scale:.2f}, " +
                    f" YawKp={yaw_kp:.1f}, R/P Kp={rp_kp:.1f}"
                )
                self.last_tuning_time = time.time()

    def get_controller_input(self) -> Tuple[float, float, float, float]:
        surge = - \
            self.apply_deadzone(self.controller.get_axis(
                self.LEFT_STICK_Y)) * self.translation_scale
        strafe = self.apply_deadzone(self.controller.get_axis(
            self.LEFT_STICK_X)) * self.translation_scale
        yaw_cmd = - \
            self.apply_deadzone(self.controller.get_axis(
                self.RIGHT_STICK_X)) * self.rotation_scale
        heave = (
            self.controller.get_axis(self.RIGHT_TRIGGER) -
            self.controller.get_axis(self.LEFT_TRIGGER)
        ) * 0.5 * self.translation_scale
        if not self.armed:
            return 0.0, 0.0, 0.0, 0.0
        return surge, strafe, heave, yaw_cmd

    def calculate_stabilization_forces(self,
                                       current_attitude: np.ndarray
                                       ) -> np.ndarray:
        if current_attitude is None or np.any(np.isnan(current_attitude)):
            return np.zeros(6)
        _, _, current_yaw = self.quaternion_to_euler(current_attitude)
        _, _, target_yaw = self.quaternion_to_euler(self.target_attitude)
        if np.isnan(current_yaw) or np.isnan(target_yaw):
            return np.zeros(6)
        yaw_error_deg = np.degrees(target_yaw-current_yaw)
        yaw_error_deg = (yaw_error_deg+180) % 360-180
        yaw_correction = self.pid_control('yaw', yaw_error_deg)
        stab_forces = np.zeros(6)
        stab_forces[0] += yaw_correction
        stab_forces[1] -= yaw_correction
        stab_forces[2] -= yaw_correction
        stab_forces[3] += yaw_correction
        return np.clip(stab_forces*self.stabilization_force_multiplier,
                       -self.stabilization_force_max,
                       self.stabilization_force_max)

    def calculate_thruster_forces(self,
                                  surge, strafe, heave,
                                  yaw_cmd, stab_forces
                                  ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculates separate thruster force vectors for the
        simulator and the real sub.
        Returns a tuple containing (simulator_forces, real_submarine_forces).
        """
        sim_manual_f = np.zeros(6)
        real_manual_f = np.zeros(6)

        # --- Block 1: Unity Simulator Force Equations ---
        # This mapping is typically for simulators where thruster directions
        # are uniform.
        sim_manual_f[0] = surge - strafe + yaw_cmd
        sim_manual_f[1] = surge + strafe - yaw_cmd
        sim_manual_f[2] = -surge - strafe - yaw_cmd
        sim_manual_f[3] = -surge + strafe + yaw_cmd
        sim_manual_f[4] = heave
        sim_manual_f[5] = heave

        # --- Block 2: Real Submarine Force Equations ---
        # This mapping accounts for the physical wiring and orientation
        # of the BlueROV2 thrusters.
        # Note the inverted surge on thrusters 2 & 3 and inverted
        # heave on thruster 5.
        real_manual_f[0] = surge - strafe + yaw_cmd
        real_manual_f[1] = surge + strafe - yaw_cmd
        real_manual_f[2] = surge + strafe + yaw_cmd
        real_manual_f[3] = surge - strafe - yaw_cmd
        real_manual_f[4] = heave
        real_manual_f[5] = -heave

        # Apply scaling and stabilization to both vectors independently
        total_sim_forces = sim_manual_f * self.manual_force_scale
        total_real_forces = real_manual_f * self.manual_force_scale

        if self.stabilization_enabled:
            total_sim_forces += stab_forces
            total_real_forces += stab_forces

        # Clip final forces and return both vectors
        clipped_sim_forces = np.clip(
            total_sim_forces, self.minForce, self.maxForce)
        clipped_real_forces = np.clip(
            total_real_forces, self.minForce, self.maxForce)

        return clipped_sim_forces, clipped_real_forces

    def run_controller(self, num_steps):
        print("🚀 Starting controller loop. Press START to arm.")
        self.visualizer = CombinedVisualizer(self.data_receiver_thread)
        self.data_receiver_thread.start()

        shutdown = False
        try:
            while not shutdown:
                if time.time()-self.last_time < self.dt:
                    time.sleep(0.002)
                    continue
                self.last_time = time.time()

                for e in pygame.event.get():
                    if e.type == pygame.QUIT:
                        shutdown = True
                        break
                    self.handle_button_events(e)
                if shutdown:
                    continue

                sim_data = self.auv.get_sensor_data()
                real_data = self.data_receiver_thread.latest_imu_data

                current_attitude = None
                if real_data and 'quaternion' in real_data:
                    current_attitude = np.array(real_data['quaternion'])
                elif sim_data and 'imu_quaternion' in sim_data:
                    current_attitude = sim_data['imu_quaternion']
                if current_attitude is None:
                    print("No IMU data available.")
                    continue

                surge, strafe, heave, yaw_cmd = self.get_controller_input()
                stab_forces = self.calculate_stabilization_forces(
                    current_attitude)

                # --- MODIFIED SECTION ---
                # Get both sets of forces calculated independently
                sim_forces, real_forces = self.calculate_thruster_forces(
                    surge, strafe, heave, yaw_cmd, stab_forces)

                # Send simulator-specific forces to the simulator
                sim_forces_to_send = sim_forces if self.armed else np.zeros(6)
                self.auv.apply_ctrl(sim_forces_to_send, num_steps)

                # If the real sub client exists, send it the
                # real-sub-specific forces
                if self.real_sub_client:
                    # Note: We use 'real_forces' here, not 'sim_forces'
                    self.real_sub_client.send_control(
                        self.armed, self.light_on, real_forces)
                self.visualizer.hud_data.update(
                    {'armed': self.armed,
                     'light_on': self.light_on,
                     'stabilization': self.stabilization_enabled,
                     'yaw_kp': self.pid_gains['yaw'].kp,
                     'trans_scale': self.translation_scale
                     }
                )
                self.visualizer.update(imu_override=current_attitude)

        except (KeyboardInterrupt, SystemExit) as e:
            print(f"\nController stopped: {e}")
        finally:
            print("Disarming and stopping systems...")
            self.data_receiver_thread.stop()
            self.data_receiver_thread.join(timeout=2)
            plt.close('all')
            try:
                self.auv.apply_ctrl(np.zeros(6), 1)
                self.auv.close()
            except Exception as e:
                print(
                    f"Could not cleanly close AUV (simulator) connection: {e}")
            if self.real_sub_client:
                self.real_sub_client.shutdown()
            pygame.quit()
            print("Cleanup complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Merged BlueROV2 Controller with correct components")
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="IP of the MuJoCo/Unity simulator.")
    parser.add_argument("--port", type=int, default=60001,
                        help="Port of the MuJoCo/Unity server.")
    parser.add_argument("--num_steps", type=int, default=1,
                        help="Simulator physics steps per cycle.")
    parser.add_argument("--sub_ip", type=str, default=None,
                        help="IP of real submarine.")
    args = parser.parse_args()
    try:
        controller = BlueROVController(args.ip, args.port, args.sub_ip)
        controller.run_controller(args.num_steps)
    except Exception as e:
        print(f"\n[FATAL ERROR] Could not start: {e}", file=sys.stderr)
        pygame.quit()
        pygame.quit()
        sys.exit(1)
