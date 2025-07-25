#!/usr/bin/env python3
"""
Async BlueROV2 Controller with significant networking performance improvements.

Key improvements:
- Async UDP socket operations for minimal latency.
- Non-blocking network calls that don't stall the control loop.
- Concurrent command execution for real robot and simulator.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
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
import asyncio
import concurrent.futures

# Use a non-interactive backend for Matplotlib
import matplotlib
matplotlib.use("Agg")


class MsgHeader(Enum):
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


get_sensor_data_cleint_recv_bytes = 20
ACK = 21  # 1 byte status + 5 floats (20 bytes)


class SubmarineDataReceiverThread(threading.Thread):
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
                    fr = cv2.imdecode(
                        np.frombuffer(jpeg, dtype=np.uint8), 
                        cv2.IMREAD_COLOR
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
                self.latest_imu_data = json.loads(d.decode('utf-8'))
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
        self.screen_width = 1280
        self.camera_width = 640
        self.plot_width = self.screen_width - self.camera_width
        self.screen_height = 480
        
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Submarine Control Dashboard")
        self.font = pygame.font.SysFont("monospace", 16)

        # Pyramid vertices for 3D visualization
        self.pyramid_verts_base = np.array([
            [-0.6, 0, 0], 
            [0.2, -0.4, -0.4], 
            [0.2, 0.4, -0.4],
            [0.2, 0.4, 0.4], 
            [0.2, -0.4, 0.4]
        ])
        
        self.pyramid_faces_indices = [
            [0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1], [1, 4, 3, 2]
        ]
        self.pyramid_face_colors = ['cyan', 'cyan', 'cyan', 'cyan', 'red']

        # Setup matplotlib plot
        plot_dpi = 100
        fig_size = (self.plot_width / plot_dpi, self.screen_height / plot_dpi)
        self.fig = plt.figure(figsize=fig_size, dpi=plot_dpi)
        self.ax_3d = self.fig.add_subplot(1, 1, 1, projection='3d')
        
        self.pyramid_collection = Poly3DCollection(
            [], 
            facecolors=self.pyramid_face_colors, 
            linewidths=0.8, 
            edgecolors='k', 
            alpha=0.6
        )
        
        self.setup_3d_plot()
        self.rotation_fix = np.eye(3)
        
        self.hud_data = {
            'armed': False, 
            'light_on': False, 
            'stabilization': True, 
            'yaw_kp': 0.0, 
            'trans_scale': 0.8
        }

    def setup_3d_plot(self):
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
            
        self.ax_3d.view_init(elev=15, azim=90)
        
        # Create frame lines for coordinate axes
        colors = ['red', 'green', 'blue']
        labels = ['X (Roll)', 'Y (Pitch)', 'Z (Yaw)']
        self.frame_lines = [
            self.ax_3d.plot([], [], [], c=c, lw=3, label=l)[0] 
            for c, l in zip(colors, labels)
        ]
        
        self.ax_3d.legend()
        self.ax_3d.add_collection3d(self.pyramid_collection)
        self.fig.tight_layout(pad=0)

    def _quaternion_to_rotation_matrix(self, q):
        q0, q1, q2, q3 = q
        return np.array([
            [q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]
        ])

    def update(self, imu_override: Optional[np.ndarray] = None):
        self.screen.fill((25, 25, 25))
        
        # Display camera feed
        with self.data_receiver.frame_lock:
            frame = self.data_receiver.latest_frame
            
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            camera_surface = pygame.surfarray.make_surface(np.rot90(frame_rgb))
            self.screen.blit(camera_surface, (0, 0))

        # HUD display
        armed_color = (255, 0, 0) if self.hud_data['armed'] else (0, 255, 0)
        self.screen.blit(
            self.font.render(f"ARMED: {self.hud_data['armed']}", 1, armed_color), 
            (10, 10)
        )
        
        light_text = "ON" if self.hud_data['light_on'] else "OFF"
        self.screen.blit(
            self.font.render(f"LIGHT: {light_text}", 1, (255, 255, 0)), 
            (10, 30)
        )
        
        stab_text = "ON" if self.hud_data.get('stabilization', False) else "OFF"
        self.screen.blit(
            self.font.render(f"STABILIZE: {stab_text}", 1, (0, 255, 255)), 
            (10, 50)
        )
        
        self.screen.blit(
            self.font.render(f"Sim Kp: {self.hud_data['yaw_kp']:.2f}", 1, (255, 255, 255)), 
            (10, 70)
        )
        
        real_kp = self.hud_data.get('yaw_kp_real', 0.0)
        self.screen.blit(
            self.font.render(f"Real Kp: {real_kp:.2f}", 1, (255, 255, 255)), 
            (10, 90)
        )
        
        trans_scale = self.hud_data.get('trans_scale', 0.8)
        self.screen.blit(
            self.font.render(f"Move Scale: {trans_scale:.2f}", 1, (255, 255, 255)), 
            (10, 110)
        )

        # Get IMU data
        imu_data = self.data_receiver.latest_imu_data
        if imu_data is None and imu_override is not None:
            w, x, y, z = imu_override
            r = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
            p = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
            y_angle = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
            
            imu_data = {
                'quaternion': imu_override,
                'euler_deg': {
                    'roll': np.degrees(r),
                    'pitch': np.degrees(p),
                    'yaw': np.degrees(y_angle)
                }
            }

        # Display attitude information
        if imu_data and 'euler_deg' in imu_data:
            pygame.draw.line(self.screen, (100, 100, 100), (10, 135), (200, 135), 1)
            
            roll_text = f"Roll:  {imu_data['euler_deg']['roll']:6.1f}°"
            self.screen.blit(self.font.render(roll_text, 1, (255, 255, 255)), (10, 145))
            
            pitch_text = f"Pitch: {imu_data['euler_deg']['pitch']:6.1f}°"
            self.screen.blit(self.font.render(pitch_text, 1, (255, 255, 255)), (10, 165))
            
            yaw_text = f"Yaw:   {imu_data['euler_deg']['yaw']:6.1f}°"
            self.screen.blit(self.font.render(yaw_text, 1, (255, 255, 255)), (10, 185))

        # Update 3D visualization
        if (imu_data and 'quaternion' in imu_data and 
            not np.any(np.isnan(imu_data['quaternion']))):
            
            q = np.array(imu_data['quaternion'])
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
                [rotated_verts[i] for i in face] 
                for face in self.pyramid_faces_indices
            ]
            self.pyramid_collection.set_verts(pyramid_faces)
            self.ax_3d.set_title('3D Orientation (Live)')
        else:
            self.pyramid_collection.set_verts([])
            self.ax_3d.set_title('3D Orientation (Waiting...)')

        # Render matplotlib plot to pygame surface
        self.fig.canvas.draw()
        plot_surface = pygame.image.frombuffer(
            self.fig.canvas.buffer_rgba(), 
            self.fig.canvas.get_width_height(), 
            "RGBA"
        )
        self.screen.blit(plot_surface, (self.camera_width, 0))
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
                
                remote_addr = (self.server_ip, self.server_port)
                self.transport, _ = await loop.create_datagram_endpoint(
                    lambda: self.protocol, 
                    remote_addr=remote_addr
                )
                
                self.connected = True
                print(f"AsyncAUV: UDP transport ready for {self.server_ip}:{self.server_port}")
                
            except Exception as e:
                self.connected = False
                print(f"AsyncAUV: UDP transport creation failed: {e}")
                raise

    async def _send_command(self, data: bytes) -> Optional[bytes]:
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                await self._ensure_transport()
                
                # Clear any pending messages
                while not self.protocol.recv_queue.empty():
                    self.protocol.recv_queue.get_nowait()
                    
                self.transport.sendto(data)
                return await asyncio.wait_for(self.protocol.recv_queue.get(), timeout=2.0)
                
            except asyncio.TimeoutError:
                if attempt == max_retries - 1:
                    print(f"AsyncAUV: Command timed out after {max_retries} attempts")
                    
            except Exception as e:
                print(f"AsyncAUV: Unexpected error: {e}")
                self.connected = False
                await asyncio.sleep(0.1)
                
        return None

    async def get_sensor_data(self) -> Optional[dict]:
        data = struct.pack('<f', float(MsgHeader.GET_SENSORDATA.value))
        response = await self._send_command(data)
        
        if response and len(response) >= get_sensor_data_cleint_recv_bytes:
            try:
                values = struct.unpack('<5f', response)
                return {
                    'imu_quaternion': np.array(values[:4]),
                    'time': values[4]
                }
            except struct.error:
                return None
                
        return None

    async def apply_ctrl(self, forces: np.ndarray, num_steps: int = 1) -> Optional[dict]:
        if len(forces) != 6:
            raise ValueError("Forces array must have 6 elements")
            
        data = struct.pack('<8f', float(MsgHeader.APPLY_CTRL.value), float(num_steps), *forces)
        response = await self._send_command(data)
        
        if response and response[0] == 0x00 and len(response) >= ACK:
            try:
                values = struct.unpack('<5f', response[1:])
                return {
                    'imu_quaternion': np.array(values[:4]),
                    'time': values[4]
                }
            except struct.error:
                return None
                
        return None

    async def reset(self) -> bool:
        pose = np.array([10.25, 6.71, 0, 1, 0, 0, 0])
        vel = np.zeros(3)
        ang_vel = np.zeros(3)
        
        data = struct.pack('<15f', float(MsgHeader.RESET.value), 0.0, *pose, *vel, *ang_vel)
        response = await self._send_command(data)
        
        return response is not None

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
        
        # Constants
        self.NEUTRAL_PULSE = 1500
        self.LIGHT_OFF = 1100
        self.LIGHT_ON = 1900
        self.AMP = 300
        self.LOOP_DURATION = 0.02

    def _to_pwm(self, nf):
        return int(self.NEUTRAL_PULSE + nf * self.AMP)

    def _send_packet(self, cmd_dict):
        try:
            message = (json.dumps(cmd_dict) + '\n').encode('utf-8')
            self.control_socket.sendto(message, self.server_address)
        except Exception as e:
            print(f"RealSubClient Error: {e}", file=sys.stderr)

    def send_control(self, armed, light_on, forces):
        forces = np.zeros(6) if not armed else forces
        
        # Clip and convert forces to PWM
        clipped_forces = np.clip(forces / 100.0, -1.0, 1.0)
        thruster_pulses = [self._to_pwm(v) for v in clipped_forces]
        
        cmd = {
            'command': 'control',
            'thruster_pulses': thruster_pulses,
            'light_pulse': self.LIGHT_ON if light_on else self.LIGHT_OFF,
            'duration': self.LOOP_DURATION
        }
        
        self._send_packet(cmd)

    def send_reset_command(self):
        print(f"Sending 'reset_orientation' command to {self.server_address}...")
        self._send_packet({'command': 'reset_orientation'})

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
    def __init__(self, auv_ip, auv_port, sub_ip):
        self.auv = AsyncAUV(auv_ip, auv_port)
        
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad connected.")
            
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print(f"Gamepad: {self.controller.get_name()}")
        
        # Initialize real submarine client if IP provided
        self.real_sub_client = RealSubmarineClient(sub_ip) if sub_ip else None
        
        # Initialize data receiver thread
        self.data_receiver_thread = SubmarineDataReceiverThread()
        
        # Control state
        self.armed = False
        self.light_on = False
        self.stabilization_enabled = True
        
        # Force limits
        self.minForce = np.full(6, -100)
        self.maxForce = np.full(6, 100)
        
        # Control scaling
        self.manual_force_scale = 100.0
        self.translation_scale = 0.8
        self.rotation_scale = 0.4
        
        # PID control setup
        self.pid_gains = {
            'roll': PIDGains(0, 0, 0),
            'pitch': PIDGains(0, 0, 0),
            'yaw': PIDGains(0, 0, 0),
            'yaw_sim': PIDGains(1.0, 0, 0),  # Start sim at 1.0
            'yaw_real': PIDGains(0.0, 0, 0)  # Start real at 0.0
        }
        
        self.pid_states = {
            'roll': PIDState(),
            'pitch': PIDState(),
            'yaw': PIDState(),
            'yaw_sim': PIDState(),  # Separate state for sim
            'yaw_real': PIDState()  # Separate state for real
        }
        
        self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Tuning parameters
        self.kp_step = 0.1
        self.translation_scale_step = 0.02
        self.translation_scale_min = 0.05
        self.translation_scale_max = 0.8
        
        # Controller button/axis mappings
        self.LEFT_STICK_X = 0
        self.LEFT_STICK_Y = 1
        self.RIGHT_STICK_X = 2
        self.LEFT_TRIGGER = 4
        self.RIGHT_TRIGGER = 5
        
        self.BTN_A = 0
        self.BTN_B = 1
        self.BTN_X = 2
        self.BTN_Y = 3
        self.BTN_MENU = 6
        self.BTN_LEFT_STICK = 7
        self.BTN_LB = 9
        self.BTN_RB = 10
        self.BTN_HAT_UP = 11
        self.BTN_HAT_DOWN = 12
        self.BTN_HAT_LEFT = 13
        self.BTN_HAT_RIGHT = 14
        self.BTN_MIDDLE = 15
        
        # Control parameters
        self.DEADZONE = 0.1
        self.dt = 0.02
        self.tuning_cooldown = 0.2
        self.last_tuning_time = 0.0
        
        # Performance monitoring
        self.loop_times = []
        self.network_times = []

    def apply_deadzone(self, v: float) -> float:
        if abs(v) < self.DEADZONE:
            return 0.0
        return np.sign(v) * (abs(v) - self.DEADZONE) / (1.0 - self.DEADZONE)

    def quaternion_to_euler(self, q: np.ndarray) -> Tuple[float, float, float]:
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        r = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        
        # Pitch (y-axis rotation)
        p = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        
        # Yaw (z-axis rotation)
        y_angle = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        
        return r, p, y_angle

    def euler_to_quaternion(self, r: float, p: float, y: float) -> np.ndarray:
        cr, cp, cy = np.cos(r*0.5), np.cos(p*0.5), np.cos(y*0.5)
        sr, sp, sy = np.sin(r*0.5), np.sin(p*0.5), np.sin(y*0.5)
        
        return np.array([
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy
        ])

    def pid_control(self, axis: str, error: float) -> float:
        g = self.pid_gains[axis]
        s = self.pid_states[axis]
        
        # Update integral with clamping
        s.integral = np.clip(s.integral + error * self.dt, -10.0, 10.0)
        
        # Calculate output
        output = (g.kp * error + 
                 g.ki * s.integral + 
                 g.kd * (error - s.prev_error) / self.dt)
        
        s.prev_error = error
        
        return 0.0 if np.isnan(output) else output

    def handle_button_events(self, event):
        if event.type != pygame.JOYBUTTONDOWN:
            return

        if event.button == self.BTN_MENU:
            self.armed = not self.armed
            print(f"\n[{'ARMED' if self.armed else 'DISARMED'}]")
            
        elif event.button == self.BTN_X:
            self.stabilization_enabled = not self.stabilization_enabled
            stab_status = 'ON' if self.stabilization_enabled else 'OFF'
            print(f"\nStabilization: {stab_status}")
            
        elif event.button == self.BTN_B:
            self.light_on = not self.light_on
            light_status = 'ON' if self.light_on else 'OFF'
            print(f"\nLIGHT: {light_status}")
            
        elif event.button == self.BTN_MIDDLE:
            raise KeyboardInterrupt("Emergency stop")
            
        elif event.button == self.BTN_LEFT_STICK:
            print("\nResetting Orientation (Target Only)...")
            self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])
            
        elif event.button == self.BTN_A:
            print("\nSending RESET signal to Sub & Sim...")
            self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])
            if self.real_sub_client:
                self.real_sub_client.send_reset_command()
            asyncio.create_task(self.auv.reset())
            
        elif event.button == self.BTN_Y:
            r, p, y_rad = self.quaternion_to_euler(self.target_attitude)
            y_deg = np.degrees(y_rad)
            new_y_deg = (y_deg + 10) % 360.0
            new_y_rad = np.radians(new_y_deg)
            self.target_attitude = self.euler_to_quaternion(r, p, new_y_rad)
            print(f"\n🎯 Target Yaw: {new_y_deg:.1f}°")

        # Handle tuning controls
        if time.time() - self.last_tuning_time > self.tuning_cooldown:
            tuned = False
            
            # UP/DOWN adjusts Real Sub Kp
            if event.button == self.BTN_HAT_UP:
                self.pid_gains['yaw_real'].kp += self.kp_step
                tuned = True
            elif event.button == self.BTN_HAT_DOWN:
                self.pid_gains['yaw_real'].kp = max(
                    0.0, self.pid_gains['yaw_real'].kp - self.kp_step)
                tuned = True
                
            # RIGHT/LEFT adjusts Simulator Kp
            elif event.button == self.BTN_HAT_RIGHT:
                self.pid_gains['yaw_sim'].kp += self.kp_step
                tuned = True
            elif event.button == self.BTN_HAT_LEFT:
                self.pid_gains['yaw_sim'].kp = max(
                    0.0, self.pid_gains['yaw_sim'].kp - self.kp_step)
                tuned = True

            if tuned:
                sim_kp = self.pid_gains['yaw_sim'].kp
                real_kp = self.pid_gains['yaw_real'].kp
                print(f"\n🔧 Tuned: SimKp={sim_kp:.2f}, RealKp={real_kp:.2f}")
                self.last_tuning_time = time.time()

    def get_controller_input(self) -> Tuple[float, float, float, float]:
        if not self.armed:
            return 0.0, 0.0, 0.0, 0.0
            
        # Get stick inputs with deadzone
        surge = (-self.apply_deadzone(self.controller.get_axis(self.LEFT_STICK_Y)) * 
                self.translation_scale)
        
        strafe = (self.apply_deadzone(self.controller.get_axis(self.LEFT_STICK_X)) * 
                 self.translation_scale)
        
        yaw_cmd = (-self.apply_deadzone(self.controller.get_axis(self.RIGHT_STICK_X)) * 
                  self.rotation_scale)
        
        # Heave from triggers
        trigger_diff = (self.controller.get_axis(self.RIGHT_TRIGGER) - 
                       self.controller.get_axis(self.LEFT_TRIGGER))
        heave = trigger_diff * 0.5 * self.translation_scale
        
        return surge, strafe, heave, yaw_cmd

    def calculate_stabilization_forces(self, current_attitude: np.ndarray, system_type: str) -> np.ndarray:
        """Calculate stabilization forces for specified system ('sim' or 'real')"""
        if current_attitude is None or np.any(np.isnan(current_attitude)):
            return np.zeros(6)
            
        # Get current and target yaw
        _, _, current_yaw = self.quaternion_to_euler(current_attitude)
        _, _, target_yaw = self.quaternion_to_euler(self.target_attitude)
        
        if np.isnan(current_yaw) or np.isnan(target_yaw):
            return np.zeros(6)
            
        # Calculate yaw error in degrees, wrapping to [-180, 180]
        yaw_error_deg = (np.degrees(target_yaw - current_yaw) + 180) % 360 - 180
        
        # Select PID gains and states based on system type
        if system_type == 'sim':
            g = self.pid_gains['yaw_sim']
            s = self.pid_states['yaw_sim']
        else:  # 'real'
            g = self.pid_gains['yaw_real']
            s = self.pid_states['yaw_real']
        
        # Update integral with clamping
        s.integral = np.clip(s.integral + yaw_error_deg * self.dt, -10.0, 10.0)
        
        # Calculate derivative
        derivative = (yaw_error_deg - s.prev_error) / self.dt
        
        # Calculate PID output
        yaw_correction = (g.kp * yaw_error_deg + 
                         g.ki * s.integral + 
                         g.kd * derivative)
        
        # Update previous error
        s.prev_error = yaw_error_deg
        
        if np.isnan(yaw_correction):
            yaw_correction = 0.0
        
        # Apply stabilization forces to thrusters
        # For simulator: positive yaw_correction should create clockwise rotation (right turn)
        if system_type == 'sim':
            stab_forces = np.array([
                yaw_correction,   # Front-left thruster
                -yaw_correction,  # Front-right thruster  
                -yaw_correction,  # Back-left thruster
                yaw_correction,   # Back-right thruster
                0,                # Vertical thruster 1
                0                 # Vertical thruster 2
            ])
        else:  # 'real'
            stab_forces = np.array([
                yaw_correction,   # Real sub thruster config
                -yaw_correction,  
                -yaw_correction,  
                yaw_correction,   
                0,                
                0                 
            ])
        
        return np.clip(stab_forces * 0.1, -15.0, 15.0)

    def calculate_thruster_forces(self, surge, strafe, heave, yaw_cmd, system_type: str, stab_forces: np.ndarray) -> np.ndarray:
        """Calculate thruster forces for specified system ('sim' or 'real')"""
        if system_type == 'sim':
            # Simulator thruster configuration
            base_forces = np.array([
                surge - strafe + yaw_cmd,
                surge + strafe - yaw_cmd,
                -surge - strafe - yaw_cmd,
                -surge + strafe + yaw_cmd,
                heave,
                heave
            ])
        else:  # 'real'
            # Real submarine thruster configuration
            base_forces = np.array([
                surge - strafe + yaw_cmd,
                surge + strafe - yaw_cmd,
                surge + strafe + yaw_cmd,
                surge - strafe - yaw_cmd,
                heave,
                -heave
            ])
        
        # Apply manual scaling and stabilization
        stabilization = stab_forces if self.stabilization_enabled else 0
        total_forces = base_forces * self.manual_force_scale + stabilization
        
        # Clip to force limits
        return np.clip(total_forces, self.minForce, self.maxForce)

    async def run_controller_async(self, num_steps, fwcmd, runtime):
        print("🚀 Starting ASYNC controller loop. Press MENU to arm.")
        
        # Initialize visualizer and data receiver
        self.visualizer = CombinedVisualizer(self.data_receiver_thread)
        self.data_receiver_thread.start()
        
        shutdown = False
        loop_count = 0
        cmdstarttime = time.time()
        
        try:
            while not shutdown:
                loop_start = time.time()
                
                # Check if runtime exceeded
                if time.time() - cmdstarttime > runtime + 0.1:
                    shutdown = True
                
                # Handle pygame events
                for e in pygame.event.get():
                    if e.type == pygame.QUIT:
                        shutdown = True
                        break
                    
                    # Handle D-pad tuning controls directly in async loop
                    if e.type == pygame.JOYBUTTONDOWN:
                        if e.button == self.BTN_MENU:
                            self.armed = not self.armed
                            print(f"\n[{'ARMED' if self.armed else 'DISARMED'}]")
                        elif e.button == self.BTN_X:
                            self.stabilization_enabled = not self.stabilization_enabled
                            stab_status = 'ON' if self.stabilization_enabled else 'OFF'
                            print(f"\nStabilization: {stab_status}")
                        elif e.button == self.BTN_B:
                            self.light_on = not self.light_on
                            light_status = 'ON' if self.light_on else 'OFF'
                            print(f"\nLIGHT: {light_status}")
                        elif e.button == self.BTN_MIDDLE:
                            raise KeyboardInterrupt("Emergency stop")
                        elif e.button == self.BTN_LEFT_STICK:
                            print("\nResetting Orientation (Target Only)...")
                            self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])
                        elif e.button == self.BTN_A:
                            print("\nSending RESET signal to Sub & Sim...")
                            self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])
                            if self.real_sub_client:
                                self.real_sub_client.send_reset_command()
                            asyncio.create_task(self.auv.reset())
                        elif e.button == self.BTN_Y:
                            r, p, y_rad = self.quaternion_to_euler(self.target_attitude)
                            y_deg = np.degrees(y_rad)
                            new_y_deg = (y_deg + 10) % 360.0
                            new_y_rad = np.radians(new_y_deg)
                            self.target_attitude = self.euler_to_quaternion(r, p, new_y_rad)
                            print(f"\n🎯 Target Yaw: {new_y_deg:.1f}°")
                        
                        # D-pad tuning controls with separate Sim/Real Kp management
                        if time.time() - self.last_tuning_time > self.tuning_cooldown:
                            tuned = False
                            
                            # UP/DOWN adjusts Simulator Yaw Kp
                            if e.button == self.BTN_HAT_UP:
                                self.pid_gains['yaw_sim'].kp += self.kp_step
                                tuned = True
                            elif e.button == self.BTN_HAT_DOWN:
                                self.pid_gains['yaw_sim'].kp = max(
                                    0.0, self.pid_gains['yaw_sim'].kp - self.kp_step)
                                tuned = True
                                
                            # LEFT/RIGHT adjusts Real Sub Yaw Kp
                            elif e.button == self.BTN_HAT_LEFT:
                                self.pid_gains['yaw_real'].kp = max(
                                    0.0, self.pid_gains['yaw_real'].kp - self.kp_step)
                                tuned = True
                            elif e.button == self.BTN_HAT_RIGHT:
                                self.pid_gains['yaw_real'].kp += self.kp_step
                                tuned = True

                            if tuned:
                                sim_kp = self.pid_gains['yaw_sim'].kp
                                real_kp = self.pid_gains['yaw_real'].kp
                                print(f"\n🔧 Tuned: SimKp={sim_kp:.2f}, RealKp={real_kp:.2f}")
                                self.last_tuning_time = time.time()
                    
                if shutdown:
                    break

                # Get controller input
                surge, strafe, heave, yaw_cmd = self.get_controller_input()
                
                # Get current attitude from real submarine or simulator
                real_data = self.data_receiver_thread.latest_imu_data
                current_attitude_sim = None
                current_attitude_real = None
                
                if real_data and 'quaternion' in real_data:
                    current_attitude_real = np.array(real_data['quaternion'])
                    
                if current_attitude_sim is None:
                    try:
                        sim_data = await asyncio.wait_for(
                            self.auv.get_sensor_data(), 
                            timeout=0.005
                        )
                        if sim_data and 'imu_quaternion' in sim_data:
                            current_attitude_sim = sim_data['imu_quaternion']
                    except asyncio.TimeoutError:
                        pass
                        
                if current_attitude_sim is None:
                    await asyncio.sleep(0.001)
                    continue

                # Calculate forces for both systems
                sim_stab_f = self.calculate_stabilization_forces(current_attitude_sim, 'sim')
                real_stab_f = self.calculate_stabilization_forces(current_attitude_real, 'real')
                
                sim_f = self.calculate_thruster_forces(surge, strafe, heave, yaw_cmd, 'sim', sim_stab_f)
                real_f = self.calculate_thruster_forces(surge, strafe, heave, yaw_cmd, 'real', real_stab_f)
                
                # Send commands to both simulator and real submarine
                tasks = [
                    self.auv.apply_ctrl(
                        sim_f if self.armed else np.zeros(6), 
                        num_steps
                    )
                ]
                
                if self.real_sub_client:
                    loop = asyncio.get_event_loop()
                    real_sub_task = loop.run_in_executor(
                        None, 
                        self.real_sub_client.send_control,
                        self.armed, 
                        self.light_on, 
                        real_f
                    )
                    tasks.append(real_sub_task)
                
                # Execute network commands with timeout
                network_start = time.time()
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*tasks, return_exceptions=True), 
                        timeout=0.015
                    )
                except asyncio.TimeoutError:
                    print("Warning: Network commands timed out")
                    
                self.network_times.append(time.time() - network_start)

                # Update visualizer
                self.visualizer.hud_data.update({
                    'armed': self.armed,
                    'light_on': self.light_on,
                    'stabilization': self.stabilization_enabled,
                    'yaw_kp': self.pid_gains['yaw_sim'].kp,  # Show sim Kp in main display
                    'yaw_kp_real': self.pid_gains['yaw_real'].kp,  # Add real Kp
                    'trans_scale': self.translation_scale
                })
                
                self.visualizer.update(imu_override=current_attitude_real)
                
                # Performance monitoring
                loop_time = time.time() - loop_start
                self.loop_times.append(loop_time)
                
                if loop_count % 100 == 0 and self.loop_times:
                    avg_loop = np.mean(self.loop_times[-100:]) * 1000
                    avg_net = np.mean(self.network_times[-100:]) * 1000
                    print(f"Perf: Loop={avg_loop:.1f}ms, Net={avg_net:.1f}ms")
                    
                loop_count += 1
                
                # Sleep for remaining time in control loop
                await asyncio.sleep(max(0, self.dt - loop_time))
                
        finally:
            print("Disarming and stopping...")
            
            # Stop data receiver thread
            self.data_receiver_thread.stop()
            self.data_receiver_thread.join(timeout=1)
            
            # Close matplotlib
            plt.close('all')
            
            # Close AUV connection
            try:
                await self.auv.apply_ctrl(np.zeros(6), 1)
                await self.auv.close()
            except Exception as e:
                print(f"Could not cleanly close AUV connection: {e}")
            
            # Shutdown real submarine client
            if self.real_sub_client:
                self.real_sub_client.shutdown()
                
            pygame.quit()
            
            # Print final performance stats
            if self.loop_times:
                avg_loop = np.mean(self.loop_times) * 1000
                avg_net = np.mean(self.network_times) * 1000
                print(f"Final Perf - Loop: {avg_loop:.1f}ms, Net: {avg_net:.1f}ms")

    def run_controller(self, num_steps, fwcmd, runtime):
        asyncio.run(self.run_controller_async(num_steps, fwcmd, runtime))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Async BlueROV2 Controller with High Performance Networking"
    )
    
    parser.add_argument(
        "--ip", 
        type=str, 
        default="127.0.0.1",
        help="IP of the MuJoCo/Unity simulator."
    )
    
    parser.add_argument(
        "--port", 
        type=int, 
        default=60001,
        help="Port of the MuJoCo/Unity server."
    )
    
    parser.add_argument(
        "--num_steps", 
        type=int, 
        default=1,
        help="Simulator physics steps per cycle."
    )
    
    parser.add_argument(
        "--sub_ip", 
        type=str, 
        default=None,
        help="IP of real submarine."
    )
    
    parser.add_argument(
        "--fwcmd", 
        type=float, 
        default=2.0,
        help="Time duration to apply forward force."
    )
    
    parser.add_argument(
        "--runtime", 
        type=float, 
        default=5.0,
        help="Time duration to send commands."
    )
    
    args = parser.parse_args()
    
    try:
        controller = BlueROVController(args.ip, args.port, args.sub_ip)
        controller.run_controller(args.num_steps, args.fwcmd, args.runtime)
    except Exception as e:
        print(f"\n[FATAL ERROR] Could not start: {e}", file=sys.stderr)
        pygame.quit()
        sys.exit(1)
