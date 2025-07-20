#!/usr/bin/env python3
"""
Complete BlueROV2 Controller System with Xbox Controller and Attitude Stabilization
Includes AUV class, sliding window buffer, deadzone system, and continuous control loop
"""

import pygame
import numpy as np
import time
import sys
import socket
import struct
import selectors
from typing import Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import argparse

# ============================================================================
# AUV CLASS DEFINITION
# ============================================================================

class MsgHeader(Enum):
    """Command message headers matching the C# server implementation."""
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

class AUV:
    def __init__(self, server_ip="127.0.0.1", server_port=60001):
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        
        # Command-specific timeouts (in seconds)
        self.command_timeouts = {
            MsgHeader.HEARTBEAT: 1.0,      # Quick response expected
            MsgHeader.GET_MODEL_INFO: 2.0,   # Model data retrieval
            MsgHeader.GET_SENSORDATA: 5.0,   # Sensor data collection
            MsgHeader.GET_RGB_IMAGE: 5.0,    # Image processing
            MsgHeader.GET_MASKED_IMAGE: 5.0, # Image processing
            MsgHeader.APPLY_CTRL: 10.0,      # Can take longer with simulation steps
            MsgHeader.STEP_SIM: 10.0,        # Can take longer with many steps
            MsgHeader.RESET: 5.0,            # Reset operations
        }
        
        self.connect()
    
    def __del__(self):
        if self.socket:
            try:
                self.socket.close()
                print("AUV: Disconnected from server")
            except:
                pass
            self.socket = None
    
    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # Optimize socket for low latency
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
            
            # Set a longer overall timeout for socket operations
            self.socket.settimeout(15.0)
            
            self.socket.connect((self.server_ip, self.server_port))
            print(f"AUV: Connected to MuJoCo server at {self.server_ip}:{self.server_port}")
            
        except Exception as e:
            print(f"AUV: Connection failed: {e}")
            raise
    
    def _send_command(self, data: bytes, expected_bytes: int, command_header: MsgHeader = None) -> bytes:
        """
        Send command and wait for complete response from server.
        
        Args:
            data: Command data to send
            expected_bytes: Expected number of response bytes
            command_header: Command header enum for timeout lookup
        
        Returns:
            Complete response bytes from server
        """
        if not self.socket:
            raise RuntimeError("Not connected to server")
        
        # Determine timeout based on command type
        timeout = self.command_timeouts.get(command_header, 5.0) if command_header else 5.0
        
        try:
            # Step 1: Send command data
            cmd_name = command_header.name if command_header else "UNKNOWN"
            self.socket.sendall(data)
            
            # Step 2: Wait for server to process and respond
            response = b""
            bytes_received = 0
            start_time = time.time()
            
            # Use a more robust approach with selectors
            sel = selectors.DefaultSelector()
            sel.register(self.socket, selectors.EVENT_READ)
            
            try:
                while bytes_received < expected_bytes:
                    elapsed_time = time.time() - start_time
                    timeout_remaining = timeout - elapsed_time
                    
                    if timeout_remaining <= 0:
                        raise TimeoutError(f"Timeout waiting for {cmd_name} response after {timeout}s. "
                                         f"Received {bytes_received}/{expected_bytes} bytes")
                    
                    # Wait for socket to become readable with remaining timeout
                    events = sel.select(timeout=min(timeout_remaining, 0.1))  # Poll every 100ms
                    
                    if not events:
                        # No data available yet, continue waiting
                        continue
                    
                    # Socket is ready for reading
                    try:
                        remaining = expected_bytes - bytes_received
                        
                        # Receive available data (up to remaining bytes needed)
                        chunk = self.socket.recv(remaining)
                        
                        if not chunk:
                            # Socket closed by server
                            raise ConnectionError("Server closed connection during response")
                        
                        response += chunk
                        bytes_received += len(chunk)
                        
                    except socket.timeout:
                        # Individual recv timed out, continue outer loop
                        continue
                    except socket.error as e:
                        if e.errno == socket.errno.EAGAIN or e.errno == socket.errno.EWOULDBLOCK:
                            # No data available right now, continue waiting
                            continue
                        else:
                            raise
            
            finally:
                sel.close()
            
            # Step 3: Verify we received the expected amount of data
            if bytes_received != expected_bytes:
                raise ValueError(f"Incomplete {cmd_name} response: received {bytes_received}/{expected_bytes} bytes")
            
            return response
            
        except Exception as e:
            print(f"AUV: Communication error for {cmd_name}: {e}")
            raise
    
    def heartbeat(self) -> bool:
        """Send heartbeat to verify connection."""
        try:
            data = struct.pack('<f', float(MsgHeader.HEARTBEAT.value))
            response = self._send_command(data, 1, MsgHeader.HEARTBEAT)
            return len(response) == 1 and response[0] == 0
        except Exception:
            return False
    
    def get_model_info(self) -> Optional[dict]:
        """Get model information from server."""
        data = struct.pack('<f', float(MsgHeader.GET_MODEL_INFO.value))
        response = self._send_command(data, 24, MsgHeader.GET_MODEL_INFO)
        
        if len(response) == 24:
            values = struct.unpack('<6f', response)
            return {
                'time': values[0],
                'nq': int(values[1]),
                'nv': int(values[2]),
                'na': int(values[3]),
                'nu': int(values[4]),
                'nbody': int(values[5])
            }
        return None
    
    def get_sensor_data(self) -> Optional[dict]:
        """Get current sensor data from simulation."""
        data = struct.pack('<f', float(MsgHeader.GET_SENSORDATA.value))
        
        # The server sends variable-length sensor data, try to get at least 20 bytes
        response = self._send_command(data, 20, MsgHeader.GET_SENSORDATA)
        
        if len(response) >= 20:
            num_floats = len(response) // 4
            values = struct.unpack(f'<{num_floats}f', response)
            
            return {
                'imu_quaternion': np.array(values[:4]),  # w, x, y, z
                'time': values[-1],  # Last value is always time
                'raw_sensor_data': np.array(values)
            }
        return None
    
    def apply_ctrl(self, forces: np.ndarray, num_steps: int = 1) -> np.ndarray:
        if len(forces) != 6:
            raise ValueError("Forces array must have exactly 6 elements")

        # Only print for debugging - comment out for continuous operation
        # print(f"AUV: Applying forces {forces} for {num_steps} steps")

        data = struct.pack('<8f', float(MsgHeader.APPLY_CTRL.value), float(num_steps), *forces)

        timeout_multiplier = max(1.0, num_steps / 10.0)
        original_timeout = self.command_timeouts[MsgHeader.APPLY_CTRL]
        self.command_timeouts[MsgHeader.APPLY_CTRL] = original_timeout * timeout_multiplier

        try:
            # Step 1: Receive success byte
            response = self._send_command(data, 1, MsgHeader.APPLY_CTRL)
            if len(response) != 1 or response[0] != 0:
                raise RuntimeError("Control command failed")

            # Step 2: Determine how many bytes to read
            model_info = self.get_model_info()
            if model_info is None:
                raise RuntimeError("Cannot fetch model info to read sensor data")
            nsensordata = model_info.get('na', 0)  # <- May need adjustment
            num_floats = nsensordata + 1
            num_floats = 5
            total_bytes = num_floats * 4

            # Step 3: Read the sensor data
            sensor_data = b""
            while len(sensor_data) < total_bytes:
                chunk = self.socket.recv(total_bytes - len(sensor_data))
                if not chunk:
                    raise RuntimeError("Socket closed before receiving full sensor data")
                sensor_data += chunk

            #  Step 4: Unpack and return
            floats = np.frombuffer(sensor_data, dtype=np.float32)
            return self.get_sensor_data()

        finally:
            self.command_timeouts[MsgHeader.APPLY_CTRL] = original_timeout

    def step_sim(self, num_steps: int = 1) -> bool:
        """Step the simulation forward."""
        data = struct.pack('<2f', float(MsgHeader.STEP_SIM.value), float(num_steps))
        
        # Use longer timeout for many simulation steps
        timeout_multiplier = max(1.0, num_steps / 10.0)
        original_timeout = self.command_timeouts[MsgHeader.STEP_SIM]
        self.command_timeouts[MsgHeader.STEP_SIM] = original_timeout * timeout_multiplier
        
        try:
            response = self._send_command(data, 1, MsgHeader.STEP_SIM)
            return len(response) == 1 and response[0] == 0
        finally:
            # Restore original timeout
            self.command_timeouts[MsgHeader.STEP_SIM] = original_timeout
    
    def reset(self, 
              pose: np.ndarray = None, 
              linear_vel: np.ndarray = None, 
              angular_vel: np.ndarray = None,
              num_steps: int = 0) -> bool:
        """Reset simulation to specified state."""

        # Default values
        if pose is None:
            pose = np.array([0, 0, 0, 1, 0, 0, 0])  # [px, py, pz, ow, ox, oy, oz]
        if linear_vel is None:
            linear_vel = np.array([0, 0, 0])  # [vx, vy, vz]
        if angular_vel is None:
            angular_vel = np.array([0, 0, 0])  # [wx, wy, wz]

        if len(pose) != 7:
            raise ValueError("Pose must have 7 elements [px, py, pz, ow, ox, oy, oz]")
        if len(linear_vel) != 3:
            raise ValueError("Linear velocity must have 3 elements [vx, vy, vz]")
        if len(angular_vel) != 3:
            raise ValueError("Angular velocity must have 3 elements [wx, wy, wz]")
        
        # Pack: [command, numSteps, pose(7), linear_vel(3), angular_vel(3)] = 15 floats
        data = struct.pack('<15f', 
                           float(MsgHeader.RESET.value),  # RESET command
                           float(num_steps),
                           *pose,
                           *linear_vel,
                           *angular_vel)
        
        response = self._send_command(data, 1, MsgHeader.RESET)
        return len(response) == 1 and response[0] == 0

# ============================================================================
# CONTROLLER CLASS DEFINITION
# ============================================================================

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
    def __init__(self, auv_ip="127.0.0.1", auv_port=60001, minMaxForces = [[-100,100],[-100,100],[-100,1000],[-100,100],[-100,100],[-100,100]]):
        """Initialize the BlueROV controller with attitude stabilization"""
        
        # Initialize AUV connection
        self.auv = AUV(auv_ip, auv_port)
        
        # Initialize pygame for controller input
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad found")
        
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        
        # Force limits
        transposed_forces = np.array(minMaxForces).T
        self.minForce = transposed_forces[0] # Array of minimum force for each thruster
        self.maxForce = transposed_forces[1] # Array of maximum force for each thruster

        # This new variable preserves the scaling of manual joystick input, separate from the final clipping limits.
        self.manual_force_scale = 100.0


        self.MAX_FORCE = 100.0
        self.MIN_FORCE = -100.0
        
        # Controller configuration
        self.setup_controller_mapping()
        
        # PID configuration for attitude control
        self.setup_pid_controllers()
        
        # Target attitude (quaternion: w, x, y, z)
        self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Control loop timing
        self.dt = 0.02  # 50Hz
        self.last_time = time.time()
        
        # Control scaling factors (now adjustable)
        self.translation_scale = 0.2  # Start with very low value for gentle movement
        self.rotation_scale = 0.4     # Scale for rotation commands
        
        # Tuning parameters - NO LIMITS, start from zero
        self.translation_scale_min = 0.05
        self.translation_scale_max = 0.8
        self.translation_scale_step = 0.02
        
        # PID tuning - no min/max, simple increments
        self.kp_step = 0.1
        self.ki_step = 0.01
        self.kd_step = 0.1
        
        # Stabilization force scaling
        self.stabilization_force_multiplier = 0.1
        self.stabilization_force_max = 15.0
        self.stabilization_mult_min = 0.01
        self.stabilization_mult_max = 0.5
        self.stabilization_mult_step = 0.01
        
        # Sliding window buffer for angle smoothing
        self.angle_buffer_size = 10
        self.angle_buffers = {
            'roll': [],
            'pitch': [],
            'yaw': []
        }
        
        # Deadzone parameters (in degrees)
        self.angle_deadzone = {
            'roll': 3.0,    # ±3 degrees
            'pitch': 3.0,   # ±3 degrees  
            'yaw': 5.0      # ±5 degrees
        }
        
        self.deadzone_min = 0.5
        self.deadzone_max = 15.0
        self.deadzone_step = 0.5
        
        # Tuning cooldown to prevent rapid changes
        self.tuning_cooldown = 0.2  # 200ms between changes
        self.last_tuning_time = 0.0
        
        # System state
        self.armed = False
        self.stabilization_enabled = True
        
        print(f"Controller initialized: {self.controller.get_name()}")
        print("Controls:")
        print("  Left Stick: Translation (surge/strafe)")
        print("  Right Stick: Rotation (yaw)")
        print("  Triggers: Vertical movement (heave)")
        print("  Start: Arm/Disarm")
        print("  Back: Emergency stop")
        print("  Y: Increase target yaw by 10°")
        print("  A: Decrease target yaw by 10°")
        print("  D-Pad Up/Down: Adjust translation force")
        print("  D-Pad Left/Right: Adjust yaw KP (±0.1)")
        print("  LB/RB: Adjust roll/pitch KP (±0.1)")
        print("  X: Toggle stabilization")
        print("  B: Increase deadzone")
        print("  LB/RB + B: Decrease deadzone")
        print(f"  Initial: Trans={self.translation_scale:.2f}, ALL KP/KD=0.0 (NO JITTER!)")
        print(f"  Deadzones: Yaw=±{self.angle_deadzone['yaw']:.1f}°, Roll/Pitch=±{self.angle_deadzone['roll']:.1f}°")
        print(f"  💡 ZERO GAINS = ZERO JITTER - Increase KP gradually to add stabilization")
        
    def setup_controller_mapping(self):
        """Setup controller axis and button mappings"""
        # Axis mappings (adjust based on your controller)
        self.LEFT_STICK_X = 0   # strafe
        self.LEFT_STICK_Y = 1   # surge
        self.RIGHT_STICK_X = 2  # yaw
        self.RIGHT_STICK_Y = 3  # pitch (unused for now)
        self.LEFT_TRIGGER = 4   # heave down
        self.RIGHT_TRIGGER = 5  # heave up
        
        # Button mappings
        self.BTN_A = 0
        self.BTN_B = 1
        self.BTN_X = 2
        self.BTN_Y = 3
        self.BTN_VIEW = 4
        self.BTN_MENU = 6
        self.BTN_XBOX = 6 # Main XBox button
        self.BTN_LEFT_STICK = 7
        self.BTN_RIGHT_STICK = 8
        self.BTN_LB = 9
        self.BTN_RB = 10
        
        # Hat/D-pad buttons for tuning
        self.BTN_HAT_UP = 11     # Increase translation force
        self.BTN_HAT_DOWN = 12   # Decrease translation force
        self.BTN_HAT_LEFT = 13   # Decrease yaw KP
        self.BTN_HAT_RIGHT = 14  # Increase yaw KP
        
        # Deadzone for controller inputs
        self.DEADZONE = 0.1
        
    def setup_pid_controllers(self):
        """Setup PID controllers for attitude stabilization"""
        # PID gains - starting at ZERO for no jitter at startup
        self.pid_gains = {
            'roll': PIDGains(kp=0.0, ki=0.0, kd=0.0),
            'pitch': PIDGains(kp=0.0, ki=0.0, kd=0.0),
            'yaw': PIDGains(kp=0.0, ki=0.0, kd=0.0)
        }
        
        # PID state tracking
        self.pid_states = {
            'roll': PIDState(),
            'pitch': PIDState(),
            'yaw': PIDState()
        }
        
    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to controller input"""
        if abs(value) < self.DEADZONE:
            return 0.0
        # Apply scaling outside deadzone
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.DEADZONE) / (1.0 - self.DEADZONE)
    
    def quaternion_to_euler(self, q: np.ndarray) -> Tuple[float, float, float]:
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert euler angles to quaternion"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])
    
    def pid_control(self, axis: str, error: float) -> float:
        """Calculate PID control output"""
        gains = self.pid_gains[axis]
        state = self.pid_states[axis]
        
        # Proportional term
        p_term = gains.kp * error
        
        # Integral term with windup protection
        state.integral += error * self.dt
        state.integral = np.clip(state.integral, -10.0, 10.0)  # Prevent windup
        i_term = gains.ki * state.integral
        
        # Derivative term
        d_term = gains.kd * (error - state.prev_error) / self.dt
        state.prev_error = error
        
        # Total PID output
        output = p_term + i_term + d_term
        
        return output
    
    def get_controller_input(self) -> Tuple[float, float, float, float]:
        """Get input from Xbox controller"""
        current_time = time.time()
        
        # Process pygame events
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == self.BTN_MENU:
                    self.armed = not self.armed
                    print(f"\n[{'ARMED' if self.armed else 'DISARMED'}]")
                elif event.button == self.BTN_X:
                    self.stabilization_enabled = not self.stabilization_enabled
                    print(f"\nStabilization: {'ON' if self.stabilization_enabled else 'OFF'}")
                elif event.button == self.BTN_VIEW:
                    print("\nEmergency stop!")
                    raise KeyboardInterrupt("Emergency stop button pressed")
                
                # Y button - increase target yaw by 10 degrees
                elif event.button == self.BTN_Y:
                    # Get current target angles
                    target_roll, target_pitch, target_yaw = self.quaternion_to_euler(self.target_attitude)
                    # Add 10 degrees (convert to radians)
                    new_target_yaw = target_yaw + np.radians(10)
                    # Wrap to [-pi, pi]
                    if new_target_yaw > np.pi:
                        new_target_yaw -= 2 * np.pi
                    # Update target attitude
                    self.target_attitude = self.euler_to_quaternion(target_roll, target_pitch, new_target_yaw)
                    print(f"\n🎯 Target Yaw: {np.degrees(new_target_yaw):.1f}° (+10°)")
                
                # A button - decrease target yaw by 10 degrees
                elif event.button == self.BTN_A:
                    # Get current target angles
                    target_roll, target_pitch, target_yaw = self.quaternion_to_euler(self.target_attitude)
                    # Subtract 10 degrees (convert to radians)
                    new_target_yaw = target_yaw - np.radians(10)
                    # Wrap to [-pi, pi]
                    if new_target_yaw < -np.pi:
                        new_target_yaw += 2 * np.pi
                    # Update target attitude
                    self.target_attitude = self.euler_to_quaternion(target_roll, target_pitch, new_target_yaw)
                    print(f"\n🎯 Target Yaw: {np.degrees(new_target_yaw):.1f}° (-10°)")
                
                # Real-time tuning controls
                elif current_time - self.last_tuning_time > self.tuning_cooldown:
                    if event.button == self.BTN_HAT_UP:
                        # Increase translation force
                        old_scale = self.translation_scale
                        self.translation_scale = min(self.translation_scale + self.translation_scale_step, self.translation_scale_max)
                        if self.translation_scale != old_scale:
                            print(f"\n🔧 Translation Force: {self.translation_scale:.2f} (↑)")
                            self.last_tuning_time = current_time
                    
                    elif event.button == self.BTN_HAT_DOWN:
                        # Decrease translation force
                        old_scale = self.translation_scale
                        self.translation_scale = max(self.translation_scale - self.translation_scale_step, self.translation_scale_min)
                        if self.translation_scale != old_scale:
                            print(f"\n🔧 Translation Force: {self.translation_scale:.2f} (↓)")
                            self.last_tuning_time = current_time
                    
                    elif event.button == self.BTN_HAT_RIGHT:
                        # Increase yaw KP
                        old_kp = self.pid_gains['yaw'].kp
                        self.pid_gains['yaw'].kp += self.kp_step
                        if self.pid_gains['yaw'].kp != old_kp:
                            print(f"\n🔧 Yaw KP: {self.pid_gains['yaw'].kp:.1f} (↑)")
                            self.last_tuning_time = current_time
                    
                    elif event.button == self.BTN_HAT_LEFT:
                        # Decrease yaw KP
                        old_kp = self.pid_gains['yaw'].kp
                        self.pid_gains['yaw'].kp = max(0.0, self.pid_gains['yaw'].kp - self.kp_step)
                        if self.pid_gains['yaw'].kp != old_kp:
                            print(f"\n🔧 Yaw KP: {self.pid_gains['yaw'].kp:.1f} (↓)")
                            self.last_tuning_time = current_time
                    
                    elif event.button == self.BTN_RB:
                        # Increase roll/pitch KP
                        old_kp = self.pid_gains['roll'].kp
                        self.pid_gains['roll'].kp += self.kp_step
                        self.pid_gains['pitch'].kp += self.kp_step
                        if self.pid_gains['roll'].kp != old_kp:
                            print(f"\n🔧 Roll/Pitch KP: {self.pid_gains['roll'].kp:.1f} (↑)")
                            self.last_tuning_time = current_time
                    
                    elif event.button == self.BTN_LB:
                        # Decrease roll/pitch KP
                        old_kp = self.pid_gains['roll'].kp
                        self.pid_gains['roll'].kp = max(0.0, self.pid_gains['roll'].kp - self.kp_step)
                        self.pid_gains['pitch'].kp = max(0.0, self.pid_gains['pitch'].kp - self.kp_step)
                        if self.pid_gains['roll'].kp != old_kp:
                            print(f"\n🔧 Roll/Pitch KP: {self.pid_gains['roll'].kp:.1f} (↓)")
                            self.last_tuning_time = current_time
                    
                    # Hold LB for KD adjustments
                    elif event.button == self.BTN_HAT_UP and self.controller.get_button(self.BTN_LB):
                        # Increase KD for all axes
                        old_kd = self.pid_gains['yaw'].kd
                        self.pid_gains['yaw'].kd += self.kd_step
                        self.pid_gains['roll'].kd += self.kd_step
                        self.pid_gains['pitch'].kd += self.kd_step
                        if self.pid_gains['yaw'].kd != old_kd:
                            print(f"\n🔧 All KD: {self.pid_gains['yaw'].kd:.1f} (↑)")
                            self.last_tuning_time = current_time
                    
                    elif event.button == self.BTN_HAT_DOWN and self.controller.get_button(self.BTN_LB):
                        # Decrease KD for all axes
                        old_kd = self.pid_gains['yaw'].kd
                        self.pid_gains['yaw'].kd = max(0.0, self.pid_gains['yaw'].kd - self.kd_step)
                        self.pid_gains['roll'].kd = max(0.0, self.pid_gains['roll'].kd - self.kd_step)
                        self.pid_gains['pitch'].kd = max(0.0, self.pid_gains['pitch'].kd - self.kd_step)
                        if self.pid_gains['yaw'].kd != old_kd:
                            print(f"\n🔧 All KD: {self.pid_gains['yaw'].kd:.1f} (↓)")
                            self.last_tuning_time = current_time
                    
                    elif event.button == self.BTN_B:
                        # Check if shoulder button is held for decrease deadzone
                        if self.controller.get_button(self.BTN_LB) or self.controller.get_button(self.BTN_RB):
                            # Decrease deadzone for all axes
                            old_deadzone = self.angle_deadzone['yaw']
                            new_deadzone = max(old_deadzone - self.deadzone_step, self.deadzone_min)
                            if new_deadzone != old_deadzone:
                                self.angle_deadzone['yaw'] = new_deadzone
                                self.angle_deadzone['roll'] = max(new_deadzone * 0.6, self.deadzone_min)
                                self.angle_deadzone['pitch'] = max(new_deadzone * 0.6, self.deadzone_min)
                                print(f"\n🔧 Deadzone - Yaw: {self.angle_deadzone['yaw']:.1f}°, Roll/Pitch: {self.angle_deadzone['roll']:.1f}° (↓)")
                                self.last_tuning_time = current_time
                        else:
                            # Increase deadzone for all axes
                            old_deadzone = self.angle_deadzone['yaw']
                            new_deadzone = min(old_deadzone + self.deadzone_step, self.deadzone_max)
                            if new_deadzone != old_deadzone:
                                self.angle_deadzone['yaw'] = new_deadzone
                                self.angle_deadzone['roll'] = min(new_deadzone * 0.6, self.deadzone_max)  # Roll/pitch smaller
                                self.angle_deadzone['pitch'] = min(new_deadzone * 0.6, self.deadzone_max)
                                print(f"\n🔧 Deadzone - Yaw: {self.angle_deadzone['yaw']:.1f}°, Roll/Pitch: {self.angle_deadzone['roll']:.1f}° (↑)")
                                self.last_tuning_time = current_time
        
        # Get stick inputs
        surge = -self.apply_deadzone(self.controller.get_axis(self.LEFT_STICK_Y))
        strafe = self.apply_deadzone(self.controller.get_axis(self.LEFT_STICK_X))
        yaw_input = -self.apply_deadzone(self.controller.get_axis(self.RIGHT_STICK_X))
        
        # Get trigger inputs for vertical movement
        left_trigger = self.controller.get_axis(self.LEFT_TRIGGER)
        right_trigger = self.controller.get_axis(self.RIGHT_TRIGGER)
        
        # Convert triggers to heave command
        heave = (right_trigger - left_trigger) * 0.5
        
        # Scale inputs with adjustable scaling
        surge *= self.translation_scale
        strafe *= self.translation_scale
        heave *= self.translation_scale
        yaw_input *= self.rotation_scale
        
        # Safety: disable all inputs if not armed
        if not self.armed:
            surge = strafe = heave = yaw_input = 0.0
            
        return surge, strafe, heave, yaw_input
    
    def calculate_thruster_forces(self, surge: float, strafe: float, heave: float, yaw_cmd: float, stabilization_forces: np.ndarray) -> np.ndarray:
        """
        Calculate forces for each thruster based on thruster configuration.
        This function now correctly combines manual joystick inputs with stabilization forces.
        
        The final control output is a sum of:
        1. Manual translation forces (surge, strafe, heave).
        2. Manual rotation force (yaw) from the joystick.
        3. PID-based stabilization forces for roll, pitch, and yaw.
        
        This allows the user to command movement while the PID controller actively corrects for orientation errors.
        
        Array format: [front_right, front_left, back_right, back_left, left_middle_top, right_middle_top]
        """
        
        # 1. Calculate manual command forces for all degrees of freedom (DOF)
        manual_forces = np.zeros(6)
        
        # Horizontal thrusters for Surge, Strafe, AND Yaw
        manual_forces[0] = surge - strafe + yaw_cmd
        manual_forces[1] = surge + strafe - yaw_cmd
        # MODIFIED: Corrected yaw logic for rear thrusters to produce rotation instead of strafe.
        manual_forces[2] = -surge - strafe - yaw_cmd
        manual_forces[3] = -surge + strafe + yaw_cmd
        
        # Vertical thrusters for Heave
        manual_forces[4] = heave
        manual_forces[5] = heave
        
        # Scale manual forces to the appropriate range
        manual_forces *= self.manual_force_scale
        
        # 2. Initialize total forces with the complete manual command
        total_forces = manual_forces
        
        # 3. Add stabilization forces if enabled
        #    This adds the PID correction on top of the manual input.
        if self.stabilization_enabled:
            # The `stabilization_forces` array contains corrections for roll, pitch, and yaw.
            total_forces += stabilization_forces
            
        # 4. Clip final forces to ensure they are within the thruster limits
        total_forces = np.clip(total_forces, self.minForce, self.maxForce)
        
        return total_forces

    def calculate_stabilization_forces(self, current_attitude: np.ndarray) -> np.ndarray:
        """
        Calculate the required forces to stabilize roll, pitch, and yaw.
        """
        # Convert quaternions to Euler angles (in radians)
        current_roll, current_pitch, current_yaw = self.quaternion_to_euler(current_attitude)
        target_roll, target_pitch, target_yaw = self.quaternion_to_euler(self.target_attitude)
        
        # Convert to degrees for easier interpretation and deadzone application
        current_roll_deg = np.degrees(current_roll)
        current_pitch_deg = np.degrees(current_pitch)
        current_yaw_deg = np.degrees(current_yaw)
        
        target_roll_deg = np.degrees(target_roll)
        target_pitch_deg = np.degrees(target_pitch)
        target_yaw_deg = np.degrees(target_yaw)

        # Update and smooth angle buffers
        def update_buffer(buffer, value):
            buffer.append(value)
            if len(buffer) > self.angle_buffer_size:
                buffer.pop(0)
            return np.mean(buffer)

        smooth_roll_deg = update_buffer(self.angle_buffers['roll'], current_roll_deg)
        smooth_pitch_deg = update_buffer(self.angle_buffers['pitch'], current_pitch_deg)
        smooth_yaw_deg = update_buffer(self.angle_buffers['yaw'], current_yaw_deg)

        # Calculate angle errors in degrees (for deadzone and PID)
        roll_error_deg = target_roll_deg - smooth_roll_deg
        pitch_error_deg = target_pitch_deg - smooth_pitch_deg
        yaw_error_deg = target_yaw_deg - smooth_yaw_deg

        # Handle yaw wrap-around (e.g., from +179 to -179 is a 2-degree error)
        if yaw_error_deg > 180:
            yaw_error_deg -= 360
        elif yaw_error_deg < -180:
            yaw_error_deg += 360
        
        # Apply deadzone to prevent jitter from small errors
        if abs(roll_error_deg) < self.angle_deadzone['roll']:
            roll_error_deg = 0.0
        if abs(pitch_error_deg) < self.angle_deadzone['pitch']:
            pitch_error_deg = 0.0
        if abs(yaw_error_deg) < self.angle_deadzone['yaw']:
            yaw_error_deg = 0.0
        
        # Calculate PID outputs based on degree errors
        roll_correction = self.pid_control('roll', roll_error_deg)
        pitch_correction = self.pid_control('pitch', pitch_error_deg)
        yaw_correction = self.pid_control('yaw', yaw_error_deg)
        
        # Map PID outputs to thruster forces
        stabilization_forces = np.zeros(6)
        
        # MODIFIED: Corrected yaw stabilization logic to match manual control correction.
        # This ensures the PID controller also produces rotation instead of strafe.
        stabilization_forces[0] += yaw_correction
        stabilization_forces[1] -= yaw_correction
        stabilization_forces[2] -= yaw_correction
        stabilization_forces[3] += yaw_correction
        
        # Roll and Pitch correction (vertical thrusters)
        stabilization_forces[4] += -roll_correction + pitch_correction
        stabilization_forces[5] += roll_correction + pitch_correction
        
        # Apply scaling and clipping
        stabilization_forces *= self.stabilization_force_multiplier
        stabilization_forces = np.clip(stabilization_forces, -self.stabilization_force_max, self.stabilization_force_max)
        
        return stabilization_forces

    def run_controller(self):
        """Main control loop"""
        print("\n🚀 Starting controller loop. Press START to arm.")
        try:
            while True:
                current_time = time.time()
                elapsed = current_time - self.last_time

                if elapsed >= self.dt:
                    self.last_time = current_time

                    # 1. Get sensor data
                    sensor_data = self.auv.get_sensor_data()
                    if not sensor_data:
                        print("Failed to get sensor data, skipping cycle.")
                        time.sleep(0.1)
                        continue
                    
                    current_attitude = sensor_data['imu_quaternion']

                    # 2. Get joystick inputs
                    surge, strafe, heave, yaw_cmd = self.get_controller_input()

                    # 3. Calculate stabilization forces from PID
                    stabilization_forces = self.calculate_stabilization_forces(current_attitude)

                    # 4. Calculate final thruster forces by combining manual and stabilization inputs
                    final_forces = self.calculate_thruster_forces(surge, strafe, heave, yaw_cmd, stabilization_forces)

                    # 5. Apply forces to AUV if armed
                    if self.armed:
                        self.auv.apply_ctrl(final_forces)
                    else:
                        # Send zero forces if disarmed to ensure safety
                        self.auv.apply_ctrl(np.zeros(6))
                        
                    # Print status (optional, can be noisy)
                    # sys.stdout.write(f"\rArmed: {self.armed} | Stab: {self.stabilization_enabled} | Forces: {np.array2string(final_forces, precision=1, separator=', ')}")
                    # sys.stdout.flush()

        except KeyboardInterrupt:
            print("\nController loop stopped by user.")
        except Exception as e:
            print(f"\nAn error occurred in the controller loop: {e}")
        finally:
            print("\nDisarming and stopping thrusters...")
            self.auv.apply_ctrl(np.zeros(6))
            pygame.quit()
            print("Cleanup complete. Exiting.")

# ============================================================================
# MAIN EXECUTION
# ============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BlueROV2 Controller with Attitude Stabilization")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="IP address of the MuJoCo server.")
    parser.add_argument("--port", type=int, default=60001, help="Port of the MuJoCo server.")
    args = parser.parse_args()

    try:
        controller = BlueROVController(auv_ip=args.ip, auv_port=args.port)
        controller.run_controller()
    except (RuntimeError, ConnectionError, TimeoutError) as e:
        print(f"\n[FATAL ERROR] Could not start controller: {e}")
        sys.exit(1)
