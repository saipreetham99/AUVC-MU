#!/usr/bin/env python3
"""
Minimal Test Script to Reset Submarine Pose in the Simulator.

This script connects to the MuJoCo simulator and sets the submarine's
position (x, y, z) and orientation (roll, pitch, yaw). This helps in
determining the necessary axis transformations for accurate tracking between
a real-world system and the simulator.

Usage:
    python reset_pose.py <x> <y> <z> <roll_deg> <pitch_deg> <yaw_deg>

Example (reset to origin with no rotation):
    python reset_pose.py 0 0 0 0 0 0

Example (set position to x=1, y=2, z=-5 and yaw by 90 degrees):
    python reset_pose.py 1 2 -5 0 0 90
"""
import sys
import socket
import struct
import selectors
import time
from enum import Enum
import numpy as np

# ============================================================================
# HELPER FUNCTION (from BlueROVController)
# ============================================================================


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles (in radians) to a quaternion [w, x, y, z].

    Args:
        roll: Rotation around the x-axis, in radians.
        pitch: Rotation around the y-axis, in radians.
        yaw: Rotation around the z-axis, in radians.

    Returns:
        A numpy array representing the quaternion [w, x, y, z].
    """
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

# ============================================================================
# AUV CLASS DEFINITION (from provided code)
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

        self.command_timeouts = {
            MsgHeader.HEARTBEAT: 1.0,
            MsgHeader.GET_MODEL_INFO: 2.0,
            MsgHeader.GET_SENSORDATA: 5.0,
            MsgHeader.GET_RGB_IMAGE: 5.0,
            MsgHeader.GET_MASKED_IMAGE: 5.0,
            MsgHeader.APPLY_CTRL: 10.0,
            MsgHeader.STEP_SIM: 10.0,
            MsgHeader.RESET: 5.0,
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
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket.settimeout(15.0)
            self.socket.connect((self.server_ip, self.server_port))
            print(f"AUV: Connected to MuJoCo server at {
                  self.server_ip}:{self.server_port}")
        except Exception as e:
            print(f"AUV: Connection failed: {e}")
            raise

    def _send_command(self, data: bytes, expected_bytes: int, command_header: MsgHeader) -> bytes:
        if not self.socket:
            raise RuntimeError("Not connected to server")

        timeout = self.command_timeouts.get(command_header, 5.0)

        try:
            cmd_name = command_header.name
            self.socket.sendall(data)

            response = b""
            bytes_received = 0
            start_time = time.time()

            sel = selectors.DefaultSelector()
            sel.register(self.socket, selectors.EVENT_READ)

            try:
                while bytes_received < expected_bytes:
                    elapsed_time = time.time() - start_time
                    timeout_remaining = timeout - elapsed_time

                    if timeout_remaining <= 0:
                        raise TimeoutError(f"Timeout waiting for {cmd_name} response. "
                                           f"Received {bytes_received}/{expected_bytes} bytes")

                    events = sel.select(timeout=min(timeout_remaining, 0.1))

                    if not events:
                        continue

                    remaining = expected_bytes - bytes_received
                    chunk = self.socket.recv(remaining)

                    if not chunk:
                        raise ConnectionError(
                            "Server closed connection during response")

                    response += chunk
                    bytes_received += len(chunk)
            finally:
                sel.close()

            if bytes_received != expected_bytes:
                raise ValueError(f"Incomplete {cmd_name} response: received {
                                 bytes_received}/{expected_bytes} bytes")

            return response

        except Exception as e:
            print(f"AUV: Communication error for {cmd_name}: {e}")
            raise

    def reset(self,
              pose: np.ndarray = None,
              linear_vel: np.ndarray = None,
              angular_vel: np.ndarray = None,
              num_steps: int = 1) -> bool:
        """Reset simulation to specified state."""
        if pose is None:
            pose = np.array([0, 0, 0, 1, 0, 0, 0])
        if linear_vel is None:
            linear_vel = np.array([0, 0, 0])
        if angular_vel is None:
            angular_vel = np.array([0, 0, 0])

        if len(pose) != 7:
            raise ValueError(
                "Pose must have 7 elements [px, py, pz, ow, ox, oy, oz]")

        data = struct.pack('<15f',
                           float(MsgHeader.RESET.value),
                           float(num_steps),
                           *pose,
                           *linear_vel,
                           *angular_vel)

        response = self._send_command(data, 1, MsgHeader.RESET)
        return len(response) == 1 and response[0] == 0

# ============================================================================
# MAIN EXECUTION
# ============================================================================


if __name__ == "__main__":
    # 1. Validate command-line arguments
    if len(sys.argv) != 7:
        print(__doc__)  # Print the docstring as usage instructions
        sys.exit(1)

    try:
        # 2. Parse arguments
        x, y, z = [float(arg) for arg in sys.argv[1:4]]
        roll_deg, pitch_deg, yaw_deg = [float(arg) for arg in sys.argv[4:7]]

        print(f"Attempting to reset sub to:")
        print(f"  Position (x,y,z):      ({x}, {y}, {z})")
        print(f"  Orientation (r,p,y)°:  ({roll_deg}, {pitch_deg}, {yaw_deg})")

        # 3. Convert orientation to quaternion
        #    np.radians converts degrees to radians for the helper function
        roll_rad = np.radians(roll_deg)
        pitch_rad = np.radians(pitch_deg)
        yaw_rad = np.radians(yaw_deg)

        orientation_quat = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        # 4. Construct the final 7-element pose vector [px, py, pz, ow, ox, oy, oz]
        position_vec = np.array([x, y, z])
        pose_to_set = np.concatenate((position_vec, orientation_quat))

        # 5. Connect to AUV and send reset command
        auv_client = AUV()
        success = auv_client.reset(pose=pose_to_set)

        # 6. Report result
        if success:
            print("\n[SUCCESS] Reset command sent successfully to the simulator.")
            print(f"  Final Pose Vector Sent: {
                  np.round(pose_to_set, 4).tolist()}")
        else:
            print("\n[FAILURE] The simulator did not confirm the reset command.")

    except ValueError:
        print("\n[ERROR] Invalid input. All arguments must be numbers.")
        print(__doc__)
        sys.exit(1)
    except (ConnectionRefusedError, TimeoutError, ConnectionError) as e:
        print(f"\n[ERROR] Could not connect to the simulator: {e}")
        print("Please ensure the simulator is running and accessible.")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] An unexpected error occurred: {e}")
        sys.exit(1)
