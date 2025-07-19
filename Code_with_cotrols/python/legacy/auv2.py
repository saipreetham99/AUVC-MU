import socket
import struct
import time
import numpy as np
import selectors
from typing import Optional, Tuple
from enum import Enum

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
            MsgHeader.HEARTBEAT: 1.0,        # Quick response expected
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
            print(f"AUV: Sending {cmd_name} command, expecting {expected_bytes} bytes")
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
                        
                        print(f"AUV: Received {len(chunk)} bytes, total: {bytes_received}/{expected_bytes}")
                        
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
            
            print(f"AUV: {cmd_name} command completed successfully")
            return response
            
        except Exception as e:
            print(f"AUV: Communication error for {cmd_name}: {e}")
            raise
    
    def heartbeat(self) -> bool:
        """Send heartbeat to verify connection."""
        data = struct.pack('<f', float(MsgHeader.HEARTBEAT.value))
        response = self._send_command(data, 1, MsgHeader.HEARTBEAT)
        return len(response) == 1 and response[0] == 0
    
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

        print(f"AUV: Applying forces {forces} for {num_steps} steps")

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
        print(f"AUV: Stepping simulation {num_steps} steps")
        
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
        
        print(f"AUV: Resetting simulation with {num_steps} initial steps")
        
        # Pack: [command, numSteps, pose(7), linear_vel(3), angular_vel(3)] = 15 floats
        data = struct.pack('<15f', 
                          float(MsgHeader.RESET.value),  # RESET command
                          float(num_steps),
                          *pose,
                          *linear_vel,
                          *angular_vel)
        
        response = self._send_command(data, 1, MsgHeader.RESET)
        return len(response) == 1 and response[0] == 0
    
    def get_rgb_image(self) -> Optional[bytes]:
        """Get RGB image from simulation (if implemented)."""
        data = struct.pack('<f', float(MsgHeader.GET_RGB_IMAGE.value))
        response = self._send_command(data, 1, MsgHeader.GET_RGB_IMAGE)
        
        if len(response) == 1 and response[0] == 1:
            print("AUV: RGB image not implemented on server")
            return None
        return response if len(response) > 1 else None
    
    def get_masked_image(self) -> Optional[bytes]:
        """Get masked image from simulation (if implemented)."""
        data = struct.pack('<f', float(MsgHeader.GET_MASKED_IMAGE.value))
        response = self._send_command(data, 1, MsgHeader.GET_MASKED_IMAGE)
        
        if len(response) == 1 and response[0] == 1:
            print("AUV: Masked image not implemented on server")
            return None
        return response if len(response) > 1 else None

# Example usage:
if __name__ == "__main__":
    auv = AUV("127.0.0.1", 60001)
    
    try:
        # Test connection
        if auv.heartbeat():
            print("✓ Heartbeat successful")
        
        # Get model info
        model_info = auv.get_model_info()
        if model_info:
            print(f"✓ Model info: {model_info}")
        
        # # Reset to initial state
        # initial_pose = np.array([0, 0, 0, 1, 0, 0, 0])
        # if auv.reset(pose=initial_pose, num_steps=10):
        #     print("✓ Reset successful")
        
        # Apply custom forces
        # forces = np.random.rand(1,6).*np.array([-10, -10, 10, 10, 0, 0])  # Forward thrust
        for _ in range(100):
            forces = np.random.uniform(-30.0, 30.0, size=6)
            floats = auv.apply_ctrl(forces, num_steps=2)
            print("✓ Applied control forces")
            print("Sensors Data", floats)
        
        # Get sensor data
        sensor_data = auv.get_sensor_data()
        if sensor_data:
            print(f"✓ Sensor data: IMU={sensor_data['imu_quaternion']}, time={sensor_data['time']}")

        # Step simulation
        if auv.step_sim(num_steps=10):
            print("✓ Simulation stepped")
            
    except Exception as e:
        print(f"Error: {e}")

