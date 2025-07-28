#!/usr/bin/python3
"""
IMU to Simulator Orientation Streamer - Stable Connection Version

Key improvements:
1. More conservative timeout handling
2. Better socket lifecycle management
3. Only reconnect on actual connection failures
"""
import socket
import json
import numpy as np
import threading
import time
import argparse
import os
import sys
import configparser
from enum import Enum
import struct
import select
from collections import deque

# --- Helper functions ---
def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    r,p,y=roll/2,pitch/2,yaw/2
    cr,sr=np.cos(r),np.sin(r)
    cp,sp=np.cos(p),np.sin(p)
    cy,sy=np.cos(y),np.sin(y)
    w=cr*cp*cy+sr*sp*sy
    x=sr*cp*cy-cr*sp*sy
    y=cr*sp*cy+sr*cp*sy
    z=cr*cp*sy-sr*sp*cy
    return np.array([w,x,y,z])

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1,x1,y1,z1=q1
    w2,x2,y2,z2=q2
    w=w1*w2-x1*x2-y1*y2-z1*z2
    x=w1*x2+x1*w2+y1*z2-z1*y2
    y=w1*y2-x1*z2+y1*w2+z1*x2
    z=w1*z2+x1*y2-y1*x2+z1*w2
    return np.array([w,x,y,z])

def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    w,x,y,z=q
    return np.array([w,-x,-y,-z])

# =========================================================================
# STABLE AUV CONNECTION CLASS
# =========================================================================
class MsgHeader(Enum):
    RESET = 9

class AUV:
    def __init__(self, server_ip, server_port):
        self.ip = server_ip
        self.port = server_port
        self.socket = None
        self.round_trip_times = deque(maxlen=50)
        self.consecutive_timeouts = 0
        self.total_commands = 0
        self.total_timeouts = 0
        
    def connect(self):
        """Connect to simulator - matching working code pattern"""
        try:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None
                
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Use same timeout as working code
            self.socket.settimeout(15.0)  # 15 second timeout like the working code
            self.socket.connect((self.ip, self.port))
            self.consecutive_timeouts = 0
            print(f"✓ [SIM] Connected to {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ [SIM] Connection failed: {e}")
            return False

    def reset_and_wait(self, pose, num_steps=1) -> tuple:
        """
        Send reset command and wait for ACK.
        Returns (success, round_trip_time_ms)
        """
        if not self.socket:
            raise RuntimeError("Not connected")
        
        if len(pose) != 7:
            raise ValueError("Pose must be 7 elements")
            
        data = struct.pack('<15f', 
                          float(MsgHeader.RESET.value), 
                          float(num_steps),
                          *pose, 
                          *np.zeros(3),  # velocities
                          *np.zeros(3))  # angular velocities
        try:
            t_start = time.perf_counter()
            
            self.socket.sendall(data)
            response = self.socket.recv(1)
            
            t_end = time.perf_counter()
            rtt_ms = (t_end - t_start) * 1000
            
            if len(response) != 1:
                print(f"✗ [SIM] Invalid response: {len(response)} bytes")
                return False, 0
                
            # Check response value
            if response[0] != 0:
                print(f"✗ [SIM] Error response: {response[0]}")
                return False, 0
                
            # Success
            self.round_trip_times.append(rtt_ms)
            self.consecutive_timeouts = 0
            self.total_commands += 1
            return True, rtt_ms
                
        except socket.timeout:
            # Timeout
            self.consecutive_timeouts += 1
            self.total_timeouts += 1
            self.total_commands += 1
            
            if self.consecutive_timeouts >= 3:
                print(f"✗ [SIM] Multiple timeouts ({self.consecutive_timeouts})")
            
            return False, 0
            
        except Exception as e:
            print(f"✗ [SIM] Error: {e}")
            raise e  # Re-raise like the working code
    
    def get_stats(self):
        """Get connection statistics"""
        if self.round_trip_times:
            avg_rtt = sum(self.round_trip_times) / len(self.round_trip_times)
            min_rtt = min(self.round_trip_times)
            max_rtt = max(self.round_trip_times)
        else:
            avg_rtt = min_rtt = max_rtt = 0
            
        timeout_rate = (self.total_timeouts / self.total_commands * 100) if self.total_commands > 0 else 0
        
        return {
            'avg_rtt': avg_rtt,
            'min_rtt': min_rtt,
            'max_rtt': max_rtt,
            'timeout_rate': timeout_rate,
            'total_commands': self.total_commands
        }
    
    def close(self):
        if self.socket:
            try:
                # Just close, don't shutdown - match the working code pattern
                self.socket.close()
            except:
                pass
            self.socket = None

# =========================================================================
# IMU READER
# =========================================================================
class IMUReader:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self.buffer = ""
        self.latest_quaternion = None
        self.latest_timestamp = None
        self.latest_sequence = 0
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.data_timestamps = deque(maxlen=500)
        self.last_stats_time = time.time()
        
    def connect(self):
        """Connect to IMU server"""
        try:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(3.0)
            self.socket.connect((self.host, self.port))
            self.socket.setblocking(False)
            self.connected = True
            self.buffer = ""
            print("✓ [IMU] Connected")
            return True
        except Exception as e:
            self.connected = False
            return False
    
    def run(self):
        """Main IMU reading loop"""
        while self.running:
            if not self.connected:
                if self.connect():
                    continue
                else:
                    time.sleep(3)
                    continue
            
            try:
                readable, _, _ = select.select([self.socket], [], [], 0.01)
                
                if readable:
                    data = self.socket.recv(4096).decode('utf-8')
                    if not data:
                        print("✗ [IMU] Connection lost")
                        self.connected = False
                        continue
                    
                    self.buffer += data
                    
                    while '\n' in self.buffer:
                        line, self.buffer = self.buffer.split('\n', 1)
                        if line.strip():
                            try:
                                json_data = json.loads(line)
                                if 'quaternion' in json_data:
                                    q = np.array(json_data['quaternion'])
                                    timestamp = time.perf_counter()
                                    with self.lock:
                                        self.latest_quaternion = q
                                        self.latest_timestamp = timestamp
                                        self.latest_sequence += 1
                                        self.data_timestamps.append(timestamp)
                            except (json.JSONDecodeError, KeyError):
                                pass
                
                # Print statistics
                if time.time() - self.last_stats_time > 10.0:
                    self._print_stats()
                    self.last_stats_time = time.time()
                    
            except Exception as e:
                if self.running:
                    print(f"✗ [IMU] Error: {e}")
                    self.connected = False
    
    def _print_stats(self):
        """Calculate and print IMU statistics"""
        with self.lock:
            if len(self.data_timestamps) > 10:
                time_span = self.data_timestamps[-1] - self.data_timestamps[0]
                data_rate = (len(self.data_timestamps) - 1) / time_span
                
                intervals = [self.data_timestamps[i] - self.data_timestamps[i-1] 
                            for i in range(1, min(100, len(self.data_timestamps)))]
                if intervals:
                    avg_interval = sum(intervals) / len(intervals)
                    jitter = np.std(intervals) * 1000
                    print(f"[IMU] Rate: {data_rate:.1f} Hz, Interval: {avg_interval*1000:.1f}ms, Jitter: {jitter:.1f}ms")
    
    def get_latest(self):
        """Get latest quaternion, its age, and sequence number"""
        with self.lock:
            if self.latest_timestamp:
                age_ms = (time.perf_counter() - self.latest_timestamp) * 1000
                return self.latest_quaternion, age_ms, self.latest_sequence
            return None, None, None
    
    def send_tare(self):
        """Send TARE command to IMU"""
        if self.connected and self.socket:
            try:
                self.socket.sendall(b"TARE\n")
                return True
            except:
                return False
        return False
    
    def stop(self):
        """Stop the IMU reader"""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass

# =========================================================================
# MAIN STREAMER
# =========================================================================
class IMUToSimStreamer:
    def __init__(self, imu_host, imu_port, sim_host, sim_port):
        self.auv = AUV(sim_host, sim_port)
        self.imu_reader = IMUReader(imu_host, imu_port)
        
        self.running = True
        self.is_first_reading = True
        self.q_tare_offset = np.array([1.0, 0.0, 0.0, 0.0])
        self.q_transform_imu_to_sim = euler_to_quaternion(0, 0, np.pi / 2.0)
        self.last_sent_sequence = -1
        
        self.update_timestamps = deque(maxlen=500)
        self.last_stats_time = time.time()
        self.skipped_old_data = 0
        self.skipped_duplicate = 0

    def _reset_simulation_orientation(self):
        """Reset simulation orientation"""
        q, _, _ = self.imu_reader.get_latest()
        if q is None:
            print("\n[SIM TARE] No IMU data available")
            return
        self.q_tare_offset = quaternion_conjugate(q)
        self.is_first_reading = True
        print("\n[SIM TARE] Simulation orientation zeroed")

    def _reset_real_imu(self):
        """Send TARE command to physical IMU"""
        if self.imu_reader.send_tare():
            print("\n[IMU TARE] Sent TARE command")
            self.is_first_reading = True
        else:
            print("\n[IMU TARE] Failed to send TARE")

    def _print_performance_stats(self):
        """Print detailed performance statistics"""
        if len(self.update_timestamps) > 10:
            time_span = self.update_timestamps[-1] - self.update_timestamps[0]
            actual_rate = (len(self.update_timestamps) - 1) / time_span
            
            intervals = [self.update_timestamps[i] - self.update_timestamps[i-1] 
                        for i in range(1, min(100, len(self.update_timestamps)))]
            if intervals:
                avg_interval = sum(intervals) / len(intervals) * 1000
                interval_std = np.std(intervals) * 1000
            else:
                avg_interval = interval_std = 0
            
            sim_stats = self.auv.get_stats()
            
            print(f"\n========== PERFORMANCE STATS ==========")
            print(f"Update rate: {actual_rate:.1f} Hz (interval: {avg_interval:.1f}±{interval_std:.1f}ms)")
            print(f"Sim RTT: avg={sim_stats['avg_rtt']:.1f}ms, min={sim_stats['min_rtt']:.1f}ms, max={sim_stats['max_rtt']:.1f}ms")
            print(f"Timeout rate: {sim_stats['timeout_rate']:.1f}% ({sim_stats['total_commands']} total commands)")
            print(f"Skipped: {self.skipped_old_data} old, {self.skipped_duplicate} duplicates")
            print(f"=======================================")

    def run(self):
        """Main synchronous loop"""
        print("Starting Stable IMU->Simulator Streamer...")
        print("\n=== CONTROLS ===")
        print("1 + Enter = Reset PHYSICAL IMU")
        print("2 + Enter = Reset SIMULATION")
        print("3 + Enter = Show detailed stats")
        print("Ctrl+C to exit")
        print("================\n")
        
        # Start IMU reader thread
        imu_thread = threading.Thread(target=self.imu_reader.run, daemon=True)
        imu_thread.start()
        
        # Wait for first IMU data
        print("Waiting for IMU data...")
        while self.imu_reader.get_latest()[0] is None and self.running:
            time.sleep(0.1)
        
        # Main loop
        sim_connected = False
        
        while self.running:
            try:
                # Non-blocking input
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    try:
                        line = input()
                        if line == '1':
                            self._reset_real_imu()
                        elif line == '2':
                            self._reset_simulation_orientation()
                        elif line == '3':
                            self._print_performance_stats()
                    except EOFError:
                        self.running = False
                        break
                
                # Ensure simulator connection
                if not sim_connected:
                    if self.auv.connect():
                        sim_connected = True
                        connection_failures = 0
                    else:
                        time.sleep(1)
                        continue
                
                # Get latest IMU data
                q_raw, age_ms, sequence = self.imu_reader.get_latest()
                
                if q_raw is None:
                    continue
                
                # Skip if data is too old
                if age_ms > 30:  # Increased threshold
                    self.skipped_old_data += 1
                    continue
                
                # Only send new data
                if sequence <= self.last_sent_sequence:
                    self.skipped_duplicate += 1
                    time.sleep(0.001)
                    continue
                
                # Handle first reading
                if self.is_first_reading:
                    print("[AUTO-TARE] First reading - zeroing simulation")
                    self.q_tare_offset = quaternion_conjugate(q_raw)
                    self.is_first_reading = False
                
                # Transform quaternion
                q_tared = quaternion_multiply(self.q_tare_offset, q_raw)
                q_sim = quaternion_multiply(self.q_transform_imu_to_sim, q_tared)
                pose_for_sim = np.concatenate(([0, 0, 0], q_sim))
                
                # Send to simulator
                success, rtt_ms = self.auv.reset_and_wait(pose=pose_for_sim, num_steps=1)  # Use 1 like working code
                
                if rtt_ms == -1:  # Connection error
                    connection_failures += 1
                    if connection_failures > 2:
                        print("✗ [SIM] Connection lost, reconnecting...")
                        sim_connected = False
                        self.auv.close()
                        connection_failures = 0
                elif success:
                    connection_failures = 0
                    self.update_timestamps.append(time.perf_counter())
                    self.last_sent_sequence = sequence
                    
                    # Show timing for first few successful commands
                    if len(self.update_timestamps) <= 5:
                        print(f"[TIMING] RTT: {rtt_ms:.1f}ms, Data age: {age_ms:.1f}ms")
                
                # Print stats periodically
                if time.time() - self.last_stats_time > 30.0:
                    self._print_performance_stats()
                    self.last_stats_time = time.time()
                    self.skipped_old_data = 0
                    self.skipped_duplicate = 0
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[ERROR] Main loop: {e}")
                time.sleep(0.1)
        
        self.shutdown()

    def shutdown(self):
        """Clean shutdown"""
        print("\nShutting down...")
        self.running = False
        
        # Final stats
        self._print_performance_stats()
        
        # Stop components
        self.imu_reader.stop()
        self.auv.close()
        
        time.sleep(0.2)
        print("Streamer stopped.")

def main():
    parser = argparse.ArgumentParser(description='Stable IMU to Simulator streamer')
    parser.add_argument('--wifi', action='store_true', help='Use WiFi IP for IMU')
    parser.add_argument('--sim_ip', type=str, default="127.0.0.1", help='Simulator IP')
    parser.add_argument('--sim_port', type=int, default=60001, help='Simulator port')
    args = parser.parse_args()
    
    # Load IMU config
    mode = 'wifi' if args.wifi else 'lan'
    config_path = os.path.expanduser('~/.rov_client_creds')
    config = configparser.ConfigParser()
    
    if not os.path.exists(config_path) or not config.read(config_path):
        sys.exit(f"✗ ERROR: Config file not found: '{config_path}'")
    
    try:
        creds = config[mode]
        imu_host = creds['rov_ip']
        imu_port = config.getint('DEFAULT', 'imu_and_depth_port')
        print(f"✓ [IMU] Using '{mode}' settings: {imu_host}:{imu_port}")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing key in config: {e}")
    
    # Start streamer
    try:
        streamer = IMUToSimStreamer(imu_host, imu_port, args.sim_ip, args.sim_port)
        streamer.run()
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
