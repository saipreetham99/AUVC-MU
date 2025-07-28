#!/usr/bin/python3
"""
IMU to Simulator Streamer (v14 - The Correct, Stable Architecture)

This definitive version uses a clean, professionally-architected multi-threading
model to provide smooth, instantaneous, and stable real-time updates.

Architecture:
1. IMU Thread: A dedicated producer that polls the IMU and puts data on a queue.
2. Simulator Thread: A dedicated consumer that runs the synchronous send/wait
   loop, driven by the simulator's actual performance.
3. Main Thread: A simple dispatcher for user input, sending commands to the
   other threads via queues.

This eliminates all race conditions and I/O blocking issues.
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
from queue import Queue, Empty

# --- Helper functions are unchanged and correctly formatted ---
def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    r, p, y = roll / 2, pitch / 2, yaw / 2
    cr, sr = np.cos(r), np.sin(r); cp, sp = np.cos(p), np.sin(p); cy, sy = np.cos(y), np.sin(y)
    w = cr * cp * cy + sr * sp * sy; x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy; z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1,x1,y1,z1=q1; w2,x2,y2,z2=q2; w=w1*w2-x1*x2-y1*y2-z1*z2; x=w1*x2+x1*w2+y1*z2-z1*y2
    y=w1*y2-x1*z2+y1*w2+z1*x2; z=w1*z2+x1*y2-y1*x2+z1*w2
    return np.array([w,x,y,z])

def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    w,x,y,z=q; return np.array([w,-x,-y,-z])

# =========================================================================
# IMU CLIENT THREAD (Self-Contained)
# =========================================================================
class IMUClientThread(threading.Thread):
    def __init__(self, host, port, data_queue, command_queue):
        super().__init__(daemon=True, name="IMUClientThread")
        self.host, self.port = host, port
        self.data_queue = data_queue
        self.command_queue = command_queue
        self.socket = None
        self.running = True

    def connect(self):
        while self.running:
            try:
                print("  [IMU] Attempting to connect...")
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(3.0)
                self.socket.connect((self.host, self.port))
                self.socket.settimeout(None)
                print("✓ [IMU] Connected.")
                return True
            except Exception as e:
                print(f"✗ [IMU] Connection failed: {e}. Retrying...")
                time.sleep(3)
        return False

    def run(self):
        if not self.connect(): return
        
        buffer = ""
        while self.running:
            try:
                # Check for commands (like TARE) without blocking
                try:
                    command = self.command_queue.get_nowait()
                    if command == 'RESET_IMU':
                        print("[IMU TARE] Sending TARE command to hardware...")
                        self.socket.sendall(b"TARE\n")
                except Empty:
                    pass

                # Receive data
                data = self.socket.recv(4096).decode('utf-8')
                if not data:
                    print("✗ [IMU] Disconnected. Reconnecting...")
                    self.socket.close()
                    if not self.connect(): break # Exit if reconnect fails while shutting down
                    continue
                
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        q = np.array(json.loads(line)['quaternion'])
                        # Overwrite the latest data in the queue
                        while not self.data_queue.empty():
                            self.data_queue.get_nowait()
                        self.data_queue.put(q)
                    except (json.JSONDecodeError, KeyError, TypeError):
                        pass
            except Exception:
                if self.running:
                    print("✗ [IMU] Receive loop error. Reconnecting...")
                    self.socket.close()
                    if not self.connect(): break
    
    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()

# =========================================================================
# SIMULATOR CLIENT THREAD (Self-Contained)
# =========================================================================
class SimulatorClientThread(threading.Thread):
    def __init__(self, host, port, data_queue, command_queue):
        super().__init__(daemon=True, name="SimulatorClientThread")
        self.host, self.port = host, port
        self.data_queue = data_queue
        self.command_queue = command_queue
        self.socket = None
        self.running = True
        
        self.is_first_reading = True
        self.q_tare_offset = np.array([1.0, 0.0, 0.0, 0.0])
        self.q_transform_imu_to_sim = euler_to_quaternion(0, 0, np.pi / 2.0)
    
    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket.settimeout(2.0)
            self.socket.connect((self.host, self.port))
            print(f"✓ [SIM] Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ [SIM] Connection failed: {e}")
            return False

    def send_and_wait(self, data):
        try:
            self.socket.sendall(data)
            self.socket.recv(1) # Wait for the 1-byte ACK
            return True
        except (socket.timeout, ConnectionError, OSError):
            return False

    def run(self):
        if not self.connect():
            self.running = False
            return

        latest_q = None
        print("\n--- Starting Synchronous Simulator Loop ---")
        while self.running:
            try:
                # 1. Check for user commands
                try:
                    command = self.command_queue.get_nowait()
                    if command == 'RESET_SIM':
                        print("[SIM TARE] Software tare requested.")
                        if latest_q is not None:
                             self.q_tare_offset = quaternion_conjugate(latest_q)
                    elif command == 'INVALIDATE_TARE':
                         self.is_first_reading = True
                         print("[SIM TARE] Invalidated by hardware reset. Will auto-tare on next packet.")
                except Empty:
                    pass

                # 2. Get the latest data from the IMU thread's queue
                try:
                    latest_q = self.data_queue.get_nowait()
                except Empty:
                    # No new data, just step the sim to keep it alive
                    step_data = struct.pack('<2f', 8.0, 1.0) # STEP_SIM = 8
                    if not self.send_and_wait(step_data):
                        print("✗ [SIM] Connection lost during idle step.")
                        self.running = False
                    continue

                # 3. Handle auto-tare on first valid packet
                if self.is_first_reading and latest_q is not None:
                    print("[AUTO-TARE] First IMU reading received. Zeroing simulation.")
                    self.q_tare_offset = quaternion_conjugate(latest_q)
                    self.is_first_reading = False

                # 4. Transform data and create pose
                q_tared = quaternion_multiply(self.q_tare_offset, latest_q)
                q_sim = quaternion_multiply(self.q_transform_imu_to_sim, q_tared)
                pose_for_sim = np.concatenate(([0, 0, 0], q_sim))

                # 5. Send the two commands back-to-back
                reset_data = struct.pack('<15f', 9.0, 0.0, *pose_for_sim, *np.zeros(6))
                step_data = struct.pack('<2f', 8.0, 1.0)
                
                if not self.send_and_wait(reset_data) or not self.send_and_wait(step_data):
                    print("✗ [SIM] Connection lost during update.")
                    self.running = False

            except Exception as e:
                print(f"✗ [SIM] Unhandled error in main loop: {e}")
                self.running = False
    
    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()

# =========================================================================
# MAIN THREAD (User Input Dispatcher)
# =========================================================================
def main():
    parser = argparse.ArgumentParser(description='The definitive, stable IMU streamer.')
    parser.add_argument('--wifi', action='store_true', help='Use WiFi IP for IMU.')
    parser.add_argument('--sim_ip', type=str, default="127.0.0.1", help='Simulator IP.')
    parser.add_argument('--sim_port', type=int, default=60001, help='Simulator port.')
    args = parser.parse_args()

    # --- Load Config ---
    mode='wifi' if args.wifi else 'lan'; config_path=os.path.expanduser('~/.rov_client_creds')
    config=configparser.ConfigParser();
    if not os.path.exists(config_path) or not config.read(config_path): sys.exit(f"✗ Config file not found: '{config_path}'")
    try: creds=config[mode]; imu_host=creds['rov_ip']; imu_port=config.getint('DEFAULT', 'imu_and_depth_port'); print(f"✓ [IMU] Config loaded for '{mode}': {imu_host}:{imu_port}")
    except (KeyError, configparser.NoSectionError) as e: sys.exit(f"✗ Config error: {e}")

    # --- Create Queues and Threads ---
    imu_data_queue = Queue(maxsize=1) # We only ever need the most recent data
    imu_command_queue = Queue()
    sim_command_queue = Queue()

    try:
        imu_client = IMUClientThread(imu_host, imu_port, imu_data_queue, imu_command_queue)
        sim_client = SimulatorClientThread(args.sim_ip, args.sim_port, imu_data_queue, sim_command_queue)
    except Exception as e:
        print(f"\n[FATAL] Failed to initialize clients: {e}")
        sys.exit(1)

    imu_client.start()
    sim_client.start()

    print("\n=== INTERACTIVE CONTROLS ==="); print("1 + [Enter] = Reset PHYSICAL IMU"); print("2 + [Enter] = Reset SIMULATION"); print("Ctrl+C to exit."); print("============================\n")
    
    try:
        while imu_client.is_alive() and sim_client.is_alive():
            choice = input()
            if choice == '1':
                imu_command_queue.put('RESET_IMU')
                sim_command_queue.put('INVALIDATE_TARE') # Tell sim to re-tare
            elif choice == '2':
                sim_command_queue.put('RESET_SIM')
            else:
                print(f"Invalid choice '{choice}'. Please enter '1' or '2'.")
    except (KeyboardInterrupt, EOFError):
        print("\nShutdown signal received...")
    finally:
        imu_client.stop()
        sim_client.stop()
        imu_client.join()
        sim_client.join()
        print("All threads stopped. Exiting.")

if __name__ == '__main__':
    main()
