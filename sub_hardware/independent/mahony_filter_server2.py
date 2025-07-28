#!/usr/bin/python3
"""
Attitude Server - Runs on Raspberry Pi with sensors
Reads IMU data, performs sensor fusion using a custom Mahony filter with
explicit gyro bias correction, and sends quaternions over the network.
Caches calibration data for convenience.
---
MODIFIED: Includes an online magnetometer calibrator to continuously
refine hard and soft iron offsets, removing the need for a manual
calibration sequence. Dependencies: `numpy`, `scipy`.
---
ADDED: Tare/zero functionality - clients can send "RESET" command to zero the orientation
"""

import numpy as np
import socket
import json
import time
import threading
import os
import argparse
from dataclasses import dataclass
from typing import Tuple, Optional

import configparser
import sys

# Required for online calibration
try:
    from scipy import linalg
except ImportError:
    print("ERROR: SciPy is not installed. Please install it: pip install scipy")
    exit(1)

# Suppress scientific notation for cleaner output
np.set_printoptions(precision=4, suppress=True)

CACHE_FILENAME = "orientation_cache.json"

# --- Quaternion & Vector Math Helpers (Self-Contained) ---


def _normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm > 1e-9 else v


def _q_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])


def _q_conjugate(q): return np.array([q[0], -q[1], -q[2], -q[3]])


def _q_to_dcm(q):
    q0, q1, q2, q3 = q
    return np.array([
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])


def _get_q_from_acc(acc):
    acc_n = _normalize(acc)
    # Simplified initial quaternion from accelerometer only, assuming forward is mostly level
    roll = np.arctan2(acc_n[1], np.sqrt(acc_n[0]**2 + acc_n[2]**2))
    pitch = np.arctan2(-acc_n[0], np.sqrt(acc_n[1]**2 + acc_n[2]**2))
    cy, sy = np.cos(0), np.sin(0)
    cp, sp = np.cos(pitch*0.5), np.sin(pitch*0.5)
    cr, sr = np.cos(roll*0.5), np.sin(roll*0.5)
    return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])


def quaternion_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    q0, q1, q2, q3 = q
    roll = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2))
    sinp = 2*(q0*q2-q3*q1)
    pitch = np.arcsin(np.clip(sinp, -1, 1))
    yaw = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))
    return roll, pitch, yaw


@dataclass
class IMUData:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float
    timestamp: float


class MagnetometerCalibrator:
    """
    Performs online magnetometer calibration by fitting an ellipsoid to collected data.
    """

    def __init__(self, cache_path=CACHE_FILENAME, collection_size=500, fit_interval=60.0):
        self.cache_path = cache_path
        self.collection_size = collection_size
        self.fit_interval = fit_interval
        self.running = False

        self.mag_points = []
        self.hard_iron = np.zeros(3)  # Offset
        self.soft_iron = np.eye(3)    # Scale/Distortion matrix

        self.data_lock = threading.Lock()
        self._load_from_cache()

    def add_point(self, mag_raw: np.ndarray):
        """Adds a new magnetometer reading to the collection."""
        if len(self.mag_points) < self.collection_size:
            self.mag_points.append(mag_raw)
        else:
            # Replace a random old point to keep the collection fresh
            idx = np.random.randint(0, self.collection_size)
            self.mag_points[idx] = mag_raw

    def correct(self, mag_raw: np.ndarray) -> np.ndarray:
        """Applies the current calibration to a raw magnetometer vector."""
        return self.soft_iron @ (mag_raw - self.hard_iron)

    def _fit_ellipsoid(self):
        """
        Calculates hard and soft iron parameters from the collected points.
        This is the core of the online calibration.
        """
        with self.data_lock:
            if len(self.mag_points) < 100:  # Need enough points for a stable fit
                return

            points = np.array(self.mag_points)
            # Ellipsoid fitting Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz - 1 = 0
            D = np.c_[points**2, 2*points[:, 0]*points[:, 1], 2*points[:, 0]
                      * points[:, 2], 2*points[:, 1]*points[:, 2], 2*points]
            v = np.ones(D.shape[0])

            try:
                # --- Solve for the ellipsoid parameters ---
                u = linalg.lstsq(D, v, rcond=None)[0]

                # --- Extract matrix form ---
                Q = np.array([
                    [u[0], u[3], u[4]],
                    [u[3], u[1], u[5]],
                    [u[4], u[5], u[2]]
                ])
                b = np.array([u[6], u[7], u[8]])

                # --- Solve for hard-iron (offset) and soft-iron (transformation) ---
                offset = -linalg.solve(Q, b)
                T = linalg.cholesky(Q)
                scale = T.T @ T

                # Check for non-invertible matrix
                if np.linalg.det(scale) < 1e-9:
                    print("WARN: Unstable magnetometer fit, skipping update.")
                    return

                self.hard_iron = offset
                self.soft_iron = scale
                # <<< FIX #1: This line was combined into one to fix the SyntaxError.
                print(f"\n[MagCal] New calibration applied. Offset: {self.hard_iron}, Fit Score: {np.mean(linalg.norm(self.correct(points), axis=1)):.2f}")
                self._save_to_cache()

            except linalg.LinAlgError:
                print(
                    "\n[MagCal] Linear algebra error during fit. Skipping update.")

    def calibration_loop(self):
        """Periodically runs the fitting algorithm."""
        self.running = True
        print("[MagCal] Online magnetometer calibration running.")
        while self.running:
            time.sleep(self.fit_interval)
            self._fit_ellipsoid()

    def _load_from_cache(self):
        try:
            if os.path.exists(self.cache_path):
                with open(self.cache_path, 'r') as f:
                    data = json.load(f)
                if 'hard_iron' in data and 'soft_iron' in data:
                    self.hard_iron = np.array(data['hard_iron'])
                    self.soft_iron = np.array(data['soft_iron'])
                    print(f"[MagCal] Loaded magnetometer calibration from {self.cache_path}")
        except Exception as e:
            print(f"WARN: Could not load mag cal from cache: {e}. Using defaults.")

    def _save_to_cache(self):
        # This function might be called from a different thread than the server's cache saver
        # A simple lock or atomic write is a good idea for more complex scenarios
        try:
            # Read existing data to not overwrite q0
            if os.path.exists(self.cache_path):
                with open(self.cache_path, 'r') as f:
                    data = json.load(f)
            else:
                data = {}

            data['hard_iron'] = self.hard_iron.tolist()
            data['soft_iron'] = self.soft_iron.tolist()
            with open(self.cache_path, 'w') as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            print(f"ERROR: Could not save mag cal to cache: {e}")

    def stop(self):
        self.running = False
        print("[MagCal] Saving final calibration...")
        self._fit_ellipsoid()  # One last fit before shutdown


class AttitudeFilter:
    def __init__(self, frequency=100.0, kp=0.5, ki=0.1):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)
        self.kp = kp
        self.ki = ki
        self.dt = 1.0 / frequency
        self.gravity_ref = np.array([0.0, 0.0, 1.0])

    def update(self, gyr: np.ndarray, acc: np.ndarray, mag: np.ndarray) -> np.ndarray:
        acc_n, mag_n = _normalize(acc), _normalize(mag)

        # Get the current orientation as a rotation matrix (sensor-to-world)
        dcm = _q_to_dcm(self.q)

        # --- Accelerometer Error ---
        # The reference vector for gravity is [0, 0, 1] in the world frame.
        # Rotate it to the sensor frame to see where it *should* be according to our current orientation.
        v_acc = dcm.T @ self.gravity_ref
        # The error is the cross product between the measured and the expected gravity vector.
        error_acc = np.cross(acc_n, v_acc)

        # --- Robust Magnetometer Error Calculation (FIXED) ---
        # The previous method created a physically incorrect horizontal reference, causing yaw drift.
        # This new method correctly handles magnetic inclination (dip).

        # 1. Rotate the measured magnetometer vector into the current estimate of the world frame.
        mag_world_est = dcm @ mag_n

        # 2. Calculate a robust reference magnetic vector in the world frame.
        #    This reference vector has a horizontal component and a vertical component,
        #    modeling the true magnetic field without assuming it's flat.
        #    We define it in a reference frame where North lies on the Y-Z plane.
        b_x = np.sqrt(mag_world_est[0]**2 + mag_world_est[1]**2)
        b_z = mag_world_est[2]
        mag_ref_world = np.array([0.0, b_x, b_z])

        # 3. Rotate this ideal world reference vector back into the sensor's frame.
        mag_ref_sensor_frame = dcm.T @ mag_ref_world

        # 4. Calculate magnetometer-based error (corrects yaw).
        error_mag = np.cross(mag_n, mag_ref_sensor_frame)

        # --- Combine errors and update the filter ---
        # Total error is the sum of the accelerometer and magnetometer corrections.
        error = error_acc + error_mag

        # Update gyro bias based on the total error
        self.bias += -self.ki * error * self.dt

        # Correct the gyroscope reading with the P and I terms
        omega = gyr - self.bias + self.kp * error

        # Update quaternion with the corrected angular velocity
        q_dot = 0.5 * _q_multiply(self.q, np.array([0.0, *omega]))
        self.q = _normalize(self.q + q_dot * self.dt)
        return self.q

    def set_initial_state(self, q0: np.ndarray):
        self.q = np.copy(q0)
        print(f"Filter initialized. Q: {self.q}")


class AttitudeServer:
    def __init__(self, host='0.0.0.0', port=9999, kp=0.5, ki=0.1, recalibrate=False):
        self.host, self.port = host, port
        self.filter = AttitudeFilter(kp=kp, ki=ki)
        self.mag_calibrator = MagnetometerCalibrator()
        self.recalibrate = recalibrate

        # Tare/Zero functionality
        self.reference_quaternion = np.array(
            [1.0, 0.0, 0.0, 0.0])  # Identity quaternion
        self.reference_lock = threading.Lock()

        self.running = False
        self.clients = []
        self.client_lock = threading.Lock()
        self.latest_icm = None
        self.latest_mmc = None
        self.data_lock = threading.Lock()

    def handle_client_commands(self, client_socket):
        """Handle incoming commands from a client."""
        client_socket.settimeout(0.1)  # Non-blocking with timeout
        buffer = ""

        while self.running:
            try:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break

                buffer += data
                # Process complete messages (ending with newline)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    if line == "RESET" or line == "TARE" or line == "ZERO":
                        # Set current orientation as reference
                        with self.reference_lock:
                            self.reference_quaternion = np.copy(self.filter.q)
                        print(
                            f"\n[TARE] Reference orientation reset. Current orientation is now zero.")

                        # Send acknowledgment
                        try:
                            ack_msg = json.dumps(
                                {"status": "RESET_OK", "timestamp": time.time()}).encode() + b'\n'
                            client_socket.sendall(ack_msg)
                        except:
                            pass

            except socket.timeout:
                continue
            except Exception:
                break

    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(5)
            s.settimeout(1.0)
            print(f"Attitude Server listening on {self.host}:{self.port}")
            print("Clients can send 'RESET' command to zero the orientation")

            while self.running:
                try:
                    c, addr = s.accept()
                    print(f"\nClient connected from {addr}")
                    with self.client_lock:
                        self.clients.append(c)

                    # Start command handler thread for this client
                    cmd_thread = threading.Thread(
                        target=self.handle_client_commands, args=(c,), daemon=True)
                    cmd_thread.start()

                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"Server error: {e}")

    def compute_relative_quaternion(self, q_current):
        """Compute quaternion relative to reference: q_relative = q_reference^(-1) * q_current"""
        with self.reference_lock:
            q_ref_inv = _q_conjugate(self.reference_quaternion)
            return _q_multiply(q_ref_inv, q_current)

    def broadcast_data(self, data):
        if not self.clients:
            return
        msg = json.dumps(data).encode() + b'\n'
        with self.client_lock:
            disconnected = [c for c in self.clients if c.fileno() == -1]
            for c in self.clients:
                try:
                    c.sendall(msg)
                except (ConnectionResetError, BrokenPipeError, OSError):
                    disconnected.append(c)
            for c in list(set(disconnected)):
                try:
                    print(f"\nClient {c.getpeername()} disconnected.")
                except OSError:
                    print("\nClient with closed socket disconnected.")
                self.clients.remove(c)
                c.close()

    def read_icm20602(self):
        try:
            from icm20602 import ICM20602
            icm = ICM20602()
            print("OK: ICM20602")
        except Exception as e:
            print(f"WARN: ICM20602: {e}. Using dummy data.")
            icm = None
        while self.running:
            try:
                if icm:
                    d = icm.read_all()
                    ax, ay, az, gx, gy, gz = d.a.x, d.a.y, d.a.z, d.g.x, d.g.y, d.g.z
                else:
                    ax, ay, az, gx, gy, gz = 0, 0, 9.81, 0.01, -0.02, 0.03
                with self.data_lock:
                    self.latest_icm = {'ax': ax, 'ay': ay, 'az': az, 'gx': np.radians(
                        gx), 'gy': np.radians(gy), 'gz': np.radians(gz), 'timestamp': time.time()}
                time.sleep(0.01)
            except Exception as e:
                print(f"\nICM20602 error: {e}")
                time.sleep(1)

    def read_mmc5983(self):
        try:
            from mmc5983 import MMC5983
            mmc = MMC5983()
            print("OK: MMC5983")
        except Exception as e:
            print(f"WARN: MMC5983: {e}. Using dummy data.")
            mmc = None
        while self.running:
            try:
                if mmc:
                    d = mmc.read_data()
                    mx, my, mz = d.x, d.y, d.z
                else:
                    mx, my, mz = 25., 5., -30.
                mag_raw = np.array([mx, my, mz])
                # Add raw point to calibrator
                self.mag_calibrator.add_point(mag_raw)
                with self.data_lock:
                    self.latest_mmc = {'mx': mx, 'my': my,
                                       'mz': mz, 'timestamp': time.time()}
                time.sleep(0.02)
            except Exception as e:
                print(f"\nMMC5983 error: {e}")
                time.sleep(1)

    def _initialize_filter_state(self):
        """Initializes the filter, either from cache or a quick sensor read."""
        q0 = None
        if not self.recalibrate:
            try:
                if os.path.exists(CACHE_FILENAME):
                    with open(CACHE_FILENAME, 'r') as f:
                        data = json.load(f)
                    if 'q0' in data and len(data['q0']) == 4:
                        q0 = np.array(data['q0'])
                        print(f"Loaded initial quaternion from {CACHE_FILENAME}")
                    else:
                        print("WARNING: Invalid q0 in cache.")
            except Exception as e:
                print(f"WARNING: Could not load q0 from cache ({e}).")

        if q0 is None:
            print(
                "No valid initial orientation in cache. Place device level to initialize.")
            print("Initializing in 3 seconds...")
            time.sleep(3.0)

            acc_readings = []
            end_time = time.time() + 2.0
            while time.time() < end_time:
                with self.data_lock:
                    if self.latest_icm:
                        acc_readings.append(
                            [self.latest_icm['ax'], self.latest_icm['ay'], self.latest_icm['az']])
                time.sleep(0.1)

            if not acc_readings:
                print("ERROR: No accelerometer data. Using default orientation.")
                q0 = np.array([1.0, 0.0, 0.0, 0.0])
            else:
                acc_avg = np.mean(acc_readings, axis=0)
                q0 = _get_q_from_acc(acc_avg)
                print(
                    f"Initialized with new orientation from accelerometer. q0: {q0}")

            try:
                # Save the new q0 to cache
                if os.path.exists(CACHE_FILENAME):
                    with open(CACHE_FILENAME, 'r') as f:
                        data = json.load(f)
                else:
                    data = {}
                data['q0'] = q0.tolist()
                with open(CACHE_FILENAME, 'w') as f:
                    json.dump(data, f, indent=4)
            except Exception as e:
                print(f"ERROR: Could not save initial q0 to cache: {e}")

        self.filter.set_initial_state(q0)
        # Initialize reference quaternion to current orientation
        with self.reference_lock:
            self.reference_quaternion = np.copy(q0)

    def filter_loop(self):
        print("Filter loop waiting for sensor data...")
        time.sleep(1)

        self._initialize_filter_state()

        print("\nInitialization complete. Starting attitude estimation.")

        while self.running:
            with self.data_lock:
                icm = self.latest_icm
                mmc = self.latest_mmc
            if icm and mmc:
                acc_raw = np.array([icm['ax'], icm['ay'], icm['az']])
                gyr_raw = np.array([icm['gx'], icm['gy'], icm['gz']])
                mag_raw = np.array([mmc['mx'], mmc['my'], mmc['mz']])

                # Get corrected magnetometer reading
                mag_corrected = self.mag_calibrator.correct(mag_raw)

                # Update filter with absolute orientation
                q_absolute = self.filter.update(
                    gyr_raw, acc_raw, mag_corrected)

                # Compute relative orientation for output
                q_relative = self.compute_relative_quaternion(q_absolute)
                r, p, y = quaternion_to_euler(q_relative)

                self.broadcast_data({
                    'timestamp': time.time(),
                    'quaternion': q_relative.tolist(),  # Send relative quaternion
                    # Relative euler angles
                    'euler': {'roll': r, 'pitch': p, 'yaw': y}
                })
                # <<< FIX #2: This line was combined into one to fix the SyntaxError.
                print(f"RPY: [{np.degrees(r):6.1f}, {np.degrees(p):6.1f}, {np.degrees(y):6.1f}]° | Mag Fit: {len(self.mag_calibrator.mag_points)}/{self.mag_calibrator.collection_size} pts   ", end='\r')

            time.sleep(self.filter.dt)

    def run(self):
        self.running = True
        threads = [
            threading.Thread(target=self.start_server, daemon=True),
            threading.Thread(target=self.read_icm20602, daemon=True),
            threading.Thread(target=self.read_mmc5983, daemon=True),
            threading.Thread(
                target=self.mag_calibrator.calibration_loop, daemon=True),
            threading.Thread(target=self.filter_loop, daemon=True)
        ]
        for t in threads:
            t.start()
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down server...")
        finally:
            self.running = False
            self.mag_calibrator.stop()  # Save final calibration
            time.sleep(0.5)
            with self.client_lock:
                for c in self.clients:
                    c.close()


def main():
    parser = argparse.ArgumentParser(
        description='Attitude Server with Mahony filter, Online Calibration, and Tare functionality')
    # Arguments for core logic are kept
    parser.add_argument('--kp', type=float, default=17.0,
                        help='Proportional gain (Kp) for attitude correction. NOTE: Lower Kp is better for online mag calibration.')
    parser.add_argument('--ki', type=float, default=0.2,
                        help='Integral gain (Ki) for gyro drift correction.')
    parser.add_argument('-c', '--recalibrate', action='store_true',
                        help='Force a new initial orientation reading, ignoring cached value.')
    # Argument for network mode
    parser.add_argument('--wifi', action='store_true', help='Use WiFi IP/network settings instead of LAN.')
    args = parser.parse_args()

    # --- Load Credentials ---
    mode = 'wifi' if args.wifi else 'lan'
    config_path = os.path.expanduser('~/.rov_server_creds')
    
    config = configparser.ConfigParser()
    if not os.path.exists(config_path) or not config.read(config_path):
        sys.exit(f"✗ ERROR: Config file not found or is empty at '{config_path}'")
    
    try:
        creds = config[mode]
        host = creds['rov_ip']
        port = config.getint('DEFAULT', 'imu_and_depth_port')
        print(f"✓ Loaded '{mode}' settings from '{config_path}'")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing section or key in config file: {e}")
    # --- End Load Credentials ---

    print(f"Starting server on {host}:{port} with Kp={args.kp}, Ki={args.ki}")
    server = AttitudeServer(host=host, port=port,
                            kp=args.kp, ki=args.ki, recalibrate=args.recalibrate)
    server.run()


if __name__ == '__main__':
    main()
