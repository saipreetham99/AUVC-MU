# File: imu.py
# This is a refactored version of the provided mahony_filter_server.py
# It removes all networking and focuses on sensor fusion.
import numpy as np
import json
import time
import threading
import os
import logging
from dataclasses import dataclass
from typing import Optional

try:
    from scipy import linalg
except ImportError:
    logging.critical("SciPy is not installed (pip install scipy). IMU online calibration disabled.")
    linalg = None

# --- Helper Functions (copied from original) ---
def _normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm > 1e-9 else v

def _q_multiply(q1, q2):
    w1, x1, y1, z1 = q1; w2, x2, y2, z2 = q2
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])

def _q_conjugate(q): return np.array([q[0], -q[1], -q[2], -q[3]])

def _q_to_dcm(q):
    q0,q1,q2,q3=q; return np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],[2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],[2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])

def _get_q_from_acc(acc):
    acc_n = _normalize(acc); roll = np.arctan2(acc_n[1], np.sqrt(acc_n[0]**2 + acc_n[2]**2)); pitch = np.arctan2(-acc_n[0], np.sqrt(acc_n[1]**2 + acc_n[2]**2)); cy, sy = np.cos(0), np.sin(0); cp, sp = np.cos(pitch*0.5), np.sin(pitch*0.5); cr, sr = np.cos(roll*0.5), np.sin(roll*0.5); return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])

def _quaternion_to_euler(q):
    q0, q1, q2, q3 = q; roll = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2)); sinp = 2*(q0*q2-q3*q1); pitch = np.arcsin(np.clip(sinp, -1, 1)); yaw = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2)); return (np.degrees(roll), np.degrees(pitch), np.degrees(yaw))

# --- Mock Sensor Classes for Testing ---
class MockICM:
    def read_all(self):
        @dataclass
        class Accel: x: float; y: float; z: float
        @dataclass
        class Gyro: x: float; y: float; z: float
        @dataclass
        class MockData: a: Accel; g: Gyro
        return MockData(a=Accel(x=0, y=0, z=9.81), g=Gyro(x=np.degrees(0.01), y=np.degrees(-0.02), z=np.degrees(0.03)))

class MockMMC:
    def read_data(self):
        @dataclass
        class MockMagData: x: float; y: float; z: float
        return MockMagData(x=25.0, y=5.0, z=-30.0)

# --- Main IMU System Components ---
class MagnetometerCalibrator:
    def __init__(self, cache_path, collection_size=500, fit_interval=60.0):
        if not linalg: logging.warning("SciPy not found, MagnetometerCalibrator will not perform fitting.")
        self.cache_path = cache_path; self.collection_size = collection_size; self.fit_interval = fit_interval
        self.running = False; self.mag_points = []; self.hard_iron = np.zeros(3); self.soft_iron = np.eye(3)
        self.data_lock = threading.Lock(); self._load_from_cache()

    def add_point(self, mag_raw):
        with self.data_lock:
            if len(self.mag_points) < self.collection_size: self.mag_points.append(mag_raw)
            else: self.mag_points[np.random.randint(0, self.collection_size)] = mag_raw

    def correct(self, mag_raw): return self.soft_iron @ (mag_raw - self.hard_iron)

    def _fit_ellipsoid(self):
        if not linalg: return
        with self.data_lock:
            if len(self.mag_points) < 100: return
            points = np.array(self.mag_points)
            D = np.c_[points**2, 2*points[:,0]*points[:,1], 2*points[:,0]*points[:,2], 2*points[:,1]*points[:,2], 2*points]; v = np.ones(D.shape[0])
            try:
                # u = linalg.lstsq(D, v, rcond=None)[0] # This line does not work
                u = linalg.lstsq(D, v)[0]
                Q = np.array([[u[0],u[3],u[4]], [u[3],u[1],u[5]], [u[4],u[5],u[2]]]); b = np.array([u[6],u[7],u[8]])
                offset = -linalg.solve(Q, b); T = linalg.cholesky(Q); scale = T.T @ T
                if np.linalg.det(scale) < 1e-9: logging.warning("[MagCal] Unstable fit, skipping update."); return
                self.hard_iron = offset; self.soft_iron = scale
                logging.info(f"[MagCal] New calibration applied. Offset: {self.hard_iron}")
                self._save_to_cache()
            except linalg.LinAlgError: logging.warning("[MagCal] LinAlgError during fit. Skipping update.")

    def calibration_loop(self):
        self.running = True
        logging.info("[MagCal] Online magnetometer calibration running.")
        while self.running: time.sleep(self.fit_interval); self._fit_ellipsoid()

    def _load_from_cache(self):
        try:
            if os.path.exists(self.cache_path):
                with open(self.cache_path, 'r') as f: data = json.load(f)
                if 'hard_iron' in data and 'soft_iron' in data:
                    self.hard_iron = np.array(data['hard_iron']); self.soft_iron = np.array(data['soft_iron'])
                    logging.info(f"[MagCal] Loaded magnetometer calibration from {self.cache_path}")
        except Exception as e: logging.warning(f"Could not load mag cal from cache: {e}")

    def _save_to_cache(self):
        try:
            data = {}
            if os.path.exists(self.cache_path):
                with open(self.cache_path, 'r') as f: data = json.load(f)
            data['hard_iron'] = self.hard_iron.tolist(); data['soft_iron'] = self.soft_iron.tolist()
            with open(self.cache_path, 'w') as f: json.dump(data, f, indent=4)
        except Exception as e: logging.error(f"Could not save mag cal to cache: {e}")

    def stop(self): self.running = False; logging.info("[MagCal] Saving final calibration..."); self._fit_ellipsoid()

class AttitudeFilter:
    def __init__(self, frequency=100.0, kp=17.0, ki=0.2):
        self.q = np.array([1.0, 0.0, 0.0, 0.0]); self.bias = np.zeros(3); self.kp = kp; self.ki = ki; self.dt = 1.0 / frequency; self.gravity_ref = np.array([0.0, 0.0, 1.0])

    def update(self, gyr, acc, mag):
        acc_n, mag_n = _normalize(acc), _normalize(mag); dcm = _q_to_dcm(self.q); v_acc = dcm.T @ self.gravity_ref; error_acc = np.cross(acc_n, v_acc)
        mag_world_est = dcm @ mag_n; b_x = np.sqrt(mag_world_est[0]**2 + mag_world_est[1]**2); b_z = mag_world_est[2]; mag_ref_world = np.array([0.0, b_x, b_z])
        mag_ref_sensor_frame = dcm.T @ mag_ref_world; error_mag = np.cross(mag_n, mag_ref_sensor_frame); error = error_acc + error_mag
        self.bias += -self.ki * error * self.dt; omega = gyr - self.bias + self.kp * error; q_dot = 0.5 * _q_multiply(self.q, np.array([0.0, *omega]))
        self.q = _normalize(self.q + q_dot * self.dt); return self.q
    
    def set_initial_state(self, q0): self.q = np.copy(q0)

class IMUSystem:
    CACHE_FILENAME = "orientation_cache.json"

    def __init__(self, kp=17.0, ki=0.2, recalibrate_on_start=False):
        self.filter = AttitudeFilter(kp=kp, ki=ki)
        self.mag_calibrator = MagnetometerCalibrator(cache_path=self.CACHE_FILENAME)
        self.recalibrate = recalibrate_on_start
        self.reference_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.running = False
        self._threads = []
        self.latest_icm = None; self.latest_mmc = None
        self.latest_fused_data = None
        self.data_lock = threading.Lock(); self.reference_lock = threading.Lock()

        try: from icm20602 import ICM20602; self.icm = ICM20602(); logging.info("OK: ICM20602 sensor found.")
        except Exception: logging.warning("ICM20602 not found. Using mock sensor."); self.icm = MockICM()

        try: from mmc5983 import MMC5983; self.mmc = MMC5983(); logging.info("OK: MMC5983 sensor found.")
        except Exception: logging.warning("MMC5983 not found. Using mock sensor."); self.mmc = MockMMC()

    def start(self):
        self.running = True
        self._threads = [
            threading.Thread(target=self._read_icm_loop, daemon=True),
            threading.Thread(target=self._read_mmc_loop, daemon=True),
            threading.Thread(target=self.mag_calibrator.calibration_loop, daemon=True),
            threading.Thread(target=self._filter_loop, daemon=True)
        ]
        for t in self._threads: t.start()
        logging.info("IMU System started with all threads.")

    def stop(self):
        logging.info("Stopping IMU System...")
        self.running = False
        self.mag_calibrator.stop()
        for t in self._threads:
            if t.is_alive(): t.join(timeout=1.0)
        logging.info("IMU System stopped.")

    def reset_orientation(self):
        with self.reference_lock, self.data_lock:
            # Set the current *absolute* orientation as the new zero reference
            self.reference_quaternion = np.copy(self.filter.q)
        logging.info(f"Reference orientation tared. New zero is {self.reference_quaternion}")

    def get_latest_data(self) -> Optional[dict]:
        """Returns the most recent fused orientation data."""
        with self.data_lock:
            return self.latest_fused_data

    def _compute_relative_quaternion(self, q_current):
        with self.reference_lock:
            q_ref_inv = _q_conjugate(self.reference_quaternion)
            return _q_multiply(q_ref_inv, q_current)

    def _read_icm_loop(self):
        while self.running:
            try:
                d=self.icm.read_all(); ax,ay,az,gx,gy,gz=d.a.x,d.a.y,d.a.z,d.g.x,d.g.y,d.g.z
                with self.data_lock: self.latest_icm={'ax':ax,'ay':ay,'az':az,'gx':np.radians(gx),'gy':np.radians(gy),'gz':np.radians(gz)}
                time.sleep(0.01)
            except Exception as e: logging.error(f"ICM20602 read error: {e}"); time.sleep(1)

    def _read_mmc_loop(self):
        while self.running:
            try:
                d=self.mmc.read_data(); mx,my,mz=d.x,d.y,d.z; mag_raw = np.array([mx, my, mz])
                self.mag_calibrator.add_point(mag_raw)
                with self.data_lock: self.latest_mmc={'mx':mx,'my':my,'mz':mz}
                time.sleep(0.02)
            except Exception as e: logging.error(f"MMC5983 read error: {e}"); time.sleep(1)

    def _initialize_filter_state(self):
        q0 = None
        if not self.recalibrate:
            try:
                if os.path.exists(self.CACHE_FILENAME):
                    with open(self.CACHE_FILENAME, 'r') as f: data = json.load(f)
                    if 'q0' in data and len(data['q0']) == 4:
                        q0 = np.array(data['q0']); logging.info("Loaded initial orientation from cache.")
            except Exception as e: logging.warning(f"Could not load q0 from cache: {e}")
        
        if q0 is None:
            logging.info("No valid orientation in cache. Place device level to initialize (3s)...")
            time.sleep(3.0)
            acc_readings = []
            for _ in range(20):
                with self.data_lock:
                    if self.latest_icm: acc_readings.append([self.latest_icm['ax'], self.latest_icm['ay'], self.latest_icm['az']])
                time.sleep(0.1)
            if not acc_readings:
                logging.error("No accelerometer data for initialization. Using default."); q0 = np.array([1.0, 0, 0, 0])
            else:
                acc_avg = np.mean(acc_readings, axis=0); q0 = _get_q_from_acc(acc_avg)
                logging.info(f"Initialized with new orientation from accelerometer: {q0}")
            try:
                data = {}; 
                if os.path.exists(self.CACHE_FILENAME):
                    with open(self.CACHE_FILENAME, 'r') as f: data = json.load(f)
                data['q0'] = q0.tolist()
                with open(self.CACHE_FILENAME, 'w') as f: json.dump(data, f, indent=4)
            except Exception as e: logging.error(f"Could not save initial q0 to cache: {e}")
        
        self.filter.set_initial_state(q0)
        self.reset_orientation() # Set the initial reference to this q0

    def _filter_loop(self):
        logging.info("Filter loop waiting for sensor data..."); time.sleep(1)
        self._initialize_filter_state()
        logging.info("IMU initialization complete. Starting attitude estimation.")
        
        while self.running:
            with self.data_lock: icm_data=self.latest_icm; mmc_data=self.latest_mmc
            if icm_data and mmc_data:
                acc_raw=np.array([icm_data['ax'], icm_data['ay'], icm_data['az']]); gyr_raw=np.array([icm_data['gx'], icm_data['gy'], icm_data['gz']]); mag_raw=np.array([mmc_data['mx'], mmc_data['my'], mmc_data['mz']])
                mag_corrected = self.mag_calibrator.correct(mag_raw)
                q_absolute = self.filter.update(gyr_raw, acc_raw, mag_corrected)
                q_relative = self._compute_relative_quaternion(q_absolute)
                roll, pitch, yaw = _quaternion_to_euler(q_relative)

                with self.data_lock:
                    self.latest_fused_data = {
                        'timestamp': time.time(),
                        'quaternion': q_relative.tolist(),
                        'euler_deg': {'roll': roll, 'pitch': pitch, 'yaw': yaw}
                    }
            time.sleep(self.filter.dt)
