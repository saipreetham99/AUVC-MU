#!/usr/bin/env python3
"""
Complete BlueROV2 Controller with a single, unified display.

This version sets the requested Matplotlib viewer angle and removes
unused command-line arguments for a cleaner implementation.
"""

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

# Use a non-interactive backend for Matplotlib so it doesn't create its own window
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from typing import Optional, Tuple
from enum import Enum
from dataclasses import dataclass


# ============================================================================
# GUI VISUALIZER CLASS
# ============================================================================
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

        plot_dpi = 100
        self.fig = plt.figure(figsize=(self.plot_width / plot_dpi, self.screen_height / plot_dpi), dpi=plot_dpi)
        self.ax_3d = self.fig.add_subplot(1, 1, 1, projection='3d')
        self.setup_3d_plot()

        self.rotation_fix = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        self.hud_data = {'armed': False, 'light_on': False, 'yaw_kp': 0.0}

    def setup_3d_plot(self):
        """Initializes the Matplotlib 3D plot properties."""
        self.ax_3d.set_xlabel('X (Fwd)'); self.ax_3d.set_ylabel('Y (Right)'); self.ax_3d.set_zlabel('Z (Down)')
        self.ax_3d.set_xlim([-1, 1]); self.ax_3d.set_ylim([-1, 1]); self.ax_3d.set_zlim([-1, 1])
        try:
            self.ax_3d.set_box_aspect([1, 1, 1])
        except AttributeError:
            pass
        # Set the user-requested viewer angle
        self.ax_3d.view_init(elev=-5, azim=90)
        
        colors = ['green', 'red', 'blue']
        labels = ['Y (Pitch)', 'X (Roll)', 'Z (Yaw)']
        self.frame_lines = [self.ax_3d.plot([], [], [], c=c, lw=3, label=l)[0] for c, l in zip(colors, labels)]
        self.ax_3d.legend()
        self.fig.tight_layout(pad=0)

    def _quaternion_to_rotation_matrix(self, q):
        q0, q1, q2, q3 = q
        return np.array([
            [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])

    def update(self):
        """Update and draw all UI elements to the single Pygame window."""
        self.screen.fill((25, 25, 25))
        with self.data_receiver.frame_lock:
            frame = self.data_receiver.latest_frame
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            camera_surface = pygame.surfarray.make_surface(np.rot90(frame_rgb))
            self.screen.blit(camera_surface, (0, 0))

        arm_text = self.font.render(f"ARMED: {self.hud_data['armed']}", 1, (255,0,0) if self.hud_data['armed'] else (0,255,0))
        light_text = self.font.render(f"LIGHT: {'ON' if self.hud_data['light_on'] else 'OFF'}", 1, (255,255,0))
        kp_text = self.font.render(f"Yaw Kp: {self.hud_data['yaw_kp']:.2f}", 1, (255, 255, 255))
        self.screen.blit(arm_text, (10, 10)); self.screen.blit(light_text, (10, 30)); self.screen.blit(kp_text, (10, 50))

        imu_data = self.data_receiver.latest_imu_data
        if imu_data and 'quaternion' in imu_data and not np.any(np.isnan(imu_data['quaternion'])):
            q = np.array(imu_data['quaternion'])
            R = self.rotation_fix @ self._quaternion_to_rotation_matrix(q)
            x_ax, y_ax, z_ax = R[:, 1], R[:, 0], R[:, 2]
            self.frame_lines[0].set_data([0, x_ax[0]], [0, x_ax[1]]); self.frame_lines[0].set_3d_properties([0, x_ax[2]])
            self.frame_lines[1].set_data([0, y_ax[0]], [0, y_ax[1]]); self.frame_lines[1].set_3d_properties([0, y_ax[2]])
            self.frame_lines[2].set_data([0, z_ax[0]], [0, z_ax[1]]); self.frame_lines[2].set_3d_properties([0, z_ax[2]])
            self.ax_3d.set_title('3D Orientation (Live)')
        else:
            self.ax_3d.set_title('3D Orientation (Waiting...)')
        
        self.fig.canvas.draw()
        size = self.fig.canvas.get_width_height()
        buffer = self.fig.canvas.buffer_rgba()
        plot_surface = pygame.image.frombuffer(buffer, size, "RGBA")
        self.screen.blit(plot_surface, (self.camera_width, 0))
        pygame.display.flip()

# ============================================================================
# NETWORK AND DATA CLASSES
# ============================================================================
class RealSubmarineClient:
    def __init__(self, server_ip, control_port=10000):
        self.server_address = (server_ip, control_port)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"RealSubClient: Ready for {self.server_address}")
        self.NEUTRAL_PULSE=1500; self.LIGHT_OFF, self.LIGHT_ON=1100, 1900
        self.AMP=400; self.LOOP_DURATION=0.02

    def _to_pwm(self, nf):
        return int(self.NEUTRAL_PULSE + nf * self.AMP)

    def _send_packet(self, cmd_dict):
        """Internal method to send any JSON command."""
        try:
            payload = (json.dumps(cmd_dict) + '\n').encode('utf-8')
            self.control_socket.sendto(payload, self.server_address)
        except Exception as e:
            print(f"RealSubClient Error: {e}", file=sys.stderr)
            
    def send_control(self, armed, light_on, forces):
        if not armed:
            forces = np.zeros(6)
        nf = np.clip(forces / 100.0, -1.0, 1.0)
        tp = [self._to_pwm(v) for v in nf]
        light = self.LIGHT_ON if light_on else self.LIGHT_OFF
        cmd = {'command': 'control', 'thruster_pulses': tp, 'light_pulse': light, 'duration': self.LOOP_DURATION}
        self._send_packet(cmd)

    def send_reset_command(self):
        """Sends a specific command to reset the IMU orientation."""
        print(f"Sending 'reset_orientation' command to {self.server_address}...")
        cmd = {'command': 'reset_orientation'}
        self._send_packet(cmd)

    def shutdown(self):
        print("RealSubClient: Shutting down.")
        self.send_control(False, False, np.zeros(6))
        time.sleep(0.1)
        self.control_socket.close()

class SubmarineDataReceiverThread(threading.Thread):
    def __init__(self, video_port=10001, imu_port=10002):
        super().__init__(daemon=True, name="SubmarineDataReceiver")
        self.video_listen_addr=('', video_port); self.imu_listen_addr=('', imu_port)
        self.latest_frame=None; self.latest_imu_data=None
        self.frame_lock=threading.Lock(); self.stop_event=threading.Event()
    def _video_loop(self):
        s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(self.video_listen_addr); s.settimeout(1.0); b={}
        while not self.stop_event.is_set():
            try:
                d,_=s.recvfrom(65536)
                if len(d)<6: continue
                fid,cs,cid=struct.unpack('!HHH',d[:6])
                if fid not in b: b[fid]={}
                b[fid][cid]=d[6:]
                if len(b[fid])==cs:
                    jpeg=b''.join(v for k,v in sorted(b[fid].items()))
                    fr=cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
                    with self.frame_lock: self.latest_frame=fr
                    del b[fid]
            except socket.timeout: continue
            except Exception: pass
        s.close(); print("SubDataReceiver: Video loop stopped.")
    def _imu_loop(self):
        s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(self.imu_listen_addr); s.settimeout(1.0)
        while not self.stop_event.is_set():
            try:
                d,_=s.recvfrom(1024)
                self.latest_imu_data=json.loads(d.decode('utf-8'))
            except socket.timeout: continue
            except Exception: self.latest_imu_data=None
        s.close(); print("SubDataReceiver: IMU loop stopped.")
    def run(self):
        print("SubDataReceiver: Starting."); vt=threading.Thread(target=self._video_loop,daemon=True); it=threading.Thread(target=self._imu_loop,daemon=True); vt.start(); it.start(); vt.join(); it.join()
    def stop(self): self.stop_event.set()

class MsgHeader(Enum): GET_SENSORDATA=4; APPLY_CTRL=7; RESET=9
class AUV:
    def __init__(self, ip, port):
        self.socket=socket.socket(socket.AF_INET, socket.SOCK_STREAM); self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1); self.socket.settimeout(15.0); self.socket.connect((ip, port)); print(f"AUV: Connected to {ip}:{port}")
    def _send(self, data, expected_bytes, command_name):
        try:
            self.socket.sendall(data); response=b''; bytes_received=0; start_time=time.time()
            while bytes_received < expected_bytes and time.time() - start_time < 10.0:
                chunk = self.socket.recv(expected_bytes - bytes_received)
                if not chunk: raise ConnectionError("Socket closed by server")
                response += chunk; bytes_received += len(chunk)
            if bytes_received < expected_bytes: raise TimeoutError(f"'{command_name}' command timed out")
            return response
        except Exception as e:
            print(f"AUV Error during '{command_name}': {e}", file=sys.stderr); return None
    def get_sensor_data(self) -> Optional[dict]:
        d=struct.pack('<f',MsgHeader.GET_SENSORDATA.value); r=self._send(d,20,"GET_SENSORDATA")
        return {'imu_quaternion': np.array(struct.unpack('<5f', r)[:4])} if r else None
    def apply_ctrl(self, f, ns) -> Optional[dict]:
        d=struct.pack('<8f',MsgHeader.APPLY_CTRL.value,float(ns),*f); r=self._send(d,20,"APPLY_CTRL")
        return {'imu_quaternion': np.array(struct.unpack('<5f', r)[:4])} if r else None
    def reset(self) -> bool:
        pose=np.array([0,0,0,1,0,0,0]); vel=np.zeros(3); ang_vel=np.zeros(3)
        data=struct.pack('<15f',MsgHeader.RESET.value,0.0,*pose,*vel,*ang_vel)
        response=self._send(data, 1, "RESET"); return response is not None
    def close(self): self.socket.close()

# ============================================================================
# CONTROLLER (Main Brain)
# ============================================================================
@dataclass
class PIDGains: kp: float; ki: float; kd: float
@dataclass
class PIDState: prev_error: float=0.0; integral: float=0.0

class BlueROVController:
    def __init__(self, auv_ip, auv_port, sub_ip):
        pygame.init(); pygame.joystick.init(); self.controller=pygame.joystick.Joystick(0); self.controller.init(); print(f"Gamepad: {self.controller.get_name()}")
        self.auv=AUV(auv_ip, auv_port)
        self.visualizer=None; self.real_sub_client=None; self.data_receiver_thread=None
        if sub_ip:
            self.real_sub_client = RealSubmarineClient(sub_ip)
            self.data_receiver_thread = SubmarineDataReceiverThread()
        self.armed=False; self.light_on=False; self.minForce=np.full(6, -100); self.maxForce=np.full(6, 100); self.manual_force_scale=100.0; self.dt=0.02; self.last_time=time.time(); self.DEADZONE=0.1
        self.target_attitude=np.array([1.0,0.0,0.0,0.0]); self.pid_gains={'yaw':PIDGains(kp=0.0,ki=0.0,kd=0.0)}; self.pid_states={'yaw':PIDState()}; self.kp_step=0.1; self.stabilization_force_multiplier=0.1; self.stabilization_force_max=15.0
        self.BTN_A=0; self.BTN_X=2; self.BTN_MENU=6; self.BTN_VIEW=4; self.BTN_HAT_UP=11; self.BTN_HAT_DOWN=12; self.LEFT_STICK_Y=1; self.LEFT_STICK_X=0; self.RIGHT_STICK_X=2; self.RIGHT_TRIGGER=5; self.LEFT_TRIGGER=4; self.translation_scale=0.2; self.rotation_scale=0.4

    def quaternion_to_euler(self, q:np.ndarray) -> Tuple[float,float,float]:
        w,x,y,z=q; r=np.arctan2(2*(w*x+y*z),1-2*(x**2+y**2)); p=np.arcsin(np.clip(2*(w*y-z*x),-1,1)); y_angle=np.arctan2(2*(w*z+x*y),1-2*(y**2+z**2)); return r,p,y_angle
    
    def pid_control(self, error:float) -> float:
        if np.isnan(error): print("PID Error: NaN input.", file=sys.stderr); return 0.0
        gains=self.pid_gains['yaw']; state=self.pid_states['yaw']
        state.integral = np.clip(state.integral + error * self.dt, -10.0, 10.0)
        derivative = (error - state.prev_error) / self.dt
        state.prev_error = error
        output = gains.kp * error + gains.ki * state.integral + gains.kd * derivative
        return 0.0 if np.isnan(output) else output
    
    def calculate_stabilization_forces(self, current_attitude:np.ndarray) -> np.ndarray:
        if current_attitude is None or np.any(np.isnan(current_attitude)): return np.zeros(6)
        _,_,current_yaw=self.quaternion_to_euler(current_attitude)
        _,_,target_yaw=self.quaternion_to_euler(self.target_attitude)
        if np.isnan(current_yaw) or np.isnan(target_yaw): return np.zeros(6)
        yaw_error_deg=np.degrees(target_yaw-current_yaw); yaw_error_deg=(yaw_error_deg+180)%360-180
        yaw_correction=self.pid_control(yaw_error_deg)
        stab_forces=np.zeros(6)
        stab_forces[0]+=yaw_correction; stab_forces[1]-=yaw_correction
        stab_forces[2]-=yaw_correction; stab_forces[3]+=yaw_correction
        return np.clip(stab_forces*self.stabilization_force_multiplier, -self.stabilization_force_max, self.stabilization_force_max)
    
    def get_controller_input(self):
        def dz(v): return 0 if abs(v)<self.DEADZONE else np.sign(v)*(abs(v)-self.DEADZONE)/(1-self.DEADZONE)
        s=-dz(self.controller.get_axis(self.LEFT_STICK_Y)); t=dz(self.controller.get_axis(self.LEFT_STICK_X)); y=-dz(self.controller.get_axis(self.RIGHT_STICK_X)); h=(self.controller.get_axis(self.RIGHT_TRIGGER)-self.controller.get_axis(self.LEFT_TRIGGER))*0.5
        if not self.armed: return 0,0,0,0
        return s*self.translation_scale, t*self.translation_scale, h*self.translation_scale, y*self.rotation_scale
    
    def calculate_thruster_forces(self, s, t, h, y, stab_forces):
        f=np.zeros(6); f[0]=s-t+y; f[1]=s+t-y; f[2]=-s-t-y; f[3]=-s+t+y; f[4]=h; f[5]=h
        total_forces = f * self.manual_force_scale + stab_forces
        return np.clip(total_forces, self.minForce, self.maxForce)

    def run_controller(self, num_steps):
        print("🚀 Starting controller loop.")
        if self.data_receiver_thread:
            self.visualizer=CombinedVisualizer(self.data_receiver_thread)
            self.data_receiver_thread.start()
        
        sensor_data=self.auv.get_sensor_data()
        if not sensor_data: raise RuntimeError("No initial simulator data.")
        
        shutdown=False
        try:
            while not shutdown:
                if time.time()-self.last_time < self.dt: time.sleep(0.002); continue
                self.last_time=time.time()
                for e in pygame.event.get():
                    if e.type == pygame.QUIT: shutdown=True; break
                    if e.type == pygame.JOYBUTTONDOWN:
                        if e.button == self.BTN_MENU: self.armed=not self.armed; print(f"ARMED: {self.armed}")
                        elif e.button == self.BTN_X: self.light_on=not self.light_on; print(f"Light: {'ON' if self.light_on else 'OFF'}")
                        elif e.button == self.BTN_A:
                            print("Resetting Orientation...")
                            if self.real_sub_client: self.real_sub_client.send_reset_command()
                            self.auv.reset(); self.target_attitude=np.array([1.,0.,0.,0.])
                        elif e.button == self.BTN_HAT_UP: self.pid_gains['yaw'].kp+=self.kp_step; print(f"Yaw Kp: {self.pid_gains['yaw'].kp:.2f}")
                        elif e.button == self.BTN_HAT_DOWN: self.pid_gains['yaw'].kp=max(0.0,self.pid_gains['yaw'].kp-self.kp_step); print(f"Yaw Kp: {self.pid_gains['yaw'].kp:.2f}")
                        elif e.button == self.BTN_VIEW: raise KeyboardInterrupt("E-Stop")
                if shutdown: continue
                
                current_attitude = sensor_data['imu_quaternion'] if sensor_data else None
                stab_forces = self.calculate_stabilization_forces(current_attitude)
                s,t,h,y = self.get_controller_input()
                final_forces = self.calculate_thruster_forces(s,t,h,y,stab_forces)
                
                if self.armed:
                    if self.real_sub_client: self.real_sub_client.send_control(self.armed, self.light_on, final_forces)
                    sensor_data = self.auv.apply_ctrl(final_forces, num_steps)
                else:
                    if self.real_sub_client: self.real_sub_client.send_control(False, False, np.zeros(6))
                    sensor_data = self.auv.apply_ctrl(np.zeros(6), num_steps)
                
                if not sensor_data: print("Lost sim connection."); shutdown=True
                if self.visualizer:
                    self.visualizer.hud_data.update({'armed':self.armed, 'light_on':self.light_on, 'yaw_kp':self.pid_gains['yaw'].kp})
                    self.visualizer.update()
        except (KeyboardInterrupt, SystemExit):
            print("\nController stopped.")
        finally:
            print("Disarming and stopping systems...")
            if self.data_receiver_thread: self.data_receiver_thread.stop(); self.data_receiver_thread.join(timeout=2)
            plt.close('all')
            try:
                self.auv.apply_ctrl(np.zeros(6), 1)
                self.auv.close()
            except:
                pass
            if self.real_sub_client:
                self.real_sub_client.shutdown()
            pygame.quit()
            print("Cleanup complete.")

# ============================================================================
# MAIN EXECUTION
# ============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Synchronized BlueROV2 Controller with Visualization")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="IP of the MuJoCo simulator.")
    parser.add_argument("--port", type=int, default=60001, help="Port of the MuJoCo simulator.")
    parser.add_argument("--num_steps", type=int, default=1, help="Simulator physics steps per cycle.")
    parser.add_argument("--sub_ip", type=str, default=None, help="IP of real submarine. Enables all real-sub features.")
    args = parser.parse_args()
    try:
        controller = BlueROVController(args.ip, args.port, args.sub_ip)
        controller.run_controller(args.num_steps)
    except Exception as e:
        print(f"\n[FATAL ERROR] Could not start: {e}", file=sys.stderr)
        pygame.quit()
        sys.exit(1)
