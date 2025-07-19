#!/usr/bin/env python3
"""
Complete BlueROV2 Controller System with optional real-time attitude and thruster visualization.
Client is compatible with a UDP-based PCA9685 motor controller.
"""

import pygame
import numpy as np
import time
import sys
import socket
import struct
import json
import threading
import argparse
from typing import Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
from collections import deque

# Attempt to import matplotlib, but don't make it a hard requirement
try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button
    from matplotlib.gridspec import GridSpec
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


# ============================================================================
# SIMULATION AUV CLASS (No changes)
# ============================================================================
class MsgHeader(Enum):
    ERROR=0;NO_ERROR=1;HEARTBEAT=2;GET_MODEL_INFO=3;GET_SENSORDATA=4;GET_RGB_IMAGE=5;GET_MASKED_IMAGE=6;APPLY_CTRL=7;STEP_SIM=8;RESET=9
class AUV:
    def __init__(self,server_ip="127.0.0.1",server_port=60001):self.server_ip,self.server_port=server_ip,server_port;self.socket=None;self.connect()
    def __del__(self):self.stop()
    def connect(self):
        try:self.socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM);self.socket.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);self.socket.settimeout(10.0);self.socket.connect((self.server_ip,self.server_port));print(f"✓ Sim AUV: Connected to {self.server_ip}:{self.server_port}")
        except Exception as e:print(f"✗ Sim AUV: Connection failed: {e}");raise
    def stop(self):
        if self.socket:
            try:self.socket.close();print("Sim AUV: Disconnected")
            except:pass
        self.socket=None
    def _send_command(self,data,expected_bytes):
        if not self.socket:raise RuntimeError("Not connected to sim server");self.socket.sendall(data);return self.socket.recv(expected_bytes)if expected_bytes>0 else b''
    def get_sensor_data(self):
        data=struct.pack('<f',float(MsgHeader.GET_SENSORDATA.value));response=self._send_command(data,20)
        if len(response)>=20:values=struct.unpack('<5f',response);return{'imu_quaternion':np.array(values[:4]),'time':values[-1]}
        return None
    def apply_ctrl(self,forces,num_steps=1):self._send_command(struct.pack('<8f',float(MsgHeader.APPLY_CTRL.value),float(num_steps),*forces),0)

# ============================================================================
# REAL SUBMARINE CLIENTS (No changes)
# ============================================================================
class AttitudeClient:
    def __init__(self, host='127.0.0.1', port=9999):
        self.host, self.port = host, port;self.socket, self.running, self.connected = None, False, False;self.latest_quaternion = np.array([1.0, 0.0, 0.0, 0.0]);self.data_lock = threading.Lock();history_len=500;self.orientation_offset = np.array([1.0, 0.0, 0.0, 0.0]);self.time_history=deque(maxlen=history_len);self.roll_history=deque(maxlen=history_len);self.pitch_history=deque(maxlen=history_len);self.yaw_history=deque(maxlen=history_len);self.start_time=time.time()
    def _quat_mult(self,q1,q2):w1,x1,y1,z1=q1;w2,x2,y2,z2=q2;return np.array([w1*w2-x1*x2-y1*y2-z1*z2,w1*x2+x1*w2+y1*z2-z1*y2,w1*y2-x1*z2+y1*w2+z1*x2,w1*z2+x1*y2-y1*x2+z1*w2])
    def _quat_conj(self,q):return np.array([q[0],-q[1],-q[2],-q[3]])
    def _quat_to_euler(self,q):w,x,y,z=q;r=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));p=np.arcsin(np.clip(2*(w*y-z*x),-1,1));yw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z));return np.degrees(r),np.degrees(p),np.degrees(yw)
    def reset_orientation_origin(self):
        with self.data_lock:print("\n--- Resetting Orientation Origin ---");self.orientation_offset=self._quat_conj(self.latest_quaternion);self.time_history.clear();self.roll_history.clear();self.pitch_history.clear();self.yaw_history.clear()
    def _connect(self):
        try:self.socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM);self.socket.settimeout(5.0);self.socket.connect((self.host,self.port));self.socket.settimeout(None);self.connected=True;print(f"✓ Attitude Client: Connected to {self.host}:{self.port}");return True
        except Exception as e:print(f"✗ Attitude Client: Connection failed: {e}");self.connected=False;return False
    def receive_data(self):
        buffer = "";
        while self.running:
            if not self.connected:print("Attitude Client: Disconnected. Reconnecting...");time.sleep(5);self._connect();continue
            try:
                data=self.socket.recv(4096).decode('utf-8')
                if not data:self.connected=False;continue
                buffer+=data
                while '\n' in buffer:
                    line,buffer=buffer.split('\n',1)
                    if line:
                        try:
                            q_raw=np.array(json.loads(line)['quaternion'])
                            with self.data_lock:self.latest_quaternion=q_raw;q_corrected=self._quat_mult(self.orientation_offset,q_raw);roll,pitch,yaw=self._quat_to_euler(q_corrected);self.time_history.append(time.time()-self.start_time);self.roll_history.append(roll);self.pitch_history.append(pitch);self.yaw_history.append(yaw)
                        except(json.JSONDecodeError,KeyError,IndexError):pass
            except Exception:self.connected=False
    def start(self):
        if self._connect():self.running=True;threading.Thread(target=self.receive_data,daemon=True).start();return True
        return False
    def stop(self):
        self.running=False;
        if self.socket:
            try:self.socket.shutdown(socket.SHUT_RDWR);self.socket.close()
            except(OSError,socket.error):pass
    def get_latest_quaternion(self):
        with self.data_lock:return self.latest_quaternion
class UDPThrusterClient:
    def __init__(self,host:str,port:int):self.host,self.port=host,port;self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.NEUTRAL_PULSE=1500;self.PWM_RANGE=400;self.LIGHT_OFF_PULSE=1100;self.LIGHT_ON_PULSE=1900;print(f"✓ UDP Thruster Client initialized for {host}:{port}")
    def _force_to_pwm(self,force:float)->int:force=np.clip(force,-100.0,100.0);return self.NEUTRAL_PULSE+int((force/100.0)*self.PWM_RANGE)
    def apply_ctrl(self,forces:np.ndarray,light_on:bool):
        if len(forces)!=6:return
        pulse_list=[self._force_to_pwm(f)for f in forces];light_pulse=self.LIGHT_ON_PULSE if light_on else self.LIGHT_OFF_PULSE;pulse_list.append(light_pulse)
        try:packet=struct.pack("<7H",*pulse_list);self.socket.sendto(packet,(self.host,self.port))
        except Exception as e:print(f"UDP send error: {e}")
    def stop(self):print("UDP Client: Sending disarm packet.");self.apply_ctrl(np.zeros(6),light_on=False);time.sleep(0.05);self.socket.close()

# ============================================================================
# VISUALIZER (MODIFIED)
# ============================================================================
class AttitudeVisualizer:
    ### MODIFIED: Now accepts the controller to read thruster data from it ###
    def __init__(self, attitude_client: AttitudeClient, controller: 'BlueROVController'):
        self.att_client = attitude_client
        self.controller = controller
        self.time_window_size = 30.0

        plt.style.use('seaborn-v0_8-darkgrid')
        self.fig = plt.figure(figsize=(18, 9))
        self.fig.suptitle(f'BlueROV2 Real-Time Diagnostics - {self.att_client.host}', fontsize=16)

        ### MODIFIED: More flexible grid layout ###
        gs = GridSpec(2, 2, height_ratios=[1, 1])
        self.ax_euler = self.fig.add_subplot(gs[0, 0])
        self.ax_thrusters = self.fig.add_subplot(gs[1, 0])
        self.ax_3d = self.fig.add_subplot(gs[:, 1], projection='3d')
        self.info_text = self.fig.text(0.5, 0.96, 'Initializing...', ha='center', va='center', fontsize=12)

        self.setup_plots()

    def _quat_to_rot_matrix(self,q):q0,q1,q2,q3=q;return np.array([[q0*q0+q1*q1-q2*q2-q3*q3,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2)],[2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3,2*(q2*q3-q0*q1)],[2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3]])

    def setup_plots(self):
        # Euler plot
        self.line_roll, = self.ax_euler.plot([],[],'r-',lw=2,label='Roll');self.line_pitch, = self.ax_euler.plot([],[],'g-',lw=2,label='Pitch');self.line_yaw, = self.ax_euler.plot([],[],'b-',lw=2,label='Yaw')
        self.ax_euler.set_title('Euler Angles');self.ax_euler.set_ylabel('Angle (degrees)');self.ax_euler.legend(loc='upper left');self.ax_euler.set_ylim(-185,185);self.ax_euler.grid(True,alpha=0.5)
        self.status_text = self.ax_euler.text(0.98, 0.98, 'Connecting...', transform=self.ax_euler.transAxes,fontsize=12,color='orange',ha='right',va='top',bbox=dict(boxstyle='round',fc='white',alpha=0.8))

        ### NEW: Thruster bar plot setup ###
        self.thruster_labels = ['T1:FL', 'T2:FR', 'T3:RL', 'T4:RR', 'T5:V1', 'T6:V2']
        self.thruster_bars = self.ax_thrusters.barh(self.thruster_labels, [0]*6, color='gray')
        self.ax_thrusters.set_title('Thruster Output (-100 to 100)')
        self.ax_thrusters.set_xlim(-105, 105)
        self.ax_thrusters.axvline(0, color='black', linestyle='--', linewidth=1) # Zero line
        self.ax_thrusters.grid(True, axis='x', linestyle=':', alpha=0.6)

        # 3D plot
        self.ax_3d.set_title('3D Orientation');self.ax_3d.set_box_aspect([1,1,1]);self.ax_3d.set_xlim([-1,1]);self.ax_3d.set_ylim([-1,1]);self.ax_3d.set_zlim([-1,1]);self.frame_lines=[self.ax_3d.plot([],[],[],c,lw=4)[0] for c in['r','g','b']]
        ax_button=self.fig.add_axes([0.45,0.01,0.1,0.05]);self.reset_button=Button(ax_button,'Reset Origin');self.reset_button.on_clicked(lambda e:self.att_client.reset_orientation_origin())

    def animate(self, frame):
        # Get attitude data
        with self.att_client.data_lock:times=list(self.att_client.time_history);rolls=list(self.att_client.roll_history);pitches=list(self.att_client.pitch_history);yaws=list(self.att_client.yaw_history);q_raw=self.att_client.latest_quaternion;q_corrected=self.att_client._quat_mult(self.att_client.orientation_offset,q_raw)

        # Get latest thruster forces from the controller
        thruster_forces = self.controller.latest_forces

        # Update plots if there is data
        if not times:return[]

        self.status_text.set_text('Connected' if self.att_client.connected else'Disconnected');self.status_text.set_color('g' if self.att_client.connected else'r')
        self.line_roll.set_data(times,rolls);self.line_pitch.set_data(times,pitches);self.line_yaw.set_data(times,yaws)
        self.ax_euler.set_xlim(max(0,times[-1]-self.time_window_size),times[-1]+1);self.ax_euler.relim();self.ax_euler.autoscale_view(scalex=False,scaley=True)

        ### NEW: Update thruster bars ###
        for i, bar in enumerate(self.thruster_bars):
            force = thruster_forces[i]
            bar.set_width(force)
            bar.set_color('cyan' if force >= 0 else 'coral')

        # Update 3D plot
        R=self._quat_to_rot_matrix(q_corrected)
        for i,line in enumerate(self.frame_lines):axis=R[:,i];line.set_data([0,axis[0]],[0,axis[1]]);line.set_3d_properties([0,axis[2]])

        rate=len(times)/(times[-1]-times[0]+1e-6)if len(times)>1 else 0
        info_str = f'Roll:{rolls[-1]:6.1f}° | Pitch:{pitches[-1]:6.1f}° | Yaw:{yaws[-1]:6.1f}° | Rate:{rate:.1f}Hz'
        self.info_text.set_text(info_str)

        return [self.line_roll,self.line_pitch,self.line_yaw,self.info_text,self.status_text,*self.frame_lines,*self.thruster_bars]

    def run(self):
        self.ani=FuncAnimation(self.fig,self.animate,interval=50,blit=False)
        plt.tight_layout(rect=[0,0.05,1,0.95])
        plt.show()

# ============================================================================
# CONTROLLER CLASS (MODIFIED)
# ============================================================================
@dataclass
class PIDGains: kp: float; ki: float; kd: float
@dataclass
class PIDState: prev_error: float = 0.0; integral: float = 0.0

class BlueROVController:
    def __init__(self, command_interface: Any, attitude_client: Optional[AttitudeClient] = None):
        self.command_interface=command_interface;self.attitude_client=attitude_client;self.is_real_sub_mode=(attitude_client is not None);pygame.init();pygame.joystick.init()
        if pygame.joystick.get_count()==0:raise RuntimeError("No gamepad found")
        self.joystick=pygame.joystick.Joystick(0);self.joystick.init()
        self.MAX_FORCE=100.0;self.MIN_FORCE=-100.0;self.dt=0.02;self.last_time=time.time();self.armed=False;self.stabilization_enabled=True;self.light_on=False

        ### NEW: Attribute to store latest forces for the visualizer ###
        self.latest_forces = np.zeros(6)

        self.setup_controller_mapping();self.setup_pid_controllers()
        self.translation_scale=0.4;self.rotation_scale=0.5;self.stabilization_force_multiplier=0.15;self.stabilization_force_max=20.0;self.angle_deadzone={'roll':3,'pitch':3,'yaw':5}
        print(f"\nController initialized for: {'REAL SUBMARINE' if self.is_real_sub_mode else 'SIMULATION'}")
    def setup_controller_mapping(self):self.axis_map={'lx':0,'ly':1,'rx':2,'ry':3,'lt':4,'rt':5};self.button_map={'a':0,'b':1,'start':6};self.deadzone=0.1
    def setup_pid_controllers(self):self.pid_gains={'roll':PIDGains(0.9,0.1,0.6),'pitch':PIDGains(0.9,0.1,0.6),'yaw':PIDGains(1.2,0.1,0.7)};self.pid_states={ax:PIDState()for ax in self.pid_gains};self.target_attitude=np.array([1.0,0.0,0.0,0.0])
    def _quaternion_to_euler(self,q):w,x,y,z=q;r=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));p=np.arcsin(np.clip(2*(w*y-z*x),-1,1));yw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z));return r,p,yw
    def _apply_deadzone(self,value):return 0 if abs(value)<self.deadzone else np.sign(value)*(abs(value)-self.deadzone)/(1-self.deadzone)
    def get_controller_input(self):
        pygame.event.pump()
        if self.joystick.get_button(self.button_map['start']):
            if not getattr(self,'start_pressed',False):self.armed=not self.armed;print(f"\n[{'ARMED' if self.armed else'DISARMED'}]")
            self.start_pressed=True
        else: self.start_pressed=False
        if self.joystick.get_button(self.button_map['a']):
            if not getattr(self,'a_pressed',False):self.stabilization_enabled=not self.stabilization_enabled;print(f"Stabilization:{'ON'if self.stabilization_enabled else'OFF'}")
            self.a_pressed=True
        else: self.a_pressed=False
        if self.joystick.get_button(self.button_map['b']):
            if not getattr(self,'b_pressed',False):self.light_on=not self.light_on;print(f"Light:{'ON'if self.light_on else'OFF'}")
            self.b_pressed=True
        else: self.b_pressed=False
        if not self.armed:return 0.0,0.0,0.0,0.0,self.light_on
        surge=-self._apply_deadzone(self.joystick.get_axis(self.axis_map['ly']))*self.translation_scale;strafe=self._apply_deadzone(self.joystick.get_axis(self.axis_map['lx']))*self.translation_scale;yaw_input=self._apply_deadzone(self.joystick.get_axis(self.axis_map['rx']))*self.rotation_scale;heave=(self.joystick.get_axis(self.axis_map['rt'])-self.joystick.get_axis(self.axis_map['lt']))/2.0*self.translation_scale;return surge,strafe,heave,yaw_input,self.light_on
    def pid_control(self,axis,error):g,s=self.pid_gains[axis],self.pid_states[axis];s.integral=np.clip(s.integral+error*self.dt,-10,10);derivative=(error-s.prev_error)/self.dt;s.prev_error=error;return g.kp*error+g.ki*s.integral+g.kd*derivative
    def calculate_stabilization_forces(self,current_attitude):
        current_rpy=np.degrees(self._quaternion_to_euler(current_attitude));target_rpy=np.degrees(self._quaternion_to_euler(self.target_attitude));error=target_rpy-current_rpy
        if error[2]>180:error[2]-=360
        elif error[2]<-180:error[2]+=360
        corrections={ax:self.pid_control(ax,e)if abs(e)>self.angle_deadzone[ax]else 0 for ax,e in zip(['roll','pitch','yaw'],error)};forces=np.zeros(6);forces[0]=forces[3]=corrections['yaw'];forces[1]=forces[2]=-corrections['yaw'];forces[4]=-corrections['roll']+corrections['pitch'];forces[5]=corrections['roll']+corrections['pitch'];return np.clip(forces*self.stabilization_force_multiplier,-self.stabilization_force_max,self.stabilization_force_max)
    def calculate_thruster_forces(self,surge,strafe,heave,yaw,stabilization_forces):manual_forces=np.array([surge-strafe+yaw,surge+strafe-yaw,-surge-strafe-yaw,-surge+strafe+yaw,heave,heave])*self.MAX_FORCE;return np.clip(manual_forces+(stabilization_forces if self.stabilization_enabled else 0),self.MIN_FORCE,self.MAX_FORCE)
    def run_controller(self):
        print("🚀 Starting controller loop. Press START to arm.")
        try:
            while True:
                if(time.time()-self.last_time)>=self.dt:
                    self.last_time=time.time()
                    current_attitude=self.attitude_client.get_latest_quaternion()if self.is_real_sub_mode else self.command_interface.get_sensor_data()['imu_quaternion']
                    if current_attitude is None:time.sleep(0.1);continue
                    surge,strafe,heave,yaw_input,light_on=self.get_controller_input()
                    stabilization_forces=self.calculate_stabilization_forces(current_attitude)
                    final_forces=self.calculate_thruster_forces(surge,strafe,heave,yaw_input,stabilization_forces)

                    ### MODIFIED: Store forces for visualizer ###
                    self.latest_forces = final_forces if self.armed else np.zeros(6)

                    if self.armed:
                        if self.is_real_sub_mode:self.command_interface.apply_ctrl(self.latest_forces,light_on)
                        else:self.command_interface.apply_ctrl(self.latest_forces)
                    else:
                        if self.is_real_sub_mode:self.command_interface.apply_ctrl(self.latest_forces,light_on)
                        else:self.command_interface.apply_ctrl(self.latest_forces)
        except KeyboardInterrupt:print("\nController loop stopped.")
        finally:
            print("\nDisarming...");self.command_interface.stop();pygame.quit();print("Cleanup complete.")

# ============================================================================
# MAIN EXECUTION (MODIFIED)
# ============================================================================
def main():
    parser=argparse.ArgumentParser(description="BlueROV2 Controller")
    parser.add_argument("--sim_ip",default="127.0.0.1",help="Sim server IP")
    parser.add_argument("--sim_port",type=int,default=60001,help="Sim server port")
    parser.add_argument("--attitude_ip",default="192.168.2.11",help="Attitude server IP")
    parser.add_argument("--attitude_port",type=int,default=9999,help="Attitude server port")
    parser.add_argument("--thruster_ip",default="192.168.2.11",help="Thruster UDP server IP")
    parser.add_argument("--thruster_port",type=int,default=5005,help="Thruster UDP server port")
    parser.add_argument("--visualize",action='store_true',help="Show live attitude plot (real sub mode only)")
    args=parser.parse_args()
    print("="*40+"\n  BlueROV2 Controller Interface\n"+"="*40)
    if args.visualize and not MATPLOTLIB_AVAILABLE:print("[WARNING] --visualize passed, but matplotlib is not installed."),;args.visualize=False
    choice=input("Select mode:\n  1) Simulation\n  2) Real Submarine\nEnter (1 or 2): ")
    if choice not in['1','2']:print("Invalid choice.");return

    command_interface=None;attitude_client=None;visualizer_thread=None
    try:
        controller = None # Define controller in this scope
        if choice=='1':
            print("\nInitializing for SIMULATION mode...")
            command_interface=AUV(server_ip=args.sim_ip,server_port=args.sim_port)
            controller=BlueROVController(command_interface=command_interface,attitude_client=None)
        elif choice=='2':
            print("\nInitializing for REAL SUBMARINE mode...")
            attitude_client=AttitudeClient(host=args.attitude_ip,port=args.attitude_port)
            if not attitude_client.start():raise ConnectionError("Could not connect to attitude server.")
            command_interface=UDPThrusterClient(host=args.thruster_ip,port=args.thruster_port)

            ### MODIFIED: Controller is created before the visualizer ###
            controller=BlueROVController(command_interface=command_interface,attitude_client=attitude_client)

            if args.visualize:
                print("--> Visualizer enabled. Launching plot window...")
                visualizer=AttitudeVisualizer(attitude_client=attitude_client, controller=controller)
                visualizer_thread=threading.Thread(target=visualizer.run,daemon=True)
                visualizer_thread.start()

        if controller:
            controller.run_controller()

    except(RuntimeError,ConnectionError,TimeoutError)as e:print(f"\n[FATAL ERROR] {e}")
    finally:
        print("\nShutting down clients...")
        if attitude_client:attitude_client.stop()
        if command_interface and hasattr(command_interface,'stop'):command_interface.stop()
        if visualizer_thread:print("Closing visualizer...");plt.close('all');visualizer_thread.join(timeout=1)

if __name__ == "__main__":
    main()
