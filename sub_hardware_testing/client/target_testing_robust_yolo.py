#!/usr/bin/env python3
"""
Target Testing robust yolo
"""

from dataclasses import dataclass
from enum import Enum, auto
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
import torch  # PyTorch for YOLO
from ultralytics import YOLO  # The YOLO library

# Use a non-interactive backend for Matplotlib
import matplotlib
matplotlib.use("Agg")

class MsgHeader(Enum):
    ERROR = (0,); NO_ERROR = (1,); HEARTBEAT = (2,); GET_MODEL_INFO = (3,)
    GET_SENSORDATA = 4; GET_RGB_IMAGE = (5,); GET_MASKED_IMAGE = (6,)
    APPLY_CTRL = 7; STEP_SIM = (8,); RESET = 9

get_sensor_data_cleint_recv_bytes = 20
ACK = 21

@dataclass
class BoundingBox:
    x: float = 0.0; y: float = 0.0; width: float = 0.0; height: float = 0.0
    @property
    def center(self) -> Tuple[float, float]: return self.x + self.width/2, self.y + self.height/2
    @property
    def area(self) -> float: return self.width * self.height
    @property
    def is_valid(self) -> bool: return self.width > 0 and self.height > 0

# States for the "Orbit and Re-advance" strategy
class ChaseStrategyState(Enum):
    IDLE = auto(); SEARCHING = auto(); ADVANCING = auto()
    ORBITING = auto(); CELEBRATING = auto()

class ChaseAndCircleStrategy:
    """
    Implements a robust "Orbit and Re-advance" strategy. It includes a grace
    period for transient bounding box losses and an active searching pattern.
    """
    def __init__(self, camera_width: int, camera_height: int):
        self.state = ChaseStrategyState.IDLE
        self.camera_center_x = camera_width / 2
        # MODIFIED: Set a new vertical target for the bbox (lower part of the screen)
        self.camera_target_y = camera_height * 0.75
        print(f"Chase strategy initialized for {camera_width}x{camera_height} camera.")
        print(f"Vertical target set to y={self.camera_target_y:.0f}")

        # --- Control Gains ---
        self.centering_kp = 0.0003
        self.heaving_kp = 0.02
        self.advance_surge = 0.8
        self.orbit_strafe_command = 0.8
        self.max_yaw_error_for_strafe = 80.0
        # NEW: Active search parameters
        self.search_yaw_command = 0.03  # Yaw rate during active search
        self.search_pattern_interval_s = 0.5  # Time to yaw in one direction

        # --- State Thresholds ---
        self.orbit_entry_thresh_area = camera_width * camera_height * 0.05
        self.orbit_exit_thresh_area = self.orbit_entry_thresh_area * 0.75
        # NEW: Confidence threshold for acquiring a new target
        self.search_confidence_threshold = 0.5

        # --- Timers and Robustness ---
        self.orbit_duration_for_win_s = 2
        self.celebration_time_s = 5.0
        self.state_timer = 0.0

        self.lost_target_grace_period_s = 0.5
        self.lost_target_timer = 0.0
        # NEW: Timers for active searching
        self.time_since_target_lost = 0.0
        self.search_yaw_timer = 0.0
        self.search_direction = 1 # 1 for right, -1 for left


    def update(self, bbox: BoundingBox, confidence: float, dt: float) -> Tuple[float, float, float, float, bool]:
        """
        Calculates control commands based on the current state and target.
        """
        surge, strafe, heave, yaw = 0.0, 0.0, 0.0, 0.0
        flash_lights = False

        # --- Handle Target Validity with Grace Period ---
        if bbox.is_valid:
            self.lost_target_timer = 0.0
        else:
            self.lost_target_timer += dt

        # --- Global Target Lost Condition ---
        if self.lost_target_timer > self.lost_target_grace_period_s:
            if self.state != ChaseStrategyState.SEARCHING:
                print(f"Target lost for over {self.lost_target_grace_period_s:.1f}s. Resetting to SEARCHING mode.")
                self.state = ChaseStrategyState.SEARCHING
                self.time_since_target_lost = 0.0 # Reset search timer

        # --- State Machine ---
        # State: IDLE / SEARCHING
        if self.state in [ChaseStrategyState.IDLE, ChaseStrategyState.SEARCHING]:
            self.time_since_target_lost += dt
            if self.time_since_target_lost > 1.5:
                self.search_yaw_timer += dt
                if self.search_yaw_timer > self.search_pattern_interval_s:
                    self.search_direction *= -1
                    self.search_yaw_timer = 0.0
                yaw = self.search_yaw_command * self.search_direction
                print(f"Actively searching... Yawing {'RIGHT' if self.search_direction > 0 else 'LEFT'}", end='\r')

            if bbox.is_valid and confidence > self.search_confidence_threshold:
                print(f"\nTarget acquired with confidence {confidence:.2f}. Advancing.")
                self.state = ChaseStrategyState.ADVANCING
                self.time_since_target_lost = 0.0

        # State: ADVANCING
        elif self.state == ChaseStrategyState.ADVANCING:
            surge = self.advance_surge
            strafe = 0.0
            
            if bbox.is_valid:
                err_x = self.camera_center_x - bbox.center[0]
                # MODIFIED: Use the new vertical target
                err_y = self.camera_target_y - bbox.center[1]
                yaw = self.centering_kp * err_x
                # MODIFIED: Corrected heave logic to be consistent (negative sign)
                heave = self.heaving_kp * err_y

                if bbox.area > self.orbit_entry_thresh_area:
                    print(f"Target at optimal range. Entering dynamic ORBITING mode.")
                    self.state = ChaseStrategyState.ORBITING
                    self.state_timer = 0.0
        
        # State: ORBITING
        elif self.state == ChaseStrategyState.ORBITING:
            surge = 0.0
            
            if bbox.is_valid:
                err_x = self.camera_center_x - bbox.center[0]
                # MODIFIED: Use the new vertical target to actively heave while orbiting
                err_y = self.camera_target_y - bbox.center[1]
                yaw = self.centering_kp * err_x
                heave = self.heaving_kp * err_y # Heave to keep target vertically aligned
                strafe_scale = max(0.0, 1.0 - (abs(err_x) / self.max_yaw_error_for_strafe))
                strafe = self.orbit_strafe_command * strafe_scale

                if bbox.area < self.orbit_exit_thresh_area:
                    print("Target has moved too far away. Re-engaging with ADVANCING.")
                    self.state = ChaseStrategyState.ADVANCING
            else:
                # Bbox is lost, but we're in the grace period. Continue orbiting.
                strafe = self.orbit_strafe_command
            
            self.state_timer += dt
            if self.state_timer > self.orbit_duration_for_win_s:
                print("Successfully orbited target. MISSION ACCOMPLISHED!")
                self.state = ChaseStrategyState.CELEBRATING
                self.state_timer = 0.0

        # State: CELEBRATING
        elif self.state == ChaseStrategyState.CELEBRATING:
            self.state_timer += dt
            flash_lights = (int(self.state_timer * 4) % 2) == 0
            if self.state_timer > self.celebration_time_s:
                print("Celebration over. Returning to SEARCHING.")
                self.state = ChaseStrategyState.SEARCHING
                self.time_since_target_lost = 0.0

        return surge, strafe, heave, yaw, flash_lights

# ... (The rest of the script is unchanged) ...
class SubmarineDataReceiverThread(threading.Thread):
    def __init__(self, video_port=10001, imu_port=10002):
        super().__init__(daemon=True, name="SubmarineDataReceiver")
        self.video_listen_addr = ("", video_port); self.imu_listen_addr = ("", imu_port)
        self.latest_frame = None; self.latest_imu_data = None
        self.frame_lock = threading.Lock(); self.stop_event = threading.Event()
    def _video_loop(self):
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(self.video_listen_addr); s.settimeout(1.0); b={}
        while not self.stop_event.is_set():
            try:
                d,a=s.recvfrom(65536)
                if len(d)<6: continue
                fid,cs,cid=struct.unpack("!HHH",d[:6])
                if fid not in b: b[fid]={}
                b[fid][cid]=d[6:]
                if len(b[fid])==cs:
                    jpeg=b"".join(v for k,v in sorted(b[fid].items()))
                    fr=cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
                    with self.frame_lock: self.latest_frame=fr
                    del b[fid]
            except (socket.timeout, Exception): pass
        s.close()
    def _imu_loop(self):
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(self.imu_listen_addr); s.settimeout(1.0)
        while not self.stop_event.is_set():
            try:
                d,a=s.recvfrom(1024); self.latest_imu_data=json.loads(d.decode("utf-8"))
            except (socket.timeout, Exception): self.latest_imu_data=None
        s.close()
    def run(self):
        vt=threading.Thread(target=self._video_loop,daemon=True); it=threading.Thread(target=self._imu_loop,daemon=True)
        vt.start(); it.start(); vt.join(); it.join()
    def stop(self): self.stop_event.set()

class CombinedVisualizer:
    def __init__(self, data_receiver):
        self.data_receiver=data_receiver; self.screen_width=1280; self.camera_width=640
        self.plot_width=self.screen_width-self.camera_width; self.screen_height=480
        self.latest_bbox=None; self.screen=pygame.display.set_mode((self.screen_width,self.screen_height))
        pygame.display.set_caption("Submarine Control Dashboard"); self.font=pygame.font.SysFont("monospace",16)
        self.pyramid_verts_base=np.array([[-.6,0,0],[.2,-.4,-.4],[.2,.4,-.4],[.2,.4,.4],[.2,-.4,.4],])
        self.pyramid_faces_indices=[[0,1,2],[0,2,3],[0,3,4],[0,4,1],[1,4,3,2],]
        self.pyramid_face_colors=["cyan","cyan","cyan","cyan","red"]
        plot_dpi=100; fig_size=(self.plot_width/plot_dpi,self.screen_height/plot_dpi)
        self.fig=plt.figure(figsize=fig_size,dpi=plot_dpi); self.ax_3d=self.fig.add_subplot(1,1,1,projection="3d")
        self.pyramid_collection=Poly3DCollection([],facecolors=self.pyramid_face_colors,linewidths=.8,edgecolors="k",alpha=.6)
        self.setup_3d_plot(); self.rotation_fix=np.eye(3)
        self.hud_data={"armed":False,"light_on":False,"stabilization":True,"yaw_kp":0.,"yaw_kp_real":0.,"trans_scale":.8,"chase_mode":"INACTIVE",}
    def setup_3d_plot(self):
        self.ax_3d.set_xlabel("X (Fwd)");self.ax_3d.set_ylabel("Y (Right)");self.ax_3d.set_zlabel("Z (Up)")
        self.ax_3d.set_xlim([-1,1]);self.ax_3d.set_ylim([-1,1]);self.ax_3d.set_zlim([-1,1])
        try: self.ax_3d.set_box_aspect([1,1,1])
        except AttributeError: pass
        self.ax_3d.view_init(elev=15,azim=90)
        colors=["red","green","blue"];labels=["X (Roll)","Y (Pitch)","Z (Yaw)"]
        self.frame_lines=[self.ax_3d.plot([],[],[],c=c,lw=3,label=l)[0] for c,l in zip(colors,labels)]
        self.ax_3d.legend();self.ax_3d.add_collection3d(self.pyramid_collection);self.fig.tight_layout(pad=0)
    def _quaternion_to_rotation_matrix(self,q):
        q0,q1,q2,q3=q
        return np.array([[q0**2+q1**2-q2**2-q3**2,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2)],[2*(q1*q2+q0*q3),q0**2-q1**2+q2**2-q3**2,2*(q2*q3-q0*q1)],[2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0**2-q1**2-q2**2+q3**2],])
    def update(self,imu_override:Optional[np.ndarray]=None):
        self.screen.fill((25,25,25))
        with self.data_receiver.frame_lock: frame=self.data_receiver.latest_frame
        if frame is not None:
            if self.latest_bbox and self.latest_bbox.is_valid:
                pt1=(int(self.latest_bbox.x),int(self.latest_bbox.y))
                pt2=(int(self.latest_bbox.x+self.latest_bbox.width),int(self.latest_bbox.y+self.latest_bbox.height))
                cv2.rectangle(frame,pt1,pt2,(0,255,0),2)
            frame_rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            camera_surface=pygame.surfarray.make_surface(np.rot90(frame_rgb))
            self.screen.blit(camera_surface,(0,0))
        y_pos=10
        chase_status=self.hud_data.get("chase_mode","INACTIVE");chase_color=(255,165,0)if chase_status!="INACTIVE"else(100,100,100)
        self.screen.blit(self.font.render(f"CHASE: {chase_status}",1,chase_color),(y_pos,10))
        armed_color=(255,0,0)if self.hud_data["armed"]else(0,255,0)
        self.screen.blit(self.font.render(f"ARMED: {self.hud_data['armed']}",1,armed_color),(y_pos,30))
        light_text="ON"if self.hud_data["light_on"]else"OFF"
        self.screen.blit(self.font.render(f"LIGHT: {light_text}",1,(255,255,0)),(y_pos,50))
        imu_data=self.data_receiver.latest_imu_data
        if imu_data and"quaternion"in imu_data and not np.any(np.isnan(imu_data["quaternion"])):
            q=np.array(imu_data["quaternion"]);R=self.rotation_fix@self._quaternion_to_rotation_matrix(q)
            x_ax,y_ax,z_ax=R[:,0],R[:,1],R[:,2]
            self.frame_lines[0].set_data([0,x_ax[0]],[0,x_ax[1]]);self.frame_lines[0].set_3d_properties([0,x_ax[2]])
            self.frame_lines[1].set_data([0,y_ax[0]],[0,y_ax[1]]);self.frame_lines[1].set_3d_properties([0,y_ax[2]])
            self.frame_lines[2].set_data([0,z_ax[0]],[0,z_ax[1]]);self.frame_lines[2].set_3d_properties([0,z_ax[2]])
            rotated_verts=self.pyramid_verts_base@R.T
            pyramid_faces=[[rotated_verts[i]for i in face]for face in self.pyramid_faces_indices]
            self.pyramid_collection.set_verts(pyramid_faces);self.ax_3d.set_title("3D Orientation (Live)")
        else:self.pyramid_collection.set_verts([]);self.ax_3d.set_title("3D Orientation (Waiting...)")
        self.fig.canvas.draw()
        plot_surface=pygame.image.frombuffer(self.fig.canvas.buffer_rgba(),self.fig.canvas.get_width_height(),"RGBA")
        self.screen.blit(plot_surface,(self.camera_width,0));pygame.display.flip()

class UDPClientProtocol(asyncio.DatagramProtocol):
    def __init__(self, on_con_lost: asyncio.Future):
        self.transport=None; self.on_con_lost=on_con_lost; self.recv_queue=asyncio.Queue()
    def connection_made(self, transport: asyncio.DatagramTransport): self.transport=transport
    def datagram_received(self, data: bytes, addr: Tuple[str, int]): self.recv_queue.put_nowait(data)
    def error_received(self, exc: Exception): print(f"UDP connection error: {exc}")
    def connection_lost(self, exc: Exception):
        if not self.on_con_lost.done(): self.on_con_lost.set_result(True)

class AsyncAUV:
    def __init__(self,server_ip="127.0.0.1",server_port=60001):
        self.server_ip=server_ip; self.server_port=server_port; self.transport=None
        self.protocol=None; self.connection_lock=asyncio.Lock(); self.connected=False
    async def _ensure_transport(self):
        async with self.connection_lock:
            if self.transport and not self.transport.is_closing(): return
            try:
                if self.transport:self.transport.close()
                loop=asyncio.get_running_loop()
                self.protocol=UDPClientProtocol(loop.create_future())
                remote_addr=(self.server_ip,self.server_port)
                self.transport,_=await loop.create_datagram_endpoint(lambda:self.protocol,remote_addr=remote_addr)
                self.connected=True; print(f"AsyncAUV: UDP transport ready for {self.server_ip}:{self.server_port}")
            except Exception as e:
                self.connected=False; print(f"AsyncAUV: UDP transport creation failed: {e}"); raise
    async def _send_command(self,data:bytes)->Optional[bytes]:
        max_retries=3
        for attempt in range(max_retries):
            try:
                await self._ensure_transport()
                while not self.protocol.recv_queue.empty(): self.protocol.recv_queue.get_nowait()
                self.transport.sendto(data)
                return await asyncio.wait_for(self.protocol.recv_queue.get(),timeout=2.)
            except asyncio.TimeoutError:
                if attempt==max_retries-1: print(f"AsyncAUV: Command timed out after {max_retries} attempts")
            except Exception as e:
                print(f"AsyncAUV: Unexpected error: {e}"); self.connected=False; await asyncio.sleep(.1)
        return None
    async def get_sensor_data(self)->Optional[dict]:
        data=struct.pack("<f",float(MsgHeader.GET_SENSORDATA.value)); response=await self._send_command(data)
        if response and len(response)>=get_sensor_data_cleint_recv_bytes:
            try: values=struct.unpack("<5f",response); return{"imu_quaternion":np.array(values[:4]),"time":values[4]}
            except struct.error: return None
        return None
    async def apply_ctrl(self,forces:np.ndarray,num_steps:int=1)->Optional[dict]:
        if len(forces)!=6: raise ValueError("Forces array must have 6 elements")
        data=struct.pack("<8f",float(MsgHeader.APPLY_CTRL.value),float(num_steps),*forces); response=await self._send_command(data)
        if response and response[0]==0 and len(response)>=ACK:
            try: values=struct.unpack("<5f",response[1:]); return{"imu_quaternion":np.array(values[:4]),"time":values[4]}
            except struct.error: return None
        return None
    async def reset(self)->bool:
        pose=np.array([10.25,6.71,0,1,0,0,0]); vel=np.zeros(3); ang_vel=np.zeros(3)
        data=struct.pack("<15f",float(MsgHeader.RESET.value),0.,*pose,*vel,*ang_vel); response=await self._send_command(data)
        return response is not None
    async def close(self):
        if self.transport:self.transport.close()
        self.connected=False

class RealSubmarineClient:
    def __init__(self,server_ip,control_port=10000):self.server_address=(server_ip,control_port);self.control_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.control_socket.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,8192);print(f"RealSubClient: Ready for {self.server_address}");self.NEUTRAL_PULSE=1500;self.LIGHT_OFF=1100;self.LIGHT_ON=1900;self.AMP=300;self.LOOP_DURATION=.02
    def _to_pwm(self,nf):return int(self.NEUTRAL_PULSE+nf*self.AMP)
    def _send_packet(self,cmd_dict):
        try:message=(json.dumps(cmd_dict)+"\n").encode("utf-8");self.control_socket.sendto(message,self.server_address)
        except Exception as e:print(f"RealSubClient Error: {e}",file=sys.stderr)
    def send_control(self,armed,light_on,forces):
        forces=np.zeros(6)if not armed else forces;clipped_forces=np.clip(forces/100.,-1.,1.);thruster_pulses=[self._to_pwm(v)for v in clipped_forces];cmd={"command":"control","thruster_pulses":thruster_pulses,"light_pulse":self.LIGHT_ON if light_on else self.LIGHT_OFF,"duration":self.LOOP_DURATION};self._send_packet(cmd)
    def send_reset_command(self):print(f"Sending 'reset_orientation' command to {self.server_address}...");self._send_packet({"command":"reset_orientation"})
    def shutdown(self):print("RealSubClient: Shutting down.");self.send_control(False,False,np.zeros(6));time.sleep(.1);self.control_socket.close()

@dataclass
class PIDGains: kp:float;ki:float;kd:float
@dataclass
class PIDState: prev_error:float=0.;integral:float=0.

class BlueROVController:
    def __init__(self, args):
        self.auv=AsyncAUV(args.ip,args.port); self.direction=args.direction; self.args=args
        pygame.init()
        self.controller=None
        if self.direction is None and not self.args.chase_target:
            pygame.joystick.init()
            if pygame.joystick.get_count()==0: raise RuntimeError("No gamepad. Use --direction or --chase_target.")
            self.controller=pygame.joystick.Joystick(0); self.controller.init(); print(f"Gamepad: {self.controller.get_name()}")
        else: print("Running in an automated mode. Gamepad input is disabled.")
        self.real_sub_client=RealSubmarineClient(args.sub_ip)if args.sub_ip else None
        self.data_receiver_thread=SubmarineDataReceiverThread()
        self.armed=False;self.light_on=False;self.stabilization_enabled=True
        self.minForce=np.full(6,-100);self.maxForce=np.full(6,100)
        self.manual_force_scale=100.;self.translation_scale=0.8;self.rotation_scale=0.4
        self.yolo_model=None
        if self.args.chase_target:
            if not self.args.weights: raise ValueError("Chase mode requires a YOLO weights file. Use --weights path/to/your.pt")
            print(f"Loading YOLO model from {self.args.weights}..."); self.yolo_model=YOLO(self.args.weights); print("YOLO model loaded successfully.")
            self.chase_strategy=ChaseAndCircleStrategy(camera_width=640,camera_height=480)
        self.pid_gains={"yaw_sim":PIDGains(0.1,0,0),"yaw_real":PIDGains(10550.,0.,0.)}
        self.pid_states={"yaw_sim":PIDState(),"yaw_real":PIDState()}; self.target_attitude=np.array([1.,0.,0.,0.])
        self.LEFT_STICK_X=0;self.LEFT_STICK_Y=1;self.RIGHT_STICK_X=2;self.RIGHT_STICK_Y=3
        self.BTN_A=0;self.BTN_B=1;self.BTN_X=2;self.BTN_Y=3;self.BTN_MENU=6
        self.DEADZONE=0.1;self.dt=0.02;self.loop_times=[];self.network_times=[]
    def apply_deadzone(self, v: float) -> float:
        if abs(v)<self.DEADZONE: return 0.
        return np.sign(v)*(abs(v)-self.DEADZONE)/(1.-self.DEADZONE)

    def get_controller_input(self)->Tuple[float,float,float,float]:
        if self.controller is None or not self.armed: return 0.,0.,0.,0.
        pygame.event.pump()
        surge=-self.apply_deadzone(self.controller.get_axis(self.LEFT_STICK_Y))
        strafe=self.apply_deadzone(self.controller.get_axis(self.LEFT_STICK_X))
        yaw_cmd=self.apply_deadzone(self.controller.get_axis(self.RIGHT_STICK_X))
        heave=-self.apply_deadzone(self.controller.get_axis(self.RIGHT_STICK_Y))
        return surge,strafe,heave,yaw_cmd

    def calculate_thruster_forces(self,surge,strafe,heave,yaw_cmd,system_type:str)->np.ndarray:
        base_forces=np.array([surge-strafe+yaw_cmd,surge+strafe-yaw_cmd,-surge-strafe-yaw_cmd,-surge+strafe+yaw_cmd,heave,heave])
        return np.clip(base_forces,self.minForce,self.maxForce)

    async def run_controller_async(self):
        if self.direction or self.args.chase_target:
            self.armed=True;mode="CHASE"if self.args.chase_target else f"'{self.direction}'"
            print(f"🚀 Starting ASYNC controller in {mode} mode. Auto-arming vehicle.")
        else:print("🚀 Starting ASYNC controller loop. Press MENU to arm.")
        self.visualizer=CombinedVisualizer(self.data_receiver_thread);self.data_receiver_thread.start()
        shutdown=False;cmdstarttime=time.time()
        try:
            while not shutdown:
                loop_start=time.time();runtime=self.args.runtime
                if runtime>0 and(time.time()-cmdstarttime>runtime):
                    print(f"\nRuntime of {runtime}s complete. Shutting down.");shutdown=True
                for e in pygame.event.get():
                    if e.type==pygame.QUIT:shutdown=True;break
                    if self.controller and e.type==pygame.JOYBUTTONDOWN:
                        if e.button==self.BTN_MENU:self.armed=not self.armed;print(f"\n[{'ARMED'if self.armed else'DISARMED'}]")
                        elif e.button==self.BTN_B:self.light_on=not self.light_on
                if shutdown:break
                surge,strafe,heave,yaw_cmd=0.,0.,0.,0.
                if self.chase_strategy and self.armed:
                    with self.data_receiver_thread.frame_lock:frame=self.data_receiver_thread.latest_frame
                    bbox_to_use=BoundingBox()
                    highest_conf = 0.0 # Initialize with zero confidence
                    if frame is not None and self.yolo_model is not None:
                        results=self.yolo_model(frame,verbose=False)
                        best_box_coords=None
                        for box in results[0].boxes:
                            if box.conf[0]>highest_conf:
                                highest_conf=box.conf[0]
                                best_box_coords=box.xywh[0]
                        if best_box_coords is not None:
                            cx,cy,w,h=best_box_coords
                            x=cx-w/2;y=cy-h/2
                            bbox_to_use=BoundingBox(x=x.item(),y=y.item(),width=w.item(),height=h.item())
                    self.visualizer.latest_bbox=bbox_to_use
                    s_surge,s_strafe,s_heave,s_yaw,flash=self.chase_strategy.update(bbox_to_use, highest_conf, self.dt)
                    surge,strafe,heave,yaw_cmd=s_surge,s_strafe,s_heave,s_yaw
                    self.light_on=flash
                elif self.direction and self.armed:
                    if self.direction=="forward":surge=self.args.fwcmd/self.manual_force_scale
                    elif self.direction=="backward":surge=-self.args.fwcmd/self.manual_force_scale
                else:surge,strafe,heave,yaw_cmd=self.get_controller_input()
                sim_f=self.calculate_thruster_forces(surge*self.translation_scale*self.manual_force_scale,strafe*self.translation_scale*self.manual_force_scale,heave*self.translation_scale*self.manual_force_scale,yaw_cmd*self.rotation_scale*self.manual_force_scale,"sim")
                tasks=[self.auv.apply_ctrl(sim_f if self.armed else np.zeros(6),self.args.num_steps)]
                if self.real_sub_client:
                    loop=asyncio.get_event_loop();real_f=sim_f
                    tasks.append(loop.run_in_executor(None,self.real_sub_client.send_control,self.armed,self.light_on,real_f))
                await asyncio.gather(*tasks,return_exceptions=True)
                chase_mode_status=self.chase_strategy.state.name if self.chase_strategy else"INACTIVE"
                self.visualizer.hud_data.update({"armed":self.armed,"light_on":self.light_on,"stabilization":self.stabilization_enabled,"yaw_kp":self.pid_gains["yaw_sim"].kp,"yaw_kp_real":self.pid_gains["yaw_real"].kp,"trans_scale":self.translation_scale,"chase_mode":chase_mode_status,})
                self.visualizer.update()
                loop_time=time.time()-loop_start
                await asyncio.sleep(max(0,self.dt-loop_time))
        finally:
            print("Disarming and stopping...")
            self.data_receiver_thread.stop();self.data_receiver_thread.join(timeout=1);plt.close("all")
            await self.auv.apply_ctrl(np.zeros(6),1);await self.auv.close()
            if self.real_sub_client:self.real_sub_client.shutdown()
            pygame.quit()
    def run(self):asyncio.run(self.run_controller_async())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Async BlueROV2 Controller with YOLO detection")
    parser.add_argument("--ip",type=str,default="127.0.0.1",help="IP of the simulator.")
    parser.add_argument("--port",type=int,default=60001,help="Port of the simulator.")
    parser.add_argument("--num_steps",type=int,default=5,help="Simulator physics steps per cycle.")
    parser.add_argument("--sub_ip",type=str,default=None,help="IP of real submarine.")
    parser.add_argument("--fwcmd",type=float,default=25.,help="Force to apply for --direction.")
    parser.add_argument("--runtime",type=float,default=0.,help="Time duration to run (0 for infinite).")
    parser.add_argument("--direction",type=str,choices=["forward","backward"],default=None,help="Constant force, bypasses controller.")
    parser.add_argument("--chase_target",action="store_true",help="Enable autonomous chase strategy using YOLO.")
    parser.add_argument("--weights",type=str,default="../unity.pt",help="Path to the YOLO model .pt weights file.")
    args = parser.parse_args()
    if args.chase_target and args.direction:
        print("[FATAL ERROR] --chase_target and --direction cannot be used at the same time.",file=sys.stderr);sys.exit(1)
    try:
        controller = BlueROVController(args)
        controller.run()
    except Exception as e:
        print(f"\n[FATAL ERROR] Could not start: {e}",file=sys.stderr);pygame.quit();sys.exit(1)
