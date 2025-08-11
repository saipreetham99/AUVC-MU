#!/usr/bin/env python3
"""
Final Integrated Testing Controller (Simulation).
- Receives UDP video stream from the Pi camera streamer.
- Runs YOLO detection to find the target.
- Implements the chase-and-circle strategy.
- Sends PWM control signals to the Pi thruster server.
- Sends abstract force commands to the simulator.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple, Dict
from collections import defaultdict
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
import torch
from ultralytics import YOLO
import supervision as sv

# Use a non-interactive backend for Matplotlib
import matplotlib
matplotlib.use("Agg")

# --- Constants for Real Submarine Control ---
NEUTRAL_PULSE = 1500
AMP = 200  # Determines max/min pulse from force. 1500 +/- 400 -> [1100, 1900]
LIGHT_OFF_PULSE = 1100
LIGHT_ON_PULSE = 1900
VIDEO_STREAM_PORT = 5004 # Port for receiving camera feed from the Pi

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

class ChaseStrategyState(Enum):
    IDLE = auto(); SEARCHING = auto(); CENTERING = auto()
    ADVANCING = auto(); CIRCLING = auto(); CELEBRATING = auto()

class ChaseAndCircleStrategy:
    def __init__(self, camera_width: int, camera_height: int):
        self.state = ChaseStrategyState.IDLE
        self.camera_center_x = camera_width / 2
        self.camera_center_y = camera_height / 2
        print(f"Chase strategy initialized for {camera_width}x{camera_height} camera.")
        self.centering_kp = 0.003
        self.advance_surge = 0.8
        self.circle_strafe = 0.5
        self.circle_distance_thresh_area = camera_width * camera_height * 0.03
        self.scan_time_s = 8.0
        self.celebration_time_s = 5.0
        self.state_timer = 0.0

    def update(self, bbox: BoundingBox, dt: float) -> Tuple[float, float, float, float, bool]:
        surge, strafe, heave, yaw = 0.0, 0.0, 0.0, 0.0; flash_lights = False
        if not bbox.is_valid:
            if self.state != ChaseStrategyState.SEARCHING:
                print("Target lost. Entering SEARCHING mode.")
                self.state = ChaseStrategyState.SEARCHING
            return 0.0, 0.0, 0.0, 0.0, False
        if self.state in [ChaseStrategyState.IDLE, ChaseStrategyState.SEARCHING]:
            print("Target acquired. Advancing and Centering.")
            self.state = ChaseStrategyState.ADVANCING
        if self.state == ChaseStrategyState.ADVANCING:
            surge = self.advance_surge
            err_x = self.camera_center_x - bbox.center[0]
            err_y = self.camera_center_y - bbox.center[1]
            yaw = self.centering_kp * err_x
            heave = self.centering_kp * err_y
            if bbox.area > self.circle_distance_thresh_area:
                print(f"Circling distance reached (Area: {bbox.area:.0f}).")
                self.state_timer = 0.0
                self.state = ChaseStrategyState.CIRCLING
        elif self.state == ChaseStrategyState.CIRCLING:
            err_x = self.camera_center_x - bbox.center[0]
            err_y = self.camera_center_y - bbox.center[1]
            yaw = self.centering_kp * err_x
            heave = self.centering_kp * err_y
            strafe = self.circle_strafe
            self.state_timer += dt
            if self.state_timer > self.scan_time_s:
                print("Scan complete! MISSION ACCOMPLISHED!")
                self.state_timer = 0.0
                self.state = ChaseStrategyState.CELEBRATING
        elif self.state == ChaseStrategyState.CELEBRATING:
            self.state_timer += dt
            flash_lights = (int(self.state_timer * 4) % 2) == 0
            if self.state_timer > self.celebration_time_s:
                print("Celebration over. Returning to SEARCHING.")
                self.state = ChaseStrategyState.SEARCHING
        return surge, strafe, heave, yaw, flash_lights

# --- MODIFICATION: This thread now uses the logic from your desktop_vision_processor.py ---
class SubmarineDataReceiverThread(threading.Thread):
    def __init__(self, video_port=VIDEO_STREAM_PORT, imu_port=10002):
        super().__init__(daemon=True, name="SubmarineDataReceiver")
        self.video_listen_addr = ("", video_port)
        self.imu_listen_addr = ("", imu_port)
        self.latest_frame = None
        self.latest_imu_data = None
        self.frame_lock = threading.Lock()
        self.stop_event = threading.Event()

    def _video_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(self.video_listen_addr)
        s.settimeout(1.0)
        buffers: Dict[int, Dict] = defaultdict(lambda: {"total": None, "parts": {}, "ts": time.time()})
        print(f"Video receiver listening on port {self.video_listen_addr[1]}")

        while not self.stop_event.is_set():
            try:
                packet, _ = s.recvfrom(65535)
                if len(packet) < 6: continue
                
                fid, total, idx = struct.unpack("!HHH", packet[:6])
                payload = packet[6:]
                buf = buffers[fid]
                buf["ts"] = time.time()
                if buf["total"] is None: buf["total"] = total
                buf["parts"][idx] = payload

                if len(buf["parts"]) == buf["total"]:
                    jpg = b"".join(buf["parts"][i] for i in range(buf["total"]))
                    frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.frame_lock:
                            self.latest_frame = frame
                    del buffers[fid]

            except socket.timeout:
                # Purge stale frame buffers
                now = time.time()
                for k in [k for k, v in buffers.items() if now - v["ts"] > 2.0]:
                    del buffers[k]
                continue
            except Exception as e:
                print(f"Video loop error: {e}")
                time.sleep(1)
        s.close()
        
    def _imu_loop(self):
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(self.imu_listen_addr); s.settimeout(1.0)
        while not self.stop_event.is_set():
            try:
                d,_=s.recvfrom(1024); self.latest_imu_data=json.loads(d.decode("utf-8"))
            except (socket.timeout, Exception): self.latest_imu_data=None
        s.close()

    def run(self):
        vt=threading.Thread(target=self._video_loop,daemon=True); it=threading.Thread(target=self._imu_loop,daemon=True)
        vt.start(); it.start(); vt.join(); it.join()

    def stop(self): self.stop_event.set()

# --- MODIFICATION: New client to send PWM signals in binary format ---
class RealSubmarinePWMClient:
    def __init__(self, server_ip, control_port=10000):
        self.server_address = (server_ip, control_port)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"RealSubPWMClient: Ready to send PWM packets to {self.server_address}")

    def _force_to_pwm(self, force):
        """Converts a force value from -100 to 100 to a PWM pulse."""
        # Clip force to ensure it's within the expected range
        force = np.clip(force, -100.0, 100.0)
        pulse = NEUTRAL_PULSE + (force / 100.0) * AMP
        return int(pulse)

    def send_control(self, armed, light_on, forces):
        if not armed:
            forces = np.zeros(6)
            light_pulse = LIGHT_OFF_PULSE
        else:
            light_pulse = LIGHT_ON_PULSE if light_on else LIGHT_OFF_PULSE

        # Convert the 6 thruster forces to PWM pulses
        thruster_pulses = [self._force_to_pwm(f) for f in forces]
        
        # Create the 7-channel pulse list (6 thrusters + 1 light)
        all_pulses = thruster_pulses + [light_pulse]

        try:
            # Pack the 7 pulses into a little-endian binary format (<7H)
            packet = struct.pack("<7H", *all_pulses)
            self.control_socket.sendto(packet, self.server_address)
        except Exception as e:
            print(f"RealSubPWMClient Error: {e}", file=sys.stderr)

    def shutdown(self):
        print("RealSubPWMClient: Shutting down and sending safe values.")
        self.send_control(False, False, np.zeros(6))
        time.sleep(0.1)
        self.control_socket.close()

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
        self.hud_data={"armed":False,"light_on":False,"chase_mode":"INACTIVE"}
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
            # The visualizer now draws the bounding box itself.
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

# The simulator communication classes remain unchanged
class UDPClientProtocol(asyncio.DatagramProtocol):
    def __init__(self, on_con_lost: asyncio.Future): self.transport=None; self.on_con_lost=on_con_lost; self.recv_queue=asyncio.Queue()
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
                self.connected=True; print(f"AsyncAUV (Simulator): UDP transport ready for {self.server_ip}:{self.server_port}")
            except Exception as e:
                self.connected=False; print(f"AsyncAUV (Simulator): UDP transport creation failed: {e}"); raise
    async def _send_command(self,data:bytes)->Optional[bytes]:
        max_retries=3
        for attempt in range(max_retries):
            try:
                await self._ensure_transport()
                while not self.protocol.recv_queue.empty(): self.protocol.recv_queue.get_nowait()
                self.transport.sendto(data)
                return await asyncio.wait_for(self.protocol.recv_queue.get(),timeout=2.)
            except asyncio.TimeoutError:
                if attempt==max_retries-1: print(f"AsyncAUV (Simulator): Command timed out after {max_retries} attempts")
            except Exception as e:
                print(f"AsyncAUV (Simulator): Unexpected error: {e}"); self.connected=False; await asyncio.sleep(.1)
        return None
    async def apply_ctrl(self,forces:np.ndarray,num_steps:int=1)->Optional[dict]:
        if len(forces)!=6: raise ValueError("Forces array must have 6 elements")
        data=struct.pack("<8f",float(MsgHeader.APPLY_CTRL.value),float(num_steps),*forces); await self._send_command(data)
        return None # Simplified to fire-and-forget for speed
    async def close(self):
        if self.transport:self.transport.close(); self.connected=False


class BlueROVController:
    def __init__(self, args):
        # --- MODIFICATION: Conditionally initialize the simulator client ---
        self.auv = None
        if not args.no_sim:
            self.auv = AsyncAUV(args.ip, args.port)
        else:
            print("Simulator connection is disabled by --no_sim flag.")

        self.args = args
        pygame.init()
        self.controller = None
        if not self.args.chase_target and not self.args.direction:
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0: raise RuntimeError("No gamepad. Use --direction or --chase_target.")
            self.controller = pygame.joystick.Joystick(0); self.controller.init(); print(f"Gamepad: {self.controller.get_name()}")
        else: print("Running in an automated mode. Gamepad input is disabled.")
        
        self.real_sub_client = RealSubmarinePWMClient(args.sub_ip, args.sub_port) if args.sub_ip else None
        
        self.data_receiver_thread = SubmarineDataReceiverThread()
        self.armed = False
        self.light_on = False
        self.minForce = np.full(6, -100)
        self.maxForce = np.full(6, 100)
        self.translation_scale = 0.8
        self.rotation_scale = 0.4
        
        self.yolo_model = None
        self.chase_strategy = None
        if self.args.chase_target:
            if not self.args.weights: raise ValueError("Chase mode requires --weights path/to/your.pt")
            print(f"Loading YOLO model from {self.args.weights}..."); self.yolo_model = YOLO(self.args.weights); print("YOLO model loaded.")
            self.chase_strategy = ChaseAndCircleStrategy(camera_width=640, camera_height=480)

        self.LEFT_STICK_X=0;self.LEFT_STICK_Y=1;self.RIGHT_STICK_X=2;self.RIGHT_STICK_Y=3
        self.BTN_B=1;self.BTN_MENU=6
        self.DEADZONE=0.1;self.dt=0.02

    # ... (keep apply_deadzone, get_controller_input, calculate_thruster_forces the same) ...
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

    def calculate_thruster_forces(self, surge, strafe, heave, yaw, system_type):
        if system_type == "sim":
            forces = np.array([surge-strafe+yaw, surge+strafe-yaw, -surge-strafe-yaw, -surge+strafe+yaw, heave, heave])
        else:
            forces = np.array([surge-strafe+yaw, surge+strafe-yaw, surge+strafe+yaw, surge-strafe-yaw, heave, -heave])
        return np.clip(forces * 100.0, self.minForce, self.maxForce)


    async def run_controller_async(self):
        if self.chase_strategy or self.args.direction:
            self.armed=True
            mode = "CHASE" if self.chase_strategy else f"'{self.args.direction}'"
            print(f"🚀 Starting in {mode} mode. Auto-arming vehicle.")
        else: print("🚀 Starting controller. Press MENU to arm/disarm.")
        
        self.visualizer = CombinedVisualizer(self.data_receiver_thread)
        self.data_receiver_thread.start()
        shutdown = False
        cmd_start_time = time.time()

        try:
            while not shutdown:
                # ... (keep the event handling loop the same) ...
                loop_start = time.time()
                if self.args.runtime > 0 and (time.time() - cmd_start_time > self.args.runtime):
                    print(f"\nRuntime of {self.args.runtime}s complete. Shutting down."); shutdown = True
                for e in pygame.event.get():
                    if e.type == pygame.QUIT: shutdown = True; break
                    if self.controller and e.type == pygame.JOYBUTTONDOWN:
                        if e.button == self.BTN_MENU: self.armed = not self.armed; print(f"\n[{'ARMED' if self.armed else 'DISARMED'}]")
                        elif e.button == self.BTN_B: self.light_on = not self.light_on
                if shutdown: break

                surge, strafe, heave, yaw_cmd = 0., 0., 0., 0.
                bbox_to_use = BoundingBox() 

                # ... (keep the chase logic / YOLO detection section the same) ...
                if self.chase_strategy and self.armed:
                    with self.data_receiver_thread.frame_lock: frame = self.data_receiver_thread.latest_frame
                    if frame is not None and self.yolo_model is not None:
                        yres = self.yolo_model(frame, conf=self.args.conf, max_det=1, verbose=False)[0]
                        dets = sv.Detections.from_ultralytics(yres)
                        if len(dets) > 0:
                            xyxy = dets.xyxy[0]
                            x1, y1, x2, y2 = xyxy
                            bbox_to_use = BoundingBox(x=x1, y=y1, width=x2-x1, height=y2-y1)
                    
                    self.visualizer.latest_bbox = bbox_to_use
                    s_surge, s_strafe, s_heave, s_yaw, flash = self.chase_strategy.update(bbox_to_use, self.dt)
                    surge, strafe, heave, yaw_cmd = s_surge, s_strafe, s_heave, s_yaw
                    self.light_on = flash
                elif self.args.direction and self.armed:
                    if self.args.direction == "forward": surge = self.args.fwcmd / 100.0
                    elif self.args.direction == "backward": surge = -self.args.fwcmd / 100.0
                else:
                    surge, strafe, heave, yaw_cmd = self.get_controller_input()

                scaled_surge = surge * self.translation_scale
                scaled_strafe = strafe * self.translation_scale
                scaled_heave = heave * self.translation_scale
                scaled_yaw = yaw_cmd * self.rotation_scale
                
                # --- MODIFICATION: Conditionally create tasks ---
                tasks = []
                # Only add the simulator task if the client was created
                if self.auv:
                    sim_f = self.calculate_thruster_forces(scaled_surge, scaled_strafe, scaled_heave, scaled_yaw, "sim")
                    tasks.append(self.auv.apply_ctrl(sim_f if self.armed else np.zeros(6), self.args.num_steps))
                
                # Always add the real submarine task if the client was created
                if self.real_sub_client:
                    real_f = self.calculate_thruster_forces(scaled_surge, scaled_strafe, scaled_heave, scaled_yaw, "real")
                    loop = asyncio.get_event_loop()
                    tasks.append(loop.run_in_executor(None, self.real_sub_client.send_control, self.armed, self.light_on, real_f))
                
                # Only run gather if there are tasks to perform
                if tasks:
                    await asyncio.gather(*tasks, return_exceptions=True)
                
                chase_mode_status = self.chase_strategy.state.name if self.chase_strategy else "INACTIVE"
                self.visualizer.hud_data.update({"armed":self.armed, "light_on":self.light_on, "chase_mode":chase_mode_status})
                self.visualizer.update()
                
                loop_time = time.time() - loop_start
                await asyncio.sleep(max(0, self.dt - loop_time))
        finally:
            print("Disarming and stopping all systems...")
            self.data_receiver_thread.stop()
            self.data_receiver_thread.join(timeout=1)
            plt.close("all")
            # --- MODIFICATION: Conditionally close the simulator connection ---
            if self.auv:
                await self.auv.close()
            if self.real_sub_client: 
                self.real_sub_client.shutdown()
            pygame.quit()

    def run(self): asyncio.run(self.run_controller_async())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Integrated BlueROV2 Controller for Sim and Real Sub")
    # --- MODIFICATION: Add the --no_sim flag ---
    parser.add_argument("--no_sim", action="store_true", help="Disable connection to the simulator.")
    
    # ... (the rest of the arguments are the same) ...
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="IP of the simulator.")
    parser.add_argument("--port", type=int, default=60001, help="Port of the simulator.")
    parser.add_argument("--num_steps", type=int, default=5, help="Simulator physics steps per cycle.")
    parser.add_argument("--sub_ip", type=str, default="192.168.2.11", help="IP of real submarine Pi.")
    parser.add_argument("--sub_port", type=int, default=5005, help="Control port on real submarine Pi.")
    parser.add_argument("--direction", type=str, choices=["forward","backward"], default=None, help="Constant force mode.")
    parser.add_argument("--fwcmd", type=float, default=25., help="Force to apply for --direction mode.")
    parser.add_argument("--runtime", type=float, default=0., help="Time to run (0 for infinite).")
    parser.add_argument("--chase_target", action="store_true", help="Enable autonomous chase using YOLO.")
    parser.add_argument("--weights", type=str, default="./best.pt", help="Path to YOLO model weights.")
    parser.add_argument("--conf", type=float, default=0.3, help="YOLO score threshold.")

    args = parser.parse_args()
    if args.chase_target and args.direction:
        sys.exit("[FATAL ERROR] --chase_target and --direction are mutually exclusive.")
    try:
        controller = BlueROVController(args)
        controller.run()
    except Exception as e:
        print(f"\n[FATAL ERROR] Could not start: {e}", file=sys.stderr); pygame.quit(); sys.exit(1)
