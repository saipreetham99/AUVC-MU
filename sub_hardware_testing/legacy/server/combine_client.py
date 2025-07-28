# File: combine_client.py
import pygame
import socket
import struct
import json
import cv2
import numpy as np
import threading
import time
import sys

# --- Configuration ---
SERVER_IP = '10.144.113.129'  # The IP of your Raspberry Pi submarine
CONTROL_PORT = 10000          # UDP port for sending commands
VIDEO_PORT = 10001            # UDP port for receiving video
IMU_PORT = 10002              # UDP port for receiving IMU data

# --- CONTROL CONSTANTS ---
NEUTRAL_PULSE = 1500
LIGHT_OFF, LIGHT_ON = 1100, 1900
DEAD = 0.6
LOOP_DURATION = 0.02 # 20ms, for a 50Hz loop rate

# --- CONTROLLER MAPPINGS ---
AXES = {
    "LEFT_STICK_X": 0, "LEFT_STICK_Y": 1,
    "RIGHT_STICK_X": 2, "RIGHT_STICK_Y": 3,
    "LT": 4, "RT": 5,
}

BUTTONS = {
    "A": 0, "B": 1, "X": 2, "Y": 3,
    "BACK": 4, "XBOX": 5, "START": 6,
    "LEFT_STICK_PRESS": 7, "RIGHT_STICK_PRESS": 8,
    "LB": 9, "RB": 10,
    "D_HAT_UP": 11, "D_HAT_DOWN": 12,
    "D_HAT_LEFT": 13, "D_HAT_RIGHT": 14,
    "MIDDLE": 15,
}

# Helper function
def clamp(x):
    return max(-1.0, min(1.0, x))

class SubmarineClient:
    def __init__(self):
        self.running = True
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.latest_imu_data = None
        
        # --- UDP Control Socket ---
        self.server_address = (SERVER_IP, CONTROL_PORT)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # --- State Variables ---
        self.armed = False
        self.light_on = False
        self.lag_enabled = False # <<< NO-DELAY IS NOW DEFAULT
        self.AMP = 100
        self.AMP_STEP = 100
        self.AMP_MIN, self.AMP_MAX = 100, 400
        self.AMP_COOLDOWN = 0.25
        self.last_amp_time = 0

        # --- Initialize Pygame and Joystick ---
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            sys.exit("Error: No gamepad found.")
        self.pad = pygame.joystick.Joystick(0)
        self.pad.init()
        print(f"Gamepad found: {self.pad.get_name()}")
        
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Submarine Control")
        self.font = pygame.font.SysFont("monospace", 16)

    def _to_pwm(self, x):
        return int(NEUTRAL_PULSE + x * self.AMP)

    def _send_command(self, command_dict):
        """Encodes and sends a command dictionary over UDP."""
        try:
            payload = (json.dumps(command_dict) + '\n').encode('utf-8')
            self.control_socket.sendto(payload, self.server_address)
        except Exception as e:
            print(f"Error sending UDP command: {e}")

    # --- Video and IMU threads remain unchanged ---
    def _video_receiver_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', VIDEO_PORT)); buffers = {}
        while self.running:
            try:
                data, _ = sock.recvfrom(65536)
                if len(data) < 6: continue
                frame_id, num_chunks, chunk_id = struct.unpack('!HHH', data[:6])
                if frame_id not in buffers: buffers[frame_id] = {}
                buffers[frame_id][chunk_id] = data[6:]
                if len(buffers[frame_id]) == num_chunks:
                    full_jpeg = b''.join(chunk[1] for chunk in sorted(buffers[frame_id].items()))
                    frame = cv2.imdecode(np.frombuffer(full_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    with self.frame_lock: self.latest_frame = frame
                    del buffers[frame_id]
                if frame_id % 100 == 0:
                    for f_id in list(buffers.keys()):
                        if f_id < frame_id - 10: del buffers[f_id]
            except Exception: pass
        sock.close()

    def _imu_receiver_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', IMU_PORT))
        while self.running:
            try:
                data, _ = sock.recvfrom(1024)
                self.latest_imu_data = json.loads(data.decode('utf-8'))
            except Exception: self.latest_imu_data = None
        sock.close()
        
    def _draw_hud(self):
        """Draws all status info, including the lag toggle state."""
        arm_text = self.font.render(f"ARMED: {self.armed}", True, (255, 0, 0) if self.armed else (0, 255, 0))
        self.screen.blit(arm_text, (10, 10))
        
        light_text = self.font.render(f"LIGHT: {'ON' if self.light_on else 'OFF'}", True, (255, 255, 0))
        self.screen.blit(light_text, (10, 30))

        amp_text = self.font.render(f"AMP: {self.AMP}", True, (255, 255, 255))
        self.screen.blit(amp_text, (10, 50))
        
        # <<<--- HUD BUTTON/STATUS FOR LAG TOGGLE --- >>>
        lag_status_text = "DELAY: OFF (MAX)" if not self.lag_enabled else "DELAY: ON (20ms)"
        lag_color = (255, 100, 0) if not self.lag_enabled else (0, 255, 255)
        lag_text = self.font.render(lag_status_text, True, lag_color)
        self.screen.blit(lag_text, (10, 70))
        
        if self.latest_imu_data:
            r = self.latest_imu_data['euler_deg']['roll']
            p = self.latest_imu_data['euler_deg']['pitch']
            y = self.latest_imu_data['euler_deg']['yaw']
            imu_text = self.font.render(f"R:{r:6.1f} P:{p:6.1f} Y:{y:6.1f}", True, (255, 255, 255))
            self.screen.blit(imu_text, (10, 90))

    def run(self):
        threading.Thread(target=self._video_receiver_loop, daemon=True).start()
        threading.Thread(target=self._imu_receiver_loop, daemon=True).start()
        clock = pygame.time.Clock()
        
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False; continue
                
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BUTTONS["START"]:
                        self.armed = not self.armed
                        print(f"\n[{'ARMED' if self.armed else 'SAFE'}]")
                        continue

                    # Lag Toggle Logic
                    elif event.button == BUTTONS["MIDDLE"]:
                        self.lag_enabled = not self.lag_enabled
                        status_text = "ENABLED (20ms)" if self.lag_enabled else "DISABLED (MAX SPEED)"
                        print(f"\n[CONTROL DELAY {status_text}]")

                    # D-Pad Controls
                    elif event.button == BUTTONS["D_HAT_RIGHT"]:
                        self.light_on = True; print("[LIGHT ON]")
                    elif event.button == BUTTONS["D_HAT_LEFT"]:
                        self.light_on = False; print("[LIGHT OFF]")
                    elif event.button == BUTTONS["D_HAT_UP"] and time.time() - self.last_amp_time > self.AMP_COOLDOWN:
                        self.AMP = min(self.AMP + self.AMP_STEP, self.AMP_MAX)
                        self.last_amp_time = time.time(); print("AMP", self.AMP)
                    elif event.button == BUTTONS["D_HAT_DOWN"] and time.time() - self.last_amp_time > self.AMP_COOLDOWN:
                        self.AMP = max(self.AMP - self.AMP_STEP, self.AMP_MIN)
                        self.last_amp_time = time.time(); print("AMP", self.AMP)

            # Axis, Deadzone, Bumper, and Safety logic is unchanged
            surge = -self.pad.get_axis(AXES["LEFT_STICK_Y"])
            strafe = self.pad.get_axis(AXES["LEFT_STICK_X"])
            heave = -self.pad.get_axis(AXES["RIGHT_STICK_Y"])
            yaw = self.pad.get_axis(AXES["RIGHT_STICK_X"])

            surge, strafe, heave, yaw = (0 if abs(v) < DEAD else v for v in [surge, strafe, heave, yaw])

            if self.pad.get_button(BUTTONS["RB"]) ^ self.pad.get_button(BUTTONS["LB"]):
                yaw = 1.0 if self.pad.get_button(BUTTONS["RB"]) else -1.0

            if not self.armed:
                surge = strafe = heave = yaw = 0

            if self.pad.get_button(BUTTONS["BACK"]):
                self.running = False; continue

            # Motor Mix & Command Packet
            t0 = self._to_pwm(clamp(surge - strafe - yaw))
            t1 = self._to_pwm(clamp(surge + strafe + yaw))
            t2 = self._to_pwm(clamp(surge + strafe - yaw))
            t3 = self._to_pwm(clamp(surge - strafe + yaw))
            t4 = self._to_pwm(clamp(heave))
            t5 = self._to_pwm(clamp(-heave))
            light_value = LIGHT_ON if self.light_on else LIGHT_OFF

            self._send_command({
                'command': 'control',
                'thruster_pulses': [t0, t1, t2, t3, t4, t5],
                'light_pulse': light_value, 
                'duration': LOOP_DURATION
            })

            # Drawing
            self.screen.fill((0, 0, 0))
            with self.frame_lock:
                if self.latest_frame is not None:
                    try:
                        frame_rgb = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
                        frame_pygame = pygame.surfarray.make_surface(np.rot90(frame_rgb))
                        self.screen.blit(frame_pygame, (0, 0))
                    except cv2.error: pass
            self._draw_hud()
            pygame.display.flip()
            
            # Conditional Loop Pacing
            if self.lag_enabled:
                clock.tick(1 / LOOP_DURATION)

        self.shutdown()

    def shutdown(self):
        print("Shutting down client...")
        self.running = False
        # Send one last neutral command for safety
        self._send_command({
            'command': 'control','thruster_pulses': [NEUTRAL_PULSE] * 6,
            'light_pulse': LIGHT_OFF, 'duration': 0
        })
        time.sleep(0.1) # Give it time to send
        self.control_socket.close()
        pygame.quit()

if __name__ == '__main__':
    client = SubmarineClient()
    try:
        client.run()
    except KeyboardInterrupt:
        client.shutdown()
