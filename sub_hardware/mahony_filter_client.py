#!/usr/bin/python3
"""
Simple Attitude Client - Visualization with server-side orientation reset functionality.
Corrected to apply a POSITIVE 90 degree rotation around the Z-axis to the sensor frame.
MODIFIED to swap the Roll and Pitch axes for final display.
"""

import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from collections import deque
import threading
import time
import argparse

class SimpleAttitudeClient:
    """Client with plotting and server-side orientation reset."""

    def __init__(self, server_host='127.0.0.1', server_port=9999):
        self.server_host = server_host
        self.server_port = server_port

        self.time_window_size = 30.0
        history_length = int(self.time_window_size * 100) # 30s at 100Hz

        # This matrix performs the +90 degree rotation from the previous request. We keep it.
        self.rotation_fix = np.array([
            [0., -1., 0.],
            [1.,  0., 0.],
            [0.,  0., 1.]
        ])

        # Connection and data state
        self.socket = None
        self.connected = False
        self.running = False
        self.latest_data = None
        self.data_lock = threading.Lock()

        # History Deques
        self.times = deque(maxlen=history_length)
        self.roll_history = deque(maxlen=history_length)
        self.pitch_history = deque(maxlen=history_length)
        self.yaw_history = deque(maxlen=history_length)

        # Configure matplotlib
        try:
            plt.style.use('seaborn-v0_8-darkgrid')
        except:
            try: plt.style.use('seaborn-darkgrid')
            except: pass
        plt.rcParams['figure.dpi'] = 100
        plt.rcParams['font.size'] = 12

        # Create figure and subplots
        self.fig = plt.figure(figsize=(15, 7))
        self.fig.subplots_adjust(bottom=0.15)

        self.ax_euler = self.fig.add_subplot(1, 2, 1)
        self.ax_3d = None
        self.fig.suptitle(f'Attitude Orientation Tracking - {server_host}', fontsize=16)

        self.setup_2d_plot()
        self.setup_widgets()
        self.start_time = time.time()
        
        self.frame_lines = None
        self.ani = None

    def setup_widgets(self):
        ax_button = self.fig.add_axes([0.4, 0.02, 0.2, 0.06])
        self.reset_button = Button(ax_button, 'Reset Orientation')
        self.reset_button.on_clicked(self._reset_orientation)
        self.reset_button.ax.set_alpha(0.5)

    def setup_2d_plot(self):
        self.line_roll, = self.ax_euler.plot([], [], 'r-', lw=2, label='Pitch')
        self.line_pitch, = self.ax_euler.plot([], [], 'g-', lw=2, label='Roll')
        self.line_yaw, = self.ax_euler.plot([], [], 'b-', lw=2, label='Yaw')

        self.ax_euler.set_xlabel('Time (s)'); self.ax_euler.set_ylabel('Angle (degrees)')
        self.ax_euler.set_title('Euler Angles'); self.ax_euler.legend(loc='upper left')
        self.ax_euler.set_ylim(-180, 180); self.ax_euler.grid(True, alpha=0.4)
        self.ax_euler.set_xlim(0, self.time_window_size)

        self.status_text = self.ax_euler.text(0.98, 0.98, 'Disconnected',
                                             transform=self.ax_euler.transAxes,
                                             fontsize=12, color='red', ha='right', va='top',
                                             bbox=dict(boxstyle='round', fc='white', alpha=0.8))

        self.info_text = self.fig.text(0.5, 0.95, 'Waiting for connection...', ha='center', va='top', fontsize=11,
                                      bbox=dict(boxstyle='round', fc='lightgray', alpha=0.8))

    def setup_3d_plot(self):
        if self.ax_3d is None:
            self.ax_3d = self.fig.add_subplot(1, 2, 2, projection='3d')
            
            self.ax_3d.set_xlabel('X (Fwd)'); self.ax_3d.set_ylabel('Y (Right)'); self.ax_3d.set_zlabel('Z (Down)')
            self.ax_3d.set_xlim([-1.2, 1.2]); self.ax_3d.set_ylim([-1.2, 1.2]); self.ax_3d.set_zlim([-1.2, 1.2])
            self.ax_3d.set_title('3D Orientation')

            try: self.ax_3d.set_box_aspect([1,1,1])
            except AttributeError: pass

            self.frame_lines = []
            colors = ['red', 'green', 'blue']; labels = ['X (Roll)', 'Y (Pitch)', 'Z (Yaw)']
            for i in range(3):
                line, = self.ax_3d.plot([0,0],[0,0],[0,0], c=colors[i], lw=4, label=labels[i], alpha=0.9)
                self.frame_lines.append(line)

            self.ax_3d.scatter([0], [0], [0], c='k', s=80); self.ax_3d.legend(loc='upper right')
            try: self.ax_3d.view_init(elev=-5, azim=90)
            except: pass

            self.ax_3d.grid(True, alpha=0.2)
            self.fig.canvas.draw()

    def hide_3d_plot(self):
        if self.ax_3d is not None:
            self.ax_3d.clear()
            self.ax_3d.set_visible(False)
            self.ax_3d.text(0.5, 0.5, 0.5, 'Waiting for connection...', 
                          horizontalalignment='center', verticalalignment='center',
                          transform=self.ax_3d.transAxes, fontsize=14, color='gray')
            self.fig.canvas.draw()

    def _quaternion_to_euler(self, q):
        q0, q1, q2, q3 = q
        roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))
        sinp = 2 * (q0 * q2 - q3 * q1)
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
        return roll, pitch, yaw

    def _quaternion_to_rotation_matrix(self, q):
        q0, q1, q2, q3 = q
        return np.array([
            [q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])

    def _reset_orientation(self, event):
        if self.connected and self.socket:
            try:
                self.socket.sendall(b"TARE\n")
                print("\n--- Sent TARE command to server ---")
                self.times.clear(); self.roll_history.clear()
                self.pitch_history.clear(); self.yaw_history.clear()
            except Exception as e:
                print(f"Error sending TARE command: {e}")

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.server_host, self.server_port))
            self.socket.settimeout(None)
            self.connected = True
            print(f"✓ Connected to {self.server_host}:{self.server_port}")
            self.setup_3d_plot()
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            self.connected = False
            if self.socket:
                self.socket.close()
                self.socket = None
            return False

    def receive_data(self):
        buffer = ""
        while self.running and self.connected:
            try:
                data = self.socket.recv(4096).decode()
                if not data: 
                    self.connected = False; break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line:
                        try:
                            json_data = json.loads(line)
                            with self.data_lock:
                                self.latest_data = json_data
                                self.process_data(json_data)
                        except json.JSONDecodeError: pass
            except Exception as e:
                print(f"Receive error: {e}"); self.connected = False; break
        
        if self.socket:
            try: self.socket.close()
            except: pass
            self.socket = None
        
        if self.running:
            print("Disconnected from server")
            self.hide_3d_plot()
            threading.Thread(target=self.reconnect_loop, daemon=True).start()

    def process_data(self, data):
        if 'status' in data:
            if data['status'] == 'RESET_OK':
                print(f"Server confirmed orientation reset at {data.get('timestamp', 'unknown')}")
            return
        
        if 'quaternion' in data:
            current_time = time.time() - self.start_time
            q = np.array(data['quaternion'])
            
            # Get the original roll and pitch from the sensor data
            roll_orig, pitch_orig, yaw_orig = self._quaternion_to_euler(q)

            # --- FIX 1: Swap Roll and Pitch data before storing it ---
            # The "Roll" history will now store the sensor's pitch values.
            # The "Pitch" history will now store the sensor's roll values.
            self.roll_history.append(-np.degrees(pitch_orig))
            self.pitch_history.append(-np.degrees(roll_orig))
            self.yaw_history.append(np.degrees(yaw_orig)) # Yaw is unchanged
            self.times.append(current_time)


    def animate(self, frame):
        if self.connected: 
            self.status_text.set_text('Connected'); self.status_text.set_color('green')
            self.reset_button.ax.set_alpha(1.0)
        else: 
            self.status_text.set_text('Disconnected'); self.status_text.set_color('red')
            self.reset_button.ax.set_alpha(0.5)

        if len(self.times) > 1:
            # The history deques already contain the swapped data, so the 2D plot
            # and text display will be correct automatically.
            times = list(self.times)
            self.line_roll.set_data(times, list(self.roll_history))
            self.line_pitch.set_data(times, list(self.pitch_history))
            self.line_yaw.set_data(times, list(self.yaw_history))

            if times[-1] > self.time_window_size: 
                self.ax_euler.set_xlim(times[-1] - self.time_window_size, times[-1])
            else: 
                self.ax_euler.set_xlim(0, self.time_window_size)
            
            if self.connected and self.ax_3d is not None and self.ax_3d.get_visible() and self.frame_lines:
                if self.latest_data and 'quaternion' in self.latest_data:
                    q = np.array(self.latest_data['quaternion'])
                    R_sensor = self._quaternion_to_rotation_matrix(q)
                    
                    # First, apply the +90 degree rotation to get the base orientation
                    R_rotated = self.rotation_fix @ R_sensor
                    x_axis_final = R_rotated[:, 0]
                    y_axis_final = R_rotated[:, 1]
                    z_axis_final = R_rotated[:, 2]
                    x_axis_final *=-1
                    y_axis_final*=-1
                    # --- FIX 2: Swap the axes for the 3D plot display ---
                    # The Roll line (red, frame_lines[0]) should show the final Pitch axis (y_axis_final)
                    self.frame_lines[0].set_data([0, x_axis_final[0]], [0, x_axis_final[1]])
                    self.frame_lines[0].set_3d_properties([0, x_axis_final[2]])

                    # The Pitch line (green, frame_lines[1]) should show the final Roll axis (x_axis_final)
                    self.frame_lines[1].set_data([0, y_axis_final[0]], [0, y_axis_final[1]])
                    self.frame_lines[1].set_3d_properties([0, y_axis_final[2]])

                    # The Yaw line (blue, frame_lines[2]) is unchanged
                    self.frame_lines[2].set_data([0, z_axis_final[0]], [0, z_axis_final[1]])
                    self.frame_lines[2].set_3d_properties([0, z_axis_final[2]])


            rate = len(self.times) / ((times[-1] - times[0]) + 1e-6) if len(self.times) > 1 else 0.0
            self.info_text.set_text(
                f'Pitch: {self.roll_history[-1]:6.1f}° | Roll: {self.pitch_history[-1]:6.1f}° | Yaw: {self.yaw_history[-1]:6.1f}° | Rate: {rate:.1f} Hz')
        else:
            self.ax_euler.set_xlim(0, self.time_window_size)
            if not self.connected:
                self.info_text.set_text('Waiting for connection...')

    def reconnect_loop(self):
        while self.running and not self.connected:
            print("Attempting to reconnect...")
            time.sleep(3)
            if self.connect():
                threading.Thread(target=self.receive_data, daemon=True).start()
                break

    def run(self):
        self.running = True
        self.ani = FuncAnimation(self.fig, self.animate, interval=33, blit=False, cache_frame_data=False)
        
        connect_thread = threading.Thread(target=self.try_initial_connection, daemon=True)
        connect_thread.start()

        try:
            plt.show()
        except Exception:
            pass
        finally:
            self.shutdown()

    def try_initial_connection(self):
        if not self.connect():
            print("Running in offline mode - will attempt to reconnect periodically.")
            threading.Thread(target=self.reconnect_loop, daemon=True).start()
        else:
            threading.Thread(target=self.receive_data, daemon=True).start()
            
    def shutdown(self):
        print("\nShutting down...")
        self.running = False
        if self.ani:
            self.ani.event_source.stop()

        socket_to_close = self.socket
        if socket_to_close:
            self.socket = None
            try: socket_to_close.shutdown(socket.SHUT_RDWR)
            except (OSError, socket.error): pass
            try: socket_to_close.close()
            except (OSError, socket.error): pass
        
        plt.close('all')

def main():
    parser = argparse.ArgumentParser(description='Simple Attitude Visualization Client')
    parser.add_argument('--server', default='192.168.2.11', help='Server IP')
    parser.add_argument('--port', type=int, default=9999, help='Server port')
    args = parser.parse_args()

    print("Attitude Visualization Client")
    print(f"Attempting to connect to {args.server}:{args.port}")
    print("-" * 40)

    client = SimpleAttitudeClient(server_host=args.server, server_port=args.port)
    client.run()
    
    print("Client has shut down.")

if __name__ == '__main__':
    main()
