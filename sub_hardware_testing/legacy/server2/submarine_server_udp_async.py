# File: submarine_server.py
import socket
import threading
import json
import time
import logging
import sys
import argparse
import struct
import numpy as np
import cv2

# Import our refactored modules
import cam
import imu
import thrusters

CONTROL_PORT = 10000        # UDP port for receiving commands
VIDEO_PORT = 10001          # UDP port for streaming video
IMU_PORT = 10002            # UDP port for streaming IMU data

# Safety timeout in seconds. If no packet is received for this duration,
# thrusters will be set to neutral.
CONTROL_TIMEOUT = 0.5

# Setup basic logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] [%(name)s] %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
# Create a logger for this module
logger = logging.getLogger('SubServer')


class SubmarineServer:
    def __init__(self, host_ip, client_ip):
        self.host_ip = host_ip
        self.client_ip = client_ip
        self.running = True
        self.camera_system = None
        self.imu_system = None
        self.thruster_system = None
        
    def start(self):
        """Initializes and starts all subsystems."""
        try:
            # --- Initialize Camera (or fallback to noise) ---
            logger.info("--- Initializing Camera ---")
            try:
                self.camera_system = cam.CameraStreamer(client_ip=self.client_ip, port=VIDEO_PORT)
                if self.camera_system.initialized:
                    self.camera_system.start()
                else:
                    raise RuntimeError("Camera hardware failed to initialize.")
            except Exception as cam_err:
                logger.warning(f"Camera initialization failed: {cam_err}. Falling back to noise stream.")
                self.camera_system = None 
                threading.Thread(target=self._noise_streamer_loop, daemon=True).start()

            # --- Initialize IMU ---
            logger.info("--- Initializing IMU ---")
            self.imu_system = imu.IMUSystem()
            self.imu_system.start()
            
            time.sleep(4) # Wait for IMU to get first reading

            # --- Initialize Thrusters (or continue without if failed) ---
            logger.info("--- Initializing Thrusters ---")
            self.thruster_system = thrusters.ThrusterController()
            if not self.thruster_system.initialized:
                logger.warning("Thruster controller (PCA9685) failed to initialize. The server will run without thruster control.")
            else:
                logger.info("Thruster controller initialized successfully.")

            logger.info("--- All systems initialized. Server is ready. ---")

            # Start networking threads
            threading.Thread(target=self._udp_control_loop, daemon=True).start()
            threading.Thread(target=self._imu_streamer_loop, daemon=True).start()

        except Exception as e:
            logger.critical(f"A critical error occurred during server startup: {e}", exc_info=True)
            self.shutdown()
            return
            
        while self.running:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received.")
                self.shutdown()

    def shutdown(self):
        """Gracefully shuts down all subsystems."""
        if not self.running: return
        logger.info("--- Server shutting down... ---")
        self.running = False
        if self.camera_system: self.camera_system.stop()
        if self.imu_system: self.imu_system.stop()
        if self.thruster_system: self.thruster_system.safe_shutdown()
        logger.info("--- Shutdown complete. ---")

    def _noise_streamer_loop(self):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info(f"Noise streamer started, sending to {self.client_ip}:{VIDEO_PORT}")
        frame_id = 0
        resolution = (640, 480); quality = [int(cv2.IMWRITE_JPEG_QUALITY), 40]; max_packet_size = 60000
        while self.running:
            try:
                noise_frame = np.random.randint(0, 255, (resolution[1], resolution[0], 3), dtype=np.uint8)
                result, encoded_frame = cv2.imencode('.jpg', noise_frame, quality)
                if not result: continue
                data = encoded_frame.tobytes()
                num_chunks = (len(data) + max_packet_size - 1) // max_packet_size
                for i in range(num_chunks):
                    chunk = data[i * max_packet_size:(i + 1) * max_packet_size]
                    header = struct.pack('!HHH', frame_id, num_chunks, i)
                    udp_socket.sendto(header + chunk, (self.client_ip, VIDEO_PORT))
                frame_id = (frame_id + 1) % 65535
                time.sleep(1/24)
            except Exception as e:
                logger.error(f"Error in noise streamer loop: {e}")
                time.sleep(1)
        udp_socket.close()
        logger.info("Noise streamer stopped.")

    def _imu_streamer_loop(self):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info(f"IMU data streamer started, sending to {self.client_ip}:{IMU_PORT}")
        while self.running:
            try:
                data = self.imu_system.get_latest_data()
                if data:
                    payload = json.dumps(data).encode('utf-8')
                    udp_socket.sendto(payload, (self.client_ip, IMU_PORT))
                time.sleep(1/50)
            except Exception as e:
                logger.error(f"Error in IMU streamer loop: {e}")
                time.sleep(1)
        udp_socket.close()

    def _udp_control_loop(self):
        """Listens for UDP command packets and handles them."""
        control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        control_socket.bind((self.host_ip, CONTROL_PORT))
        control_socket.settimeout(CONTROL_TIMEOUT)
        logger.info(f"Control server listening on UDP {self.host_ip}:{CONTROL_PORT}")

        while self.running:
            try:
                data, addr = control_socket.recvfrom(1024)
                line = data.strip()
                if not line: continue
                msg = json.loads(line.decode('utf-8'))
                command = msg.get('command')

                if command == 'control':
                    if self.thruster_system and self.thruster_system.initialized:
                        self.thruster_system.apply_control(
                            msg['thruster_pulses'],
                            msg['light_pulse'],
                            0
                        )
                    else:
                        logger.debug("PCA9685 chip not detected! Ignoring thruster command.")

                elif command == 'reset_orientation':
                    self.imu_system.reset_orientation()
                    logger.info(f"Rx from {addr}: Command to reset IMU orientation.")
                else:
                    logger.warning(f"Rx from {addr}: Unknown command '{command}'")

            except socket.timeout:
                logger.warning(f"CONTROL TIMEOUT: No packet for {CONTROL_TIMEOUT}s. Setting thrusters to neutral.")
                if self.thruster_system and self.thruster_system.initialized:
                    self.thruster_system.set_neutral()

            except json.JSONDecodeError:
                logger.error("Received invalid JSON from client.")
            except Exception as e:
                if self.running:
                    logger.error(f"Error in UDP control loop: {e}", exc_info=True)
        control_socket.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Submarine Server")
    parser.add_argument(
        '--host', 
        type=str, 
        default='0.0.0.0', 
        help='The IP address for the server to listen on. Default is 0.0.0.0 (all interfaces).'
    )
    parser.add_argument(
        '--client', 
        type=str, 
        default='192.168.2.1', 
        help='The IP address of the client to send data to (Client IP). Default is 192.168.2.1.'
    )
    args = parser.parse_args()

    server = SubmarineServer(host_ip=args.host, client_ip=args.client)
    try:
        server.start()
    finally:
        server.shutdown()
