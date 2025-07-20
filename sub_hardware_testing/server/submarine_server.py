# File: submarine_server.py
import socket
import threading
import json
import time
import logging
import sys

# Import our refactored modules
import cam
import imu
import thrusters

# --- Configuration ---
HOST_IP = '0.0.0.0'         # Listen on all available network interfaces
# CLIENT_IP is no longer needed for sending, but kept for camera/imu
CLIENT_IP = '10.144.113.184'

CONTROL_PORT = 10000        # UDP port for receiving commands
VIDEO_PORT = 10001          # UDP port for streaming video
IMU_PORT = 10002            # UDP port for streaming IMU data

# Safety timeout in seconds. If no packet is received for this duration,
# thrusters will be set to neutral.
CONTROL_TIMEOUT = 0.5

# Setup basic logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)

class SubmarineServer:
    def __init__(self):
        self.running = True
        self.camera_system = None
        self.imu_system = None
        self.thruster_system = None
        
    def start(self):
        """Initializes and starts all subsystems."""
        try:
            logging.info("--- Initializing Camera ---")
            self.camera_system = cam.CameraStreamer(client_ip=CLIENT_IP, port=VIDEO_PORT)
            self.camera_system.start()

            logging.info("--- Initializing IMU ---")
            self.imu_system = imu.IMUSystem()
            self.imu_system.start()
            
            time.sleep(4) # Wait for IMU to get first reading

            logging.info("--- Initializing Thrusters ---")
            self.thruster_system = thrusters.ThrusterController()
            logging.info("--- All systems initialized. Server is ready. ---")

            # Start networking threads
            threading.Thread(target=self._udp_control_loop, daemon=True).start()
            threading.Thread(target=self._imu_streamer_loop, daemon=True).start()

        except Exception as e:
            logging.critical(f"Failed to initialize server: {e}", exc_info=True)
            self.shutdown()
            return
            
        while self.running:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                logging.info("Keyboard interrupt received.")
                self.shutdown()

    def shutdown(self):
        """Gracefully shuts down all subsystems."""
        if not self.running: return
        logging.info("--- Server shutting down... ---")
        self.running = False
        if self.camera_system: self.camera_system.stop()
        if self.imu_system: self.imu_system.stop()
        if self.thruster_system: self.thruster_system.safe_shutdown()
        logging.info("--- Shutdown complete. ---")

    def _imu_streamer_loop(self):
        """Streams IMU data over UDP."""
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logging.info(f"IMU data streamer started, sending to {CLIENT_IP}:{IMU_PORT}")
        while self.running:
            try:
                data = self.imu_system.get_latest_data()
                if data:
                    payload = json.dumps(data).encode('utf-8')
                    udp_socket.sendto(payload, (CLIENT_IP, IMU_PORT))
                time.sleep(1/50)
            except Exception as e:
                logging.error(f"Error in IMU streamer loop: {e}")
                time.sleep(1)
        udp_socket.close()

    def _udp_control_loop(self):
        """Listens for UDP command packets and handles them."""
        control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        control_socket.bind((HOST_IP, CONTROL_PORT))
        control_socket.settimeout(CONTROL_TIMEOUT) # Set the safety timeout
        logging.info(f"Control server listening on UDP {HOST_IP}:{CONTROL_PORT}")

        while self.running:
            try:
                # Wait for a packet
                data, addr = control_socket.recvfrom(1024)
                
                # We expect a JSON string followed by a newline
                line = data.strip()
                if not line: continue

                msg = json.loads(line.decode('utf-8'))
                command = msg.get('command')

                if command == 'control':
                    # Log the received command for debugging
                    logging.info(f"Rx from {addr}: Thrusters: {msg['thruster_pulses']}, Light: {msg['light_pulse']}")
                    
                    self.thruster_system.apply_control(
                        msg['thruster_pulses'],
                        msg['light_pulse'],
                        0 # Duration is now handled client-side
                    )
                elif command == 'reset_orientation':
                    self.imu_system.reset_orientation()
                    logging.info(f"Rx from {addr}: Command to reset IMU orientation.")
                else:
                    logging.warning(f"Rx from {addr}: Unknown command '{command}'")

            except socket.timeout:
                # This is the safety feature. If no packet is received for the
                # duration of the timeout, set thrusters to neutral.
                logging.warning(f"CONTROL TIMEOUT: No packet received for {CONTROL_TIMEOUT}s. Setting thrusters to neutral.")
                self.thruster_system.set_neutral()
            except json.JSONDecodeError:
                logging.error("Received invalid JSON from client.")
            except Exception as e:
                if self.running:
                    logging.error(f"Error in UDP control loop: {e}", exc_info=True)

        control_socket.close()


if __name__ == '__main__':
    server = SubmarineServer()
    try:
        server.start()
    finally:
        server.shutdown()
