# File: submarine_server.py
import argparse
import json
import logging
import socket
import struct
import sys
import threading
import time

# Import our refactored modules
import cam
import cv2
import depth  # New depth sensor module
import imu
import numpy as np
import thrusters

CONTROL_PORT = 10000  # UDP port for receiving commands
VIDEO_PORT = 10001  # UDP port for streaming video
IMU_PORT = 10002  # UDP port for streaming IMU data (now includes depth)

# Safety timeout in seconds. If no packet is received for this duration,
# thrusters will be set to neutral.
CONTROL_TIMEOUT = 0.5

# Setup basic logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(name)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
# Create a logger for this module
logger = logging.getLogger("SubServer")


class SubmarineServer:
    def __init__(
        self,
        host_ip,
        client_ip,
        surface_depth_cm=10.0,
        pressure_units="mbar",
        depth_bus=6,
    ):
        self.host_ip = host_ip
        self.client_ip = client_ip
        self.surface_depth_cm = surface_depth_cm
        self.pressure_units = pressure_units
        self.depth_bus = depth_bus
        self.running = True
        self.camera_system = None
        self.imu_system = None
        self.thruster_system = None
        self.depth_system = None

    def start(self):
        """Initializes and starts all subsystems."""
        try:
            # --- Initialize Camera (or fallback to noise) ---
            logger.info("--- Initializing Camera ---")
            self.camera_system = cam.CameraStreamer(
                client_ip=self.client_ip, port=VIDEO_PORT
            )
            if self.camera_system.initialized:
                self.camera_system.start()
            else:
                logger.warning(
                    "Camera initialization failed. Falling back to noise stream."
                )
                self.camera_system = None  # Discard the failed object
                threading.Thread(target=self._noise_streamer_loop, daemon=True).start()

            # --- Initialize IMU ---
            logger.info("--- Initializing IMU ---")
            self.imu_system = imu.IMUSystem()
            self.imu_system.start()

            # --- Initialize Depth Sensor ---
            logger.info("--- Initializing Depth Sensor ---")
            self.depth_system = depth.DepthSensor(
                bus=self.depth_bus,
                surface_depth_cm=self.surface_depth_cm,
                pressure_units=self.pressure_units,
            )
            if not self.depth_system.initialized:
                logger.warning(
                    "Depth sensor failed to initialize. The server will run without depth sensing."
                )
                self.depth_system = None
            else:
                self.depth_system.start()
                logger.info("Depth sensor initialized successfully.")

            time.sleep(4)  # Wait for IMU and depth sensors to get first readings

            # --- Initialize Thrusters (or continue without if failed) ---
            logger.info("--- Initializing Thrusters ---")
            self.thruster_system = thrusters.ThrusterController()
            if not self.thruster_system.initialized:
                logger.warning(
                    "Thruster controller (PCA9685) failed to initialize. The server will run without thruster control."
                )
            else:
                logger.info("Thruster controller initialized successfully.")

            logger.info("--- All systems initialized. Server is ready. ---")

            # Start networking threads
            threading.Thread(target=self._udp_control_loop, daemon=True).start()
            threading.Thread(
                target=self._combined_data_streamer_loop, daemon=True
            ).start()

        except Exception as e:
            logger.critical(
                f"A critical error occurred during server startup: {e}", exc_info=True
            )
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
        if not self.running:
            return
        logger.info("--- Server shutting down... ---")
        self.running = False
        if self.camera_system:
            self.camera_system.stop()
        if self.imu_system:
            self.imu_system.stop()
        if self.depth_system:
            self.depth_system.stop()
        if self.thruster_system:
            self.thruster_system.safe_shutdown()
        logger.info("--- Shutdown complete. ---")

    def _noise_streamer_loop(self):
        """Fallback video streamer that sends random noise when camera fails."""
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info(f"Noise streamer started, sending to {self.client_ip}:{VIDEO_PORT}")
        frame_id = 0
        resolution = (640, 480)
        quality = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
        max_packet_size = 60000
        while self.running:
            try:
                noise_frame = np.random.randint(
                    0, 255, (resolution[1], resolution[0], 3), dtype=np.uint8
                )
                result, encoded_frame = cv2.imencode(".jpg", noise_frame, quality)
                if not result:
                    continue
                data = encoded_frame.tobytes()
                num_chunks = (len(data) + max_packet_size - 1) // max_packet_size
                for i in range(num_chunks):
                    chunk = data[i * max_packet_size : (i + 1) * max_packet_size]
                    header = struct.pack("!HHH", frame_id, num_chunks, i)
                    udp_socket.sendto(header + chunk, (self.client_ip, VIDEO_PORT))
                frame_id = (frame_id + 1) % 65535
                time.sleep(1 / 24)
            except Exception as e:
                logger.error(f"Error in noise streamer loop: {e}")
                time.sleep(1)
        udp_socket.close()
        logger.info("Noise streamer stopped.")

    def _combined_data_streamer_loop(self):
        """Streams combined IMU orientation and depth data."""
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info(
            f"Combined data streamer started, sending to {self.client_ip}:{IMU_PORT}"
        )
        while self.running:
            try:
                # Get latest data from both systems
                imu_data = self.imu_system.get_latest_data()
                depth_data = None
                if self.depth_system and self.depth_system.initialized:
                    depth_data = self.depth_system.get_latest_data()

                # Create combined data structure
                combined_data = {
                    "timestamp": time.time(),
                    "orientation": imu_data,
                    "depth": depth_data,
                }

                payload = json.dumps(combined_data).encode("utf-8")
                udp_socket.sendto(payload, (self.client_ip, IMU_PORT))
                time.sleep(1 / 50)  # 50Hz update rate

            except Exception as e:
                logger.error(f"Error in combined data streamer loop: {e}")
                time.sleep(1)
        udp_socket.close()
        logger.info("Combined data streamer stopped.")

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
                if not line:
                    continue
                msg = json.loads(line.decode("utf-8"))
                command = msg.get("command")

                if command == "control":
                    # Check if thruster system is available before using it
                    if self.thruster_system and self.thruster_system.initialized:
                        self.thruster_system.apply_control(
                            msg["thruster_pulses"],
                            msg["light_pulse"],
                            0,  # Duration is now handled client-side
                        )
                    else:
                        # Log that the command is being ignored
                        logger.debug(
                            "PCA9685 chip not detected! Ignoring thruster command."
                        )

                elif command == "reset_orientation":
                    # Reset both IMU orientation and depth reference
                    self.imu_system.reset_orientation()
                    if self.depth_system and self.depth_system.initialized:
                        self.depth_system.reset_depth()
                    logger.info(
                        f"Rx from {addr}: Command to reset IMU orientation and depth reference."
                    )

                else:
                    logger.warning(f"Rx from {addr}: Unknown command '{command}'")

            except socket.timeout:
                logger.warning(
                    f"CONTROL TIMEOUT: No packet received for {CONTROL_TIMEOUT}s. Setting thrusters to neutral."
                )
                # Also check here before setting to neutral
                if self.thruster_system and self.thruster_system.initialized:
                    self.thruster_system.set_neutral()

            except json.JSONDecodeError:
                logger.error("Received invalid JSON from client.")
            except Exception as e:
                if self.running:
                    logger.error(f"Error in UDP control loop: {e}", exc_info=True)
        control_socket.close()


if __name__ == "__main__":
    # Setup command-line argument parser
    parser = argparse.ArgumentParser(description="Submarine Server with Depth Sensing")
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="The IP address for the server to listen on. Default is 0.0.0.0 (all interfaces).",
    )
    parser.add_argument(
        "--client",
        type=str,
        default="192.168.2.1",
        help="The IP address of the client to send data to (Client IP). Default is 192.168.2.1.",
    )
    parser.add_argument(
        "--surface-depth",
        type=float,
        default=10.0,
        help="Initial depth in cm to consider as surface reference. Default is 10.0 cm.",
    )
    parser.add_argument(
        "--pressure-units",
        type=str,
        choices=["mbar", "psi", "atm", "torr"],
        default="mbar",
        help="Units for pressure readings. Default is mbar.",
    )
    parser.add_argument(
        "--depth-bus",
        type=int,
        default=6,
        help="I2C bus number for the depth sensor. Default is 6.",
    )
    args = parser.parse_args()

    # Pass the parsed arguments to the server instance
    server = SubmarineServer(
        host_ip=args.host,
        client_ip=args.client,
        surface_depth_cm=args.surface_depth,
        pressure_units=args.pressure_units,
        depth_bus=args.depth_bus,
    )
    try:
        server.start()
    finally:
        server.shutdown()
