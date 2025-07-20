# File: cam.py
import cv2
import socket
import struct
import time
import threading
import logging

class CameraStreamer:
    """
    Captures video frames, encodes them as JPEG, splits them into chunks,
    and streams them over UDP to a specified client.
    """
    def __init__(self, client_ip, port, resolution=(640, 480), quality=70, fps=24):
        self.client_ip = client_ip
        self.port = port
        self.resolution = resolution
        self.quality = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        self.max_packet_size = 60000  # Max size of a UDP packet payload

        self.running = False
        self._thread = None
        self._frame_id = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            logging.error("Cannot open camera. Is it connected?")
            raise IOError("Cannot open camera")
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        logging.info(f"Camera initialized at {self.resolution} @ {fps}fps, quality={quality}")

    def start(self):
        """Starts the video streaming thread."""
        if self.running:
            logging.warning("Camera streamer is already running.")
            return
        self.running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()
        logging.info(f"Camera streaming started, sending to {self.client_ip}:{self.port}")

    def stop(self):
        """Stops the video streaming thread."""
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join()
        self.cap.release()
        self.sock.close()
        logging.info("Camera streaming stopped and resources released.")

    def _stream_loop(self):
        """The main loop for capturing and sending frames."""
        while self.running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    logging.warning("Failed to grab frame from camera.")
                    time.sleep(0.1)
                    continue

                # Encode the frame as JPEG
                result, encoded_frame = cv2.imencode('.jpg', frame, self.quality)
                if not result:
                    continue

                data = encoded_frame.tobytes()
                data_size = len(data)

                # Split data into chunks
                num_chunks = (data_size + self.max_packet_size - 1) // self.max_packet_size
                
                for i in range(num_chunks):
                    start = i * self.max_packet_size
                    end = start + self.max_packet_size
                    chunk = data[start:end]
                    
                    # Create header: Frame ID, Total Chunks, Chunk Index
                    header = struct.pack('!HHH', self._frame_id, num_chunks, i)
                    packet = header + chunk
                    
                    self.sock.sendto(packet, (self.client_ip, self.port))
                
                self._frame_id = (self._frame_id + 1) % 65535 # Wrap around to avoid overflow
                
                # Small sleep to yield CPU, actual FPS is limited by camera
                time.sleep(1/50) 

            except Exception as e:
                logging.error(f"Error in camera stream loop: {e}")
                # If a major error occurs, stop to prevent spamming logs
                if not self.cap.isOpened():
                    self.running = False
                    break
                time.sleep(1)
