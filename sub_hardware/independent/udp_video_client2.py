#!/usr/bin/env python3
"""
Receives UDP video stream packets, reconstructs frames, and displays
them in a window. Corrected for macOS GUI event handling.
"""
import cv2
import socket
import struct
import numpy as np
import time
from collections import defaultdict
import os
import configparser
import sys
import argparse

FRAME_TIMEOUT = 0.5  # seconds before discarding an incomplete frame

def main():
    # --- Argument and Configuration Loading ---
    parser = argparse.ArgumentParser(description="UDP Video Stream Client")
    parser.add_argument('--wifi', action='store_true', help='Use WiFi IP/network settings instead of LAN.')
    args = parser.parse_args()

    mode = 'wifi' if args.wifi else 'lan'
    config_path = os.path.expanduser('~/.rov_server_creds')
    config = configparser.ConfigParser()

    if not os.path.exists(config_path) or not config.read(config_path):
        sys.exit(f"✗ ERROR: Config file not found or is empty at '{config_path}'")

    try:
        creds = config[mode]
        server_ip = creds['rov_ip']
        port = config.getint('DEFAULT', 'video_port')
        print(f"✓ Loaded '{mode}' settings from '{config_path}'")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing section or key in config file: {e}")

    # --- Socket Initialization ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    sock.settimeout(1.0)

    print(f"--> Listening for video stream from {server_ip} on port {port}")
    print("--> A video window should appear. Press ESC in the window to quit.")

    buffers = defaultdict(lambda: {'parts': {}, 'timestamp': 0, 'total_chunks': 0})
    
    while True:
        try:
            # --- Packet Reception ---
            try:
                packet, addr = sock.recvfrom(65535)
            except socket.timeout:
                # Purge stale frames if no new packets arrive
                now = time.time()
                for fid in list(buffers.keys()):
                    if now - buffers[fid]['timestamp'] > FRAME_TIMEOUT:
                        del buffers[fid]
                # THE CRITICAL CHANGE IS HERE: We must call waitKey in every loop iteration
                # to give the GUI a chance to process events, even if no packet is received.
                if cv2.waitKey(1) == 27:
                    break
                continue

            if addr[0] != server_ip:
                continue

            # --- Packet Parsing and Frame Buffering ---
            header = packet[:6]
            payload = packet[6:]
            frame_id, total_chunks, chunk_id = struct.unpack('!HHH', header)

            frame_buffer = buffers[frame_id]
            frame_buffer['parts'][chunk_id] = payload
            frame_buffer['timestamp'] = time.time()
            frame_buffer['total_chunks'] = total_chunks
            
            # --- Frame Assembly and Display ---
            if len(frame_buffer['parts']) == frame_buffer['total_chunks']:
                ordered_parts = [frame_buffer['parts'][i] for i in range(total_chunks)]
                jpg_data = b''.join(ordered_parts)
                
                frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)

                if frame is not None:
                    cv2.imshow('UDP Video Stream', frame)
                else:
                    print(f"! Frame {frame_id} failed to decode.")
                
                del buffers[frame_id]

            # THE CRITICAL CHANGE IS HERE: Call waitKey(1) on EVERY loop.
            # This processes GUI events, draws the window, and checks for the ESC key.
            if cv2.waitKey(1) == 27:  # 27 is the ASCII for the ESC key
                print("✓ ESC key pressed. Exiting.")
                break

        except KeyboardInterrupt:
            break

    print("\n✓ Client shutting down.")
    sock.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
