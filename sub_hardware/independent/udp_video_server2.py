#!/usr/bin/env python3
"""
Fire webcam frames over UDP.
Reads client IP and port from a config file.
"""
import cv2
import socket
import struct
import argparse
import os
import configparser
import sys
import time

CHUNK = 60_000  # keep packets < 65_507 bytes

def main():
    # --- Load Credentials ---
    parser = argparse.ArgumentParser(description="UDP Video Stream Server")
    parser.add_argument('--wifi', action='store_true',
                        help='Use WiFi IP/network settings instead of LAN.')
    parser.add_argument('--source', type=int, default=0,
                        help='cv2.VideoCapture index')
    parser.add_argument('--quality', type=int, default=None, # Default to None to use config
                        help='JPEG quality (0-100)')
    parser.add_argument('--chunk', type=int, default=CHUNK)
    args = parser.parse_args()

    mode = 'wifi' if args.wifi else 'lan'
    config_path = os.path.expanduser('~/.rov_server_creds')
    config = configparser.ConfigParser()

    if not os.path.exists(config_path) or not config.read(config_path):
        sys.exit(f"✗ ERROR: Config file not found or is empty at '{config_path}'")

    try:
        creds = config[mode]
        # The server sends to the client_ip
        client_ip = creds['client_ip']
        port = config.getint('DEFAULT', 'video_port')
        # Use command line quality if provided, otherwise use config file
        quality = args.quality if args.quality is not None else config.getint('DEFAULT', 'video_quality', fallback=75)
        print(f"✓ Loaded '{mode}' settings from '{config_path}'")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing section or key in config file: {e}")
    # --- End Load Credentials ---

    cam = cv2.VideoCapture(args.source)
    if not cam.isOpened():
        sys.exit(f"✗ ERROR: Cannot open camera source {args.source}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    enc_param = [cv2.IMWRITE_JPEG_QUALITY, quality]
    fid = 0

    print(f"Streaming video to {client_ip} on port {port} with {quality}% quality.")
    print("Press Ctrl+C to stop.")

    while True:
        ok, frame = cam.read()
        if not ok:
            print("Camera read failed, stopping.")
            break

        ok, buf = cv2.imencode('.jpg', frame, enc_param)
        if not ok:
            print("JPEG encoding failed.")
            continue

        data = buf.tobytes()
        blocks = (len(data) - 1) // args.chunk + 1

        for idx in range(blocks):
            start = idx * args.chunk
            part = data[start:start + args.chunk]
            header = struct.pack('!HHH', fid & 0xFFFF, blocks, idx)
            sock.sendto(header + part, (client_ip, port))
        
        fid += 1
        # A small sleep can prevent overwhelming the network
        time.sleep(0.01)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nServer is shutting down.")
