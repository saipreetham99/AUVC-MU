#!/usr/bin/env python3
"""
Fire webcam frames over UDP.
Default client IP 198.168.2.1, port 5005.
"""

import cv2
import socket
import struct
import argparse

CHUNK = 60_000                 # keep packets < 65 507 bytes
PORT  = 5005

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--client', default='198.168.2.1',
                    help='where to throw the packets')
    ap.add_argument('--source', type=int, default=0,
                    help='cv2.VideoCapture index')
    ap.add_argument('--quality', type=int, default=70,
                    help='JPEG quality (0‑100)')
    ap.add_argument('--chunk', type=int, default=CHUNK)
    args = ap.parse_args()

    cam  = cv2.VideoCapture(args.source)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    enc_param = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
    fid = 0

    while True:
        ok, frame = cam.read()
        if not ok:
            break

        ok, buf = cv2.imencode('.jpg', frame, enc_param)
        if not ok:
            continue

        data   = buf.tobytes()
        blocks = (len(data) - 1) // args.chunk + 1

        for idx in range(blocks):
            start = idx * args.chunk
            part  = data[start:start + args.chunk]
            header = struct.pack('!HHH', fid & 0xFFFF, blocks, idx)
            sock.sendto(header + part, (args.client, PORT))

        fid += 1

if __name__ == '__main__':
    main()

