#!/usr/bin/env python3
"""
Pull UDP packets, rebuild frames, show video.
Default server filter 198.168.2.11, port 5005.
"""

import cv2
import socket
import struct
import numpy as np
import argparse
import time
from collections import defaultdict

PORT = 5004
TIMEOUT = 0.5        # seconds before tossing half‑baked frame

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--server', default='198.168.2.11',
                    help='accept packets only from this IP (use "any" to allow all)')
    ap.add_argument('--timeout', type=float, default=TIMEOUT)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', PORT))
    sock.settimeout(1.0)

    buffers = defaultdict(lambda: {'total': None, 'parts': {}, 'ts': time.time()})

    while True:
        try:
            packet, addr = sock.recvfrom(65_535)
        except socket.timeout:
            continue
        if args.server != 'any' and addr[0] != args.server:
            continue

        fid, total, idx = struct.unpack('!HHH', packet[:6])
        payload = packet[6:]

        buf = buffers[fid]
        buf['ts'] = time.time()
        if buf['total'] is None:
            buf['total'] = total
        buf['parts'][idx] = payload

        # got all chunks?
        if len(buf['parts']) == buf['total']:
            parts = [buf['parts'][i] for i in range(buf['total'])]
            jpg   = b''.join(parts)
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow('UDP Stream', frame)
                if cv2.waitKey(1) == 27:   # ESC to quit
                    break
            del buffers[fid]

        # purge stale frames
        now = time.time()
        dead = [k for k, v in buffers.items() if now - v['ts'] > args.timeout]
        for k in dead:
            del buffers[k]

    sock.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

