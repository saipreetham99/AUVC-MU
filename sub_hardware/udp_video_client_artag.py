#!/usr/bin/env python3
"""
Pull UDP packets, rebuild frames, show video + AprilTag overlay.
Default server filter 198.168.2.11, port 5005.
"""

import cv2
import socket
import struct
import numpy as np
import argparse
import time
from collections import defaultdict
import apriltag                                   # ← NEW

PORT = 5004
TIMEOUT = 0.5        # seconds before tossing half‑baked frame

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--server', default='198.168.2.11',
                    help='accept packets only from this IP (use "any" to allow all)')
    ap.add_argument('--timeout', type=float, default=TIMEOUT)
    args = ap.parse_args()

    # build tag detector once (cpu‑only, no fuss)
    opts = apriltag.DetectorOptions(families='tag36h11')   # ← NEW
    detector = apriltag.Detector(opts)                     # ← NEW

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
                # -------- AprilTag pass ----------
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        # ← NEW
                tags = detector.detect(gray)                          # ← NEW
                for t in tags:                                        # ← NEW
                    pts = t.corners.astype(int)                       # ← NEW
                    for i in range(4):                                # ← NEW
                        cv2.line(frame, tuple(pts[i]), tuple(pts[(i+1)%4]), (0,255,0), 2)
                    cX, cY = map(int, t.center)                       # ← NEW
                    cv2.circle(frame, (cX, cY), 4, (0,0,255), -1)
                    cv2.putText(frame, f'id {t.tag_id}', (cX+6, cY-6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                # ---------------------------------

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

