#!/usr/bin/env python3
"""
UDP frame re‑assembly + AprilTag + YOLOv8 live view.
Add --record to save the plain stream (no overlays).  ESC quits.
"""

import cv2
import socket
import struct
import numpy as np
import argparse
import time
from collections import defaultdict
from datetime import datetime

from pupil_apriltags import Detector
from ultralytics import YOLO
import supervision as sv

# ------------------------------------------------------------------------
PORT    = 5004
TIMEOUT = 0.5   # toss partial frame after this many secs
RF_INT  = 3     # run YOLO every Nth frame
# ------------------------------------------------------------------------


def parse():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="192.168.2.11",
                    help='accept packets only from this IP (use "any" to allow all)')
    ap.add_argument("--timeout", type=float, default=TIMEOUT,
                    help="seconds before discarding a half‑finished frame")
    ap.add_argument("--weights", default="best.pt",
                    help="path to YOLOv8 weights")
    ap.add_argument("--conf", type=float, default=0.5,
                    help="YOLO confidence 0‑1")
    ap.add_argument("--record", action="store_true",
                    help="save the raw stream (no boxes/tags) to an .mp4 file")
    return ap.parse_args()


def main():
    opt = parse()

    # detectors
    tag_det     = Detector(families="tag36h11")
    yolo        = YOLO(opt.weights)
    box_annot   = sv.BoxAnnotator()
    label_annot = sv.LabelAnnotator()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", PORT))
    sock.settimeout(1.0)

    buffers = defaultdict(lambda: {"total": None, "parts": {}, "ts": time.time()})
    rf_id  = 0
    t0, frames = time.time(), 0
    writer = None            # <<< VideoWriter comes alive when first frame arrives

    print(f"[INFO] listening on *:{PORT}")

    while True:
        # ── read one UDP packet ──────────────────────────────────────────
        try:
            packet, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue

        if opt.server != "any" and addr[0] != opt.server:
            continue

        fid, total, idx = struct.unpack("!HHH", packet[:6])
        payload         = packet[6:]

        buf       = buffers[fid]
        buf["ts"] = time.time()
        if buf["total"] is None:
            buf["total"] = total
        buf["parts"][idx] = payload

        # ── frame complete? ──────────────────────────────────────────────
        if len(buf["parts"]) == buf["total"]:
            jpg   = b"".join(buf["parts"][i] for i in range(buf["total"]))
            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            del buffers[fid]
            if frame is None:
                continue

            raw_frame = frame.copy()   # <<< un‑touched copy for recording

            # AprilTag overlay (only on the copy we'll show)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            for tag in tag_det.detect(gray):
                pts = tag.corners.astype(int)
                for i in range(4):
                    cv2.line(frame, tuple(pts[i]), tuple(pts[(i + 1) % 4]),
                             (0, 255, 0), 2)
                cx, cy = map(int, tag.center)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(frame, f"id {tag.tag_id}",
                            (cx + 6, cy - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # YOLO every RF_INT frames
            if rf_id % RF_INT == 0:
                yres = yolo(frame, conf=opt.conf, max_det=1, verbose=False)[0]
                dets = sv.Detections.from_ultralytics(yres)
                frame = box_annot.annotate(frame, dets)
                frame = label_annot.annotate(frame, dets)
            rf_id += 1

            # FPS counter
            frames += 1
            if frames >= 20:
                fps    = frames / (time.time() - t0)
                t0, frames = time.time(), 0
            cv2.putText(frame,
                        f"FPS {fps:.1f}" if 'fps' in locals() else "FPS …",
                        (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2)

            # start writer lazily
            if opt.record and writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                ts     = datetime.now().strftime("%Y%m%d_%H%M%S")
                fname  = f"stream_{ts}.mp4"
                h, w   = raw_frame.shape[:2]
                writer = cv2.VideoWriter(fname, fourcc, 30.0, (w, h))
                print(f"[INFO] recording raw stream to {fname}")

            if writer is not None:
                writer.write(raw_frame)     # <<< save the clean frame

            # show annotated frame
            cv2.imshow("stream", frame)
            if cv2.waitKey(1) == 27:  # ESC
                break

        # purge stale buffers
        now = time.time()
        for k in [k for k, v in buffers.items() if now - v["ts"] > opt.timeout]:
            del buffers[k]

    # cleanup
    sock.close()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

