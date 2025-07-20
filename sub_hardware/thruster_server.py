#!/usr/bin/python3
import time
import signal
import socket
import struct
from pca9685 import PCA9685

NEUTRAL = 1500          # µs pulse
RANGE = 400           # 1500 ±400 → 1100-1900
PWM_CH = range(6)
LOOP_DT = 0.02          # 50 Hz
TIMEOUT = 0.25          # if nothing arrives, go limp
UDP_PORT = 5005

pca = PCA9685()
pca.set_pwm_frequency(50)
pca.output_enable()


def all_neutral():
    for ch in PWM_CH:
        pca.pwm[ch] = NEUTRAL


def shutdown(*_):
    all_neutral()
    raise SystemExit


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGHUP, shutdown)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
sock.settimeout(LOOP_DT)

last_rx = time.time()
fmt = "<6H"              # 6×uint16, little-endian

while True:
    try:
        data, _ = sock.recvfrom(12)
        pulses = struct.unpack(fmt, data)   # tuple of 6 ints
        for ch, val in zip(PWM_CH, pulses):
            pca.pwm[ch] = val
        last_rx = time.time()
    except socket.timeout:
        if time.time() - last_rx > TIMEOUT:
            all_neutral()
