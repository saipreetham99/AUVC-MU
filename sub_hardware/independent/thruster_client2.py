import pygame
import socket
import struct
import time
import sys
import os
import configparser
import argparse

# Constants
NEUTRAL = 1500
AMP, AMP_STEP = 100, 100
LIGHT_OFF = 1100
LIGHT_ON = 1900
AMP_MIN, AMP_MAX = 100, 400
DEAD, LOOP_DT = 0.6, 0.02

# Joystick mapping — tweak if needed
SURGE_AXIS, STRAFE_AXIS, HEAVE_AXIS, YAW_AXIS = 1, 0, 3, 2

BTN_LB, BTN_RB = 9, 10
BTN_BACK, BTN_START = 4, 6
BTN_HAT_UP, BTN_HAT_DOWN = 11, 12
BTN_HAT_RIGHT, BTN_HAT_LEFT = 14, 13

AMP_COOLDOWN, last_amp = 0.25, 0
SHOW_STATUS, STATUS_DT, last_stat = True, 0.5, 0
REQUIRE_BACK = False
armed = False
light = False


def clamp(x):
    return max(-1.0, min(1.0, x))


def to_pwm(x):
    return int(NEUTRAL + x * AMP)


def send_neutral(sock, addr):
    sock.sendto(struct.pack("<6H", *(NEUTRAL,) * 6), addr)


def safe_shutdown(sock, addr):
    print("\nNeutral shutdown…")
    send_neutral(sock, addr)
    time.sleep(0.05)
    send_neutral(sock, addr)
    sock.close()
    pygame.quit()


def main():
    global AMP, armed, light, last_amp, last_stat

    # --- Load Credentials ---
    parser = argparse.ArgumentParser(description="ROV Thruster Control Client")
    parser.add_argument('--wifi', action='store_true',
                        help='Use WiFi IP/network settings instead of LAN.')
    args = parser.parse_args()

    mode = 'wifi' if args.wifi else 'lan'
    # The client needs to know the server's ports, so we read the server config file
    config_path = os.path.expanduser('~/.rov_server_creds')

    config = configparser.ConfigParser()
    if not os.path.exists(config_path) or not config.read(config_path):
        sys.exit(f"✗ ERROR: Config file not found or is empty at '{
                 config_path}'")

    try:
        creds = config[mode]
        rov_ip = creds['rov_ip']
        thruster_port = config.getint('DEFAULT', 'thruster_port')
        addr = (rov_ip, thruster_port)
        print(f"✓ Loaded '{mode}' settings from '{config_path}'")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing section or key in config file: {e}")
    # --- End Load Credentials ---

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("No gamepad found")

    pad = pygame.joystick.Joystick(0)
    pad.init()
    print(f"Gamepad: {pad.get_name()} ➜ {addr[0]}:{addr[1]}")

    send_neutral(sock, addr)
    t_next = time.time()

    try:
        while True:
            for e in pygame.event.get():
                if e.type == pygame.JOYBUTTONDOWN:
                    if e.button == BTN_START:
                        armed = not armed
                        print(f"[{'ARM' if armed else 'SAFE'}]")

                    elif e.button == BTN_HAT_RIGHT:
                        light = True
                        print("[LIGHT ON]")

                    elif e.button == BTN_HAT_LEFT:
                        light = False
                        print("[LIGHT OFF]")

                    elif e.button == BTN_HAT_UP and time.time() - last_amp > AMP_COOLDOWN:
                        AMP = min(AMP + AMP_STEP, AMP_MAX)
                        last_amp = time.time()
                        print("AMP", AMP)

                    elif e.button == BTN_HAT_DOWN and time.time() - last_amp > AMP_COOLDOWN:
                        AMP = max(AMP - AMP_STEP, AMP_MIN)
                        last_amp = time.time()
                        print("AMP", AMP)

            surge = -pad.get_axis(SURGE_AXIS)
            strafe = pad.get_axis(STRAFE_AXIS)
            heave = -pad.get_axis(HEAVE_AXIS)
            yaw = pad.get_axis(YAW_AXIS)

            surge = 0 if abs(surge) < DEAD else surge
            strafe = 0 if abs(strafe) < DEAD else strafe
            heave = 0 if abs(heave) < DEAD else heave
            yaw = 0 if abs(yaw) < DEAD else yaw

            rb, lb = pad.get_button(BTN_RB), pad.get_button(BTN_LB)
            if rb ^ lb:
                yaw = 1.0 if rb else -1.0

            if REQUIRE_BACK and not pad.get_button(BTN_BACK):
                surge = strafe = heave = yaw = 0
            if not armed:
                surge = strafe = heave = yaw = 0

            if pad.get_button(BTN_BACK):
                raise KeyboardInterrupt(
                    "Back button pressed. Exiting cleanly...")

            fl = clamp(surge - strafe - yaw)
            fr = clamp(surge + strafe + yaw)
            rl = clamp(surge + strafe - yaw)
            rr = clamp(surge - strafe + yaw)
            v1 = clamp(heave)
            v2 = clamp(-heave)
            light_pwm = LIGHT_ON if light else LIGHT_OFF

            sock.sendto(
                struct.pack(
                    "<7H",
                    to_pwm(fl), to_pwm(fr), to_pwm(rl), to_pwm(rr),
                    to_pwm(v1), to_pwm(v2), light_pwm,
                ),
                addr,
            )

            if SHOW_STATUS and time.time() - last_stat > STATUS_DT:
                print(
                    f"[{'ARM' if armed else 'SAFE'}] "
                    f"F:{surge:+.2f} S:{strafe:+.2f} H:{heave:+.2f} Y:{yaw:+.2f} "
                    f"L:{'ON' if light else 'OFF'}",
                    end="\r", flush=True,
                )
                last_stat = time.time()

            t_next += LOOP_DT
            delay = t_next - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                t_next = time.time()

    except KeyboardInterrupt as e:
        print(f"\n{e}")
    finally:
        safe_shutdown(sock, addr)


if __name__ == "__main__":
    main()
