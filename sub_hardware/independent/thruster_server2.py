#!/usr/bin/env python3
"""
Receives 7-channel UDP packets (6 thrusters, 1 light) and controls them
via a PCA9685 board. Includes signal handling and a safety timeout.
"""
import time
import signal
import socket
import struct
import os
import configparser
import sys
import argparse
from pca9685 import PCA9685

# --- Configuration ---
NEUTRAL_PULSE = 1500      # Pulse width in µs for neutral thruster position
LIGHT_OFF_PULSE = 1100    # Pulse width in µs to turn the light off

# --- Channel Assignments ---
THRUSTER_CHANNELS = range(6)  # Channels 0 through 5
LIGHT_CHANNEL = 9            # Channel 9 for the light

# --- Timing and Safety ---
LOOP_DT = 0.02          # 50 Hz loop rate for the socket timeout
TIMEOUT = 0.5           # Seconds of no packets before safety shutdown

# --- PCA9685 Setup ---
pca = PCA9685()
pca.set_pwm_frequency(50)  # Set PWM frequency to 50Hz
pca.output_enable()


def safe_shutdown(*_):
    """
    Called on signal (Ctrl+C) or timeout. Sets all outputs to a safe state
    and exits the program.
    """
    print("\nExecuting safe shutdown...")
    # Set all thrusters to neutral
    for ch in THRUSTER_CHANNELS:
        pca.pwm[ch] = NEUTRAL_PULSE
    # Turn the light off
    pca.pwm[LIGHT_CHANNEL] = LIGHT_OFF_PULSE
    # Allow a moment for commands to process
    time.sleep(0.05)
    # Disable the PCA9685 output driver
    pca.output_disable()
    print("Shutdown complete.")
    raise SystemExit


# --- Signal Handling for Clean Exit ---
# Register the shutdown function for SIGINT (Ctrl+C) and SIGHUP (hangup)
signal.signal(signal.SIGINT, safe_shutdown)
signal.signal(signal.SIGHUP, safe_shutdown)


def main():
    # --- Load Credentials ---
    parser = argparse.ArgumentParser(description="ROV Thruster Server")
    parser.add_argument('--wifi', action='store_true',
                        help='Use WiFi IP/network settings instead of LAN.')
    args = parser.parse_args()

    mode = 'wifi' if args.wifi else 'lan'
    config_path = os.path.expanduser('~/.rov_server_creds')

    config = configparser.ConfigParser()
    if not os.path.exists(config_path) or not config.read(config_path):
        sys.exit(f"✗ ERROR: Config file not found or is empty at '{
                 config_path}'")

    try:
        udp_port = config.getint('DEFAULT', 'thruster_port')
        print(f"✓ Loaded '{mode}' settings from '{config_path}'")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing section or key in config file: {e}")
    # --- End Load Credentials ---

    # --- Socket Setup ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind to all available interfaces on the specified port
    sock.bind(("", udp_port))
    # Set a short timeout to make the recvfrom call non-blocking
    sock.settimeout(LOOP_DT)

    print(f"Server listening on port {udp_port}...")
    print("Press Ctrl+C to exit.")

    # --- Main Loop ---
    last_rx_time = time.time()
    # The format string for our packet: 7 little-endian unsigned shorts
    packet_format = "<7H"
    # The expected size of the packet in bytes (7 * 2 bytes)
    packet_size = 14

    try:
        while True:
            try:
                # Attempt to receive a packet
                data, _ = sock.recvfrom(packet_size)

                # Ensure the packet is the correct size before unpacking
                if len(data) == packet_size:
                    # Unpack the data into a tuple of 7 pulse widths
                    pulses = struct.unpack(packet_format, data)
                    fl, fr, rl, rr, v1, v2, light_pwm = pulses

                    # Assign pulse values to the thruster channels
                    pca.pwm[0] = fl
                    pca.pwm[1] = fr
                    pca.pwm[2] = rl
                    pca.pwm[3] = rr
                    pca.pwm[4] = v1
                    pca.pwm[5] = v2

                    # Assign pulse value to the light channel
                    pca.pwm[LIGHT_CHANNEL] = light_pwm

                    # Update the timestamp of the last valid packet
                    last_rx_time = time.time()

            except socket.timeout:
                # This exception is expected every LOOP_DT seconds if no data arrives.
                # We check if the time since the last good packet exceeds our safety TIMEOUT.
                if time.time() - last_rx_time > TIMEOUT:
                    print(f"\nNo packet received for {
                          TIMEOUT}s. Thruster are set to Neutral")
                    # safe_shutdown()
                    for ch in range(6):
                        pca.pwm[ch] = NEUTRAL_PULSE
                    # The line below is not strictly needed as safe_shutdown exits,
                    # but it's good practice to reset the timer if it didn't.
                    last_rx_time = time.time()

    except SystemExit:
        # This catches the exit call from the signal handler to prevent an error message.
        pass
    finally:
        # Ensure the socket is closed, regardless of how the loop exits.
        sock.close()


if __name__ == '__main__':
    main()
