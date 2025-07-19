import time
import numpy as np
import signal
import sys
from mj_client import MjClient

"""
This script applies forces to a Unity-based MjScene simulation.
It includes proper handling of Ctrl+C to ensure forces are zeroed before exit.
"""

# Configure these to match your actual body and site IDs in Unity
BODY_ID = 1  # Update this to match your Unity scene's body ID for "bfloat1"
SITE_ID = 0  # Update this to match your Unity scene's site ID for "sfloat1"

# Create the client
client = MjClient(host="127.0.0.1", port=60000)

# Define the force to apply
my_force = np.array([0.0, 0.0, 0.05])  # Force along global X-axis
my_torque = np.array([0.0, 0.0, 0.0])   # No torque

my_actuator_force = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])  # Force along global X-axis

# Function to send zero force and exit cleanly
def cleanup_and_exit(signal=None, frame=None):
    print("\nReceived exit signal. Cleaning up...")
    # Send a zero force command to stop any forces
    try:
        # Command 3 = Apply force, with all zeros
        zero_force_cmd = np.array([2.0, float(BODY_ID), float(SITE_ID),
                                  0.0, 0.0, 0.0,  # Zero force
                                  0.0, 0.0, 0.0], # Zero torque
                                  dtype=np.float32)
        client.send_request(zero_force_cmd)
        print("Successfully zeroed out forces.")
    except Exception as e:
        print(f"Error during cleanup: {e}")

    print("Exiting...")
    sys.exit(0)

# Register signal handler for Ctrl+C
signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# Initial check - make sure we can connect to the server
print("Testing connection to Unity simulation...")
try:
    response = client.send_request([0.0])  # Command 0 = Get model info
    if len(response) > 0:
        print("Connected successfully!")

        # Print model info if available
        if len(response) >= 6:
            print(f"Model info:")
            print(f"  Time: {response[0]}")
            print(f"  Position coordinates (nq): {int(response[1])}")
            print(f"  Velocity coordinates (nv): {int(response[2])}")
            print(f"  Actuator activations (na): {int(response[3])}")
            print(f"  Control inputs (nu): {int(response[4])}")
            print(f"  Number of bodies: {int(response[5])}")
    else:
        print("Warning: Connected but received empty response")
except Exception as e:
    print(f"Connection failed: {e}")
    sys.exit(1)

# Run for 30 seconds
duration = 30.0
start_time = time.time()

print(f"\nApplying force [{my_force[0]}, {my_force[1]}, {my_force[2]}] to body {BODY_ID} at site {SITE_ID}")
print(f"Running for {duration} seconds. Press Ctrl+C to stop...\n")

try:
    # Main loop
    while time.time() - start_time < duration:
        # Create force command
        # Command format: [2.0, body_id, site_id, fx, fy, fz, tx, ty, tz]
        force_cmd = np.array([2.0, float(BODY_ID), float(SITE_ID),
                              my_actuator_force[0], my_actuator_force[1], my_actuator_force[2],
                              my_actuator_force[3], my_actuator_force[4], my_actuator_force[5]],
                              dtype=np.float32)

        # Send force command
        response = client.send_request(force_cmd)

        # Check if successful
        success = len(response) > 0 and response[0] > 0.5

        # Get current state periodically
        if int(time.time() - start_time) % 5 == 0:
            try:
                state_cmd = np.array([1.0], dtype=np.float32)  # Command 1 = Get state
                state_response = client.send_request(state_cmd)

                print(f"t={time.time() - start_time:.1f}s: Force applied, success={success}")

                if len(state_response) > 1:
                    print(f"  Current state: {state_response[:min(5, len(state_response))]}")
            except:
                pass

        # Sleep to control update rate (50Hz)
        time.sleep(0.02)

except Exception as e:
    print(f"Error during execution: {e}")
finally:
    # Always clean up properly when done
    cleanup_and_exit()
