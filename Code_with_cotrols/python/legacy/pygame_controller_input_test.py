import pygame
import sys

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Check for joysticks
if pygame.joystick.get_count() == 0:
    print("No controller connected.")
    sys.exit()

# Use the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Controller connected: {joystick.get_name()}")

# Mapping helpers (Xbox controller standard layout)
BUTTONS = {
    0: "A",
    1: "B",
    2: "X",
    3: "Y",
    4: "ViewButton",
    5: "XboxButton",
    6: "MenuButton",
    7: "LeftStick",
    8: "RightStick",
    9: "LB",
    10: "RB",
    11: "DPad: Up",
    12: "DPad: Down",
    13: "DPad: Left",
    14: "DPad: Right",
    15: "Back",
}

AXES = {
    0: "Left Stick X",
    1: "Left Stick Y",
    2: "Right Stick X",
    3: "Right Stick Y",
    4: "LT",
    5: "RT"
}

# Main loop
clock = pygame.time.Clock()
running = True
print("Listening for controller input...")

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Button pressed
        elif event.type == pygame.JOYBUTTONDOWN:
            btn = BUTTONS.get(event.button, f"Unknown ({event.button})")
            print(f"[Button Down] {btn} : {event.button}")

        # Button released
        elif event.type == pygame.JOYBUTTONUP:
            btn = BUTTONS.get(event.button, f"Unknown ({event.button})")
            print(f"[Button Up] {btn}: {event.button}")

        # Analog stick or trigger moved
        elif event.type == pygame.JOYAXISMOTION:
            axis_name = AXES.get(event.axis, f"Axis {event.axis}")
            value = round(event.value, 2)
            # Optional threshold to ignore drift
            if abs(value) > 0.1:
                print(f"[Axis Motion] {axis_name}[{event.axis}]: {value}")

        # Hat (D-pad)
        elif event.type == pygame.JOYHATMOTION:
            print(f"[D-Pad] {event.value}")  # (x, y): e.g., (1, 0) = right

    clock.tick(60)

pygame.quit()
