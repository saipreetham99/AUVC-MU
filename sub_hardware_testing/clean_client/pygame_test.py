import pygame
import time

DEADZONE = 0.8

# Map button IDs to names
buttons = {
    0: "BUTTON_A",
    1: "BUTTON_B",
    2: "BUTTON_X",
    3: "BUTTON_Y",
    4: "BACK",
    5: "XBOX_BUTTON",
    6: "START",
    7: "LEFT_STICK_PRESS",
    8: "RIGHT_STICK_PRESS",
    9: "LB",
    10: "RB",
    11: "D_HAT_UP",
    12: "D_HAT_DOWN",
    13: "D_HAT_LEFT",
    14: "D_HAT_RIGHT",
    15: "MIDDLE_BUTTON"
}

# Map axis IDs to names
axes = {
    0: "LEFT_STICK_X",
    1: "LEFT_STICK_Y",
    2: "RIGHT_STICK_X",
    3: "RIGHT_STICK_Y",
    4: "LT",
    5: "RT",
}

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Name:", joystick.get_name())
print("Axes:", joystick.get_numaxes())
print("Buttons:", joystick.get_numbuttons())
print("Hats:", joystick.get_numhats())

while True:
    time.sleep(0.01)
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button in buttons:
                button_name = buttons[event.button]
                print(f"[{button_name}] Button {event.button} pressed")
            else:
                print(f"[UNKNOWN_BUTTON] Button {event.button} pressed")

        elif event.type == pygame.JOYAXISMOTION:
            value = event.value
            if abs(value) > DEADZONE:
                if event.axis in axes:
                    axis_name = axes[event.axis]
                    print(f"[{axis_name}] Axis {event.axis} moved to {value:.2f}")
                else:
                    print(f"[UNKNOWN_AXIS] Axis {event.axis} moved to {value:.2f}")

        elif event.type == pygame.JOYHATMOTION:
            print(f"Hat moved to {event.value}")

