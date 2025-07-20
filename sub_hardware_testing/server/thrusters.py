# File: thrusters.py
import time
import logging

try:
    from pca9685 import PCA9685
except ImportError:
    logging.critical("Failed to import pca9685. Is the library installed? Using mock interface.")
    # Define a mock class that accurately reflects the real library's API
    class PCA9685:
        def __init__(self): 
            # The real library uses a .pwm attribute that can be assigned to.
            self.pwm = [0]*16
            logging.info("Mock PCA: Initialized.")
        def set_pwm_frequency(self, freq): 
            logging.info(f"Mock PCA: Freq set to {freq}Hz")
        def output_enable(self): 
            logging.info("Mock PCA: Output enabled.")
        def output_disable(self): 
            logging.info("Mock PCA: Output disabled.")

class ThrusterController:
    """
    Manages the PCA9685 board to control 6 thrusters and 1 light.
    Handles safe startup and shutdown procedures.
    THIS VERSION USES DIRECT PULSE WIDTH ASSIGNMENT.
    """
    NEUTRAL_PULSE = 1500
    LIGHT_OFF_PULSE = 1100
    THRUSTER_CHANNELS = range(6)
    LIGHT_CHANNEL = 9

    def __init__(self):
        logging.info("Initializing Thruster Controller (PCA9685)...")
        self.pca = PCA9685()
        self.pca.set_pwm_frequency(50)
        # Set all outputs to a known safe state on startup
        self.set_neutral()
        self.pca.output_enable()
        logging.info("Thruster ESCs initialized. Audible sound should occur now.")

    def set_neutral(self):
        """Sets all thrusters to neutral and turns off the light by directly assigning pulse values."""
        for ch in self.THRUSTER_CHANNELS:
            # Directly assign the pulse width value. This is the correct logic.
            self.pca.pwm[ch] = self.NEUTRAL_PULSE
            
        self.pca.pwm[self.LIGHT_CHANNEL] = self.LIGHT_OFF_PULSE
        time.sleep(0.05) # Allow time for commands to process

    def apply_control(self, thruster_pulses, light_pulse, duration):
        """
        Applies specified pulse values directly to the PCA9685 board's channels.

        Args:
            thruster_pulses (list[int]): A list of 6 integer pulse values (e.g., 1100-1900).
            light_pulse (int): The integer pulse value for the light.
            duration (float): The time in seconds to hold these values.
        """
        if len(thruster_pulses) != 6:
            logging.error(f"Invalid thruster pulse count: {len(thruster_pulses)}. Expected 6.")
            return

        # Assign each thruster pulse directly to its channel.
        for i, pulse in enumerate(thruster_pulses):
            self.pca.pwm[self.THRUSTER_CHANNELS[i]] = pulse
        
        # Assign the light pulse directly.
        self.pca.pwm[self.LIGHT_CHANNEL] = light_pulse

        # The client-server loop depends on this sleep to maintain the command rate.
        time.sleep(duration)
        
    def safe_shutdown(self):
        """Sets all outputs to a safe state and disables the PCA9685 driver."""
        logging.info("Executing safe thruster shutdown...")
        self.set_neutral()
        self.pca.output_disable()
        logging.info("Thruster shutdown complete.")
