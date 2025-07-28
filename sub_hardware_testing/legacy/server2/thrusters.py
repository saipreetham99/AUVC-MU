# File: thrusters.py
# MODIFIED VERSION
import logging
import time
from pca9685 import PCA9685  # <-- Directly import your working driver

logger = logging.getLogger(__name__)

# --- Configuration Constants ---
NEUTRAL_PULSE = 1500
LIGHT_OFF_PULSE = 1100
THRUSTER_CHANNELS = range(6) # Channels 0-5
LIGHT_CHANNEL = 9            # Using channel 9 for the light

class ThrusterController:
    """
    A simplified controller that directly uses the pca9685.py driver.
    This class handles initializing the PCA9685 board and provides a simple
    interface for applying thruster and light control values.
    """
    def __init__(self, pwm_frequency=50):
        self.pca = None
        self.initialized = False
        logger.info("Initializing Thruster Controller (PCA9685)...")

        try:
            # --- Simplified Initialization Logic ---
            # 1. Create an instance of the driver
            self.pca = PCA9685()
            
            # 2. Set the required PWM frequency
            self.pca.set_pwm_frequency(pwm_frequency)
            
            # 3. Enable the output driver
            self.pca.output_enable()
            
            # 4. If all of the above succeeded, we are initialized.
            self.initialized = True
            logger.info("OK: Thruster controller (PCA9685) initialized successfully.")
            
        except Exception as e:
            # If any part of the setup fails, log the error and remain uninitialized.
            logger.error(f"Failed to initialize Thruster Controller (PCA9685): {e}", exc_info=True)
            logger.warning("Thruster control will be disabled.")
            self.initialized = False

    def apply_control(self, thruster_pulses, light_pulse, duration=0):
        """
        Applies the received pulse widths directly to the PCA9685 channels.
        The 'duration' argument is ignored to match the client's continuous stream.
        """
        if not self.initialized:
            return

        try:
            # Directly set the PWM values for each thruster channel
            for i, channel in enumerate(THRUSTER_CHANNELS):
                if i < len(thruster_pulses):
                    self.pca.pwm[channel] = thruster_pulses[i]
            
            # Set the light channel value
            self.pca.pwm[LIGHT_CHANNEL] = light_pulse

        except Exception as e:
            logger.error(f"Error applying thruster control: {e}")

    def set_neutral(self):
        """Sets all thrusters to their neutral position."""
        if not self.initialized:
            return
            
        logger.info("Setting all thrusters to neutral.")
        for channel in THRUSTER_CHANNELS:
            self.pca.pwm[channel] = NEUTRAL_PULSE
            
        # Also turn the light off as a safety measure
        self.pca.pwm[LIGHT_CHANNEL] = LIGHT_OFF_PULSE

    def safe_shutdown(self):
        """Gracefully shuts down the PCA9685 board."""
        if not self.initialized:
            return
        
        logger.info("Executing safe thruster shutdown...")
        self.set_neutral()
        time.sleep(0.05) # Allow a moment for commands to process
        self.pca.output_disable()
        logger.info("Thruster shutdown complete.")
