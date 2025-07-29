# File: thrusters.py
import time
import logging

# Get a logger for this module
logger = logging.getLogger(__name__)

try:
    from pca9685 import PCA9685
    IS_MOCK = False
except ImportError:
    logger.critical("Failed to import pca9685 library. Using mock interface.")
    IS_MOCK = True
    # Define a mock class that accurately reflects the real library's API
    class PCA9685:
        def __init__(self, i2c, address=0x40): 
            self._i2c = i2c
            self._address = address
            self.pwm = [0]*16
            logger.info("Mock PCA: Initialized.")
        def set_pwm_frequency(self, freq): 
            logger.info(f"Mock PCA: Freq set to {freq}Hz")
        def output_enable(self): 
            logger.info(f"Mock PCA: Output enabled.")
        def output_disable(self): 
            logger.info(f"Mock PCA: Output disabled.")


class ThrusterController:
    """
    Manages the PCA9685 board to control 6 thrusters and 1 light.
    Handles safe startup and shutdown procedures and hardware failures.
    """
    NEUTRAL_PULSE = 1500
    LIGHT_OFF_PULSE = 1100
    THRUSTER_CHANNELS = range(6)
    LIGHT_CHANNEL = 9

    def __init__(self):
        """
        Initializes the Thruster Controller. Sets an 'initialized' flag to indicate
        success or failure, as __init__ cannot return a value.
        """
        self.initialized = False
        self.pca = None
        
        try:
            logger.info("Initializing Thruster Controller (PCA9685)...")
            
            if IS_MOCK:
                # The mock will always "succeed" without needing other libraries
                self.pca = PCA9685(i2c=None)
            else:
                # For real hardware, we need board and busio. Import them here.
                # If these imports fail, the except block will catch it.
                import board
                import busio
                
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685()

            self.pca.set_pwm_frequency(50)
            self.set_neutral()
            self.pca.output_enable()
            
            logger.info("Thruster ESCs initialized. Audible sound should occur now.")
            self.initialized = True

        except Exception as e:
            # This will now correctly catch any error: missing pca9685, board, busio, or I2C hardware error.
            logger.error(f"Failed to initialize Thruster Controller (PCA9685): {e}")
            logger.warning("Thruster control will be disabled.")
            # self.initialized remains False

    def set_neutral(self):
        """Sets all thrusters to neutral and turns off the light."""
        if not self.initialized:
            return
        for ch in self.THRUSTER_CHANNELS:
            self.pca.pwm[ch] = self.NEUTRAL_PULSE
        self.pca.pwm[self.LIGHT_CHANNEL] = self.LIGHT_OFF_PULSE
        time.sleep(0.05)

    def apply_control(self, thruster_pulses, light_pulse, duration):
        """Applies specified pulse values to the PCA9685 board's channels."""
        if not self.initialized:
            return
            
        if len(thruster_pulses) != 6:
            logger.error(f"Invalid thruster pulse count: {len(thruster_pulses)}. Expected 6.")
            return

        for i, pulse in enumerate(thruster_pulses):
            self.pca.pwm[self.THRUSTER_CHANNELS[i]] = pulse
        
        self.pca.pwm[self.LIGHT_CHANNEL] = light_pulse

        logger.info(
            f"ApplyingForces:{thruster_pulses}, Light: {light_pulse}"
        )
        time.sleep(duration)
        
    def safe_shutdown(self):
        """Sets all outputs to a safe state and disables the PCA9685 driver."""
        if not self.initialized:
            return
        logger.info("Executing safe thruster shutdown...")
        self.set_neutral()
        self.pca.output_disable()
        logger.info("Thruster shutdown complete.")
