# File: depth.py
import json
import logging
import os
import threading
import time
from typing import Optional

try:
    import ms5837

    IS_MOCK = False
except ImportError:
    logging.critical("ms5837 library not found. Using mock depth sensor.")
    IS_MOCK = True

    # Mock ms5837 module for testing
    class MockMS5837:
        UNITS_mbar = 0
        UNITS_psi = 1
        UNITS_atm = 2
        UNITS_Torr = 3
        UNITS_Centigrade = 0
        UNITS_Farenheit = 1
        UNITS_Kelvin = 2
        DENSITY_FRESHWATER = 997
        DENSITY_SALTWATER = 1029

        def __init__(self, bus=6):
            self.bus = bus
            self._pressure_mbar = 1013.25  # Sea level pressure
            self._temperature_c = 20.0
            self._fluid_density = self.DENSITY_FRESHWATER
            self._base_depth = 0.1  # Simulate starting at 10cm depth

        def init(self):
            return True

        def read(self):
            # Simulate slight variations
            import random

            self._pressure_mbar += random.uniform(-0.1, 0.1)
            self._temperature_c += random.uniform(-0.05, 0.05)
            return True

        def pressure(self, units=None):
            if units is None:
                units = self.UNITS_mbar
            if units == self.UNITS_mbar:
                return self._pressure_mbar
            elif units == self.UNITS_psi:
                return self._pressure_mbar * 0.0145038
            elif units == self.UNITS_atm:
                return self._pressure_mbar / 1013.25
            elif units == self.UNITS_Torr:
                return self._pressure_mbar * 0.750062
            return self._pressure_mbar

        def temperature(self, units=None):
            if units is None:
                units = self.UNITS_Centigrade
            if units == self.UNITS_Centigrade:
                return self._temperature_c
            elif units == self.UNITS_Farenheit:
                return self._temperature_c * 9 / 5 + 32
            elif units == self.UNITS_Kelvin:
                return self._temperature_c + 273.15
            return self._temperature_c

        def depth(self):
            # Simulate depth based on pressure
            pressure_diff = self._pressure_mbar - 1013.25
            return self._base_depth + (pressure_diff / 100.0)  # Rough conversion

        def setFluidDensity(self, density):
            self._fluid_density = density

        def altitude(self):
            return 0.0

    # Create mock ms5837 module
    class MockMs5837Module:
        MS5837_30BA = MockMS5837
        UNITS_mbar = MockMS5837.UNITS_mbar
        UNITS_psi = MockMS5837.UNITS_psi
        UNITS_atm = MockMS5837.UNITS_atm
        UNITS_Torr = MockMS5837.UNITS_Torr
        UNITS_Centigrade = MockMS5837.UNITS_Centigrade
        UNITS_Farenheit = MockMS5837.UNITS_Farenheit
        UNITS_Kelvin = MockMS5837.UNITS_Kelvin
        DENSITY_FRESHWATER = MockMS5837.DENSITY_FRESHWATER
        DENSITY_SALTWATER = MockMS5837.DENSITY_SALTWATER

    ms5837 = MockMs5837Module()


class DepthSensor:
    CACHE_FILENAME = "orientation_cache.json"

    def __init__(
        self, bus=6, fluid_density=None, surface_depth_cm=10.0, pressure_units="mbar"
    ):
        """
        Initialize depth sensor.

        Args:
            bus: I2C bus number for the sensor
            fluid_density: Fluid density (None for freshwater, or specify custom)
            surface_depth_cm: Initial depth to consider as surface (default 10cm)
            pressure_units: Units for pressure readings ('mbar', 'psi', 'atm', 'torr')
        """
        self.initialized = False
        self.sensor = None
        self.running = False
        self.bus = bus
        self.surface_depth_cm = surface_depth_cm
        self.pressure_units = pressure_units

        # Reference values for zero depth
        self.reference_depth = None

        # Latest readings
        self.latest_data = None
        self.data_lock = threading.Lock()
        self.reference_lock = threading.Lock()

        # Thread for continuous reading
        self._thread = None

        try:
            logging.info(f"Initializing depth sensor (MS5837) on bus {bus}...")

            self.sensor = ms5837.MS5837_30BA(bus=bus)

            if not self.sensor.init():
                raise RuntimeError("Sensor could not be initialized")

            # Set fluid density
            if fluid_density is None:
                self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
                logging.info("Using freshwater density for depth calculations")
            else:
                self.sensor.setFluidDensity(fluid_density)
                logging.info(f"Using custom fluid density: {fluid_density}")

            # Initial reading to establish baseline
            if not self.sensor.read():
                raise RuntimeError("Initial sensor read failed")

            # Load reference from cache or set initial reference
            if not self._load_reference_from_cache():
                self._set_initial_reference()

            if IS_MOCK:
                logging.info("Depth sensor initialized successfully (MOCK MODE)")
            else:
                logging.info(f"Depth sensor initialized successfully on bus {bus}")
            self.initialized = True

        except Exception as e:
            logging.error(f"Failed to initialize depth sensor: {e}")
            logging.warning("Depth sensing will be disabled.")

    def _set_initial_reference(self):
        """Set the initial reference depth based on surface_depth_cm"""
        try:
            current_depth_m = self.sensor.depth()

            with self.reference_lock:
                # Set current depth as our reference point (this becomes "zero")
                self.reference_depth = current_depth_m

            logging.info(
                f"Surface reference set at {current_depth_m:.3f}m absolute depth (assuming {self.surface_depth_cm}cm from true surface)"
            )

            # Save to cache
            self._save_reference_to_cache()

        except Exception as e:
            logging.error(f"Failed to set initial depth reference: {e}")

    def start(self):
        """Start the depth sensor reading thread"""
        if not self.initialized:
            logging.error("Cannot start depth sensor, initialization failed.")
            return

        if self.running:
            logging.warning("Depth sensor is already running.")
            return

        self.running = True
        self._thread = threading.Thread(target=self._reading_loop, daemon=True)
        self._thread.start()
        logging.info("Depth sensor started")

    def stop(self):
        """Stop the depth sensor reading thread"""
        logging.info("Stopping depth sensor...")
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logging.info("Depth sensor stopped.")

    def reset_depth(self):
        """Reset current depth to be the new zero reference (surface)"""
        if not self.initialized:
            return

        try:
            if not self.sensor.read():
                logging.error("Failed to read sensor for depth reset")
                return

            current_depth_m = self.sensor.depth()

            with self.reference_lock:
                self.reference_depth = current_depth_m

            logging.info(
                f"Depth reference reset. New surface at {current_depth_m:.3f}m absolute depth"
            )
            self._save_reference_to_cache()

        except Exception as e:
            logging.error(f"Failed to reset depth reference: {e}")

    def get_latest_data(self) -> Optional[dict]:
        """Get the latest depth sensor data"""
        with self.data_lock:
            return self.latest_data

    def _reading_loop(self):
        """Main loop for reading sensor data"""
        while self.running:
            try:
                if not self.sensor.read():
                    logging.warning("Depth sensor read failed")
                    time.sleep(1.0)
                    continue

                # Get raw readings
                pressure = self.sensor.pressure(self._get_pressure_units())
                temperature = self.sensor.temperature()
                absolute_depth_m = self.sensor.depth()

                # Calculate relative depth (negative going deeper)
                relative_depth_m = self._calculate_relative_depth(absolute_depth_m)
                relative_depth_cm = relative_depth_m * 100.0

                # Create data structure
                depth_data = {
                    "timestamp": time.time(),
                    "pressure": {"value": pressure, "units": self.pressure_units},
                    "temperature": {"value": temperature, "units": "celsius"},
                    "depth": {
                        "relative_m": relative_depth_m,
                        "relative_cm": relative_depth_cm,
                        "absolute_m": absolute_depth_m,
                    },
                }

                with self.data_lock:
                    self.latest_data = depth_data

                time.sleep(0.02)  # 50Hz reading rate

            except Exception as e:
                logging.error(f"Error in depth sensor reading loop: {e}")
                time.sleep(1.0)

    def _calculate_relative_depth(self, current_depth_m):
        """Calculate depth relative to the reference surface"""
        try:
            with self.reference_lock:
                if self.reference_depth is None:
                    return 0.0

                # Calculate relative depth (negative going deeper from surface)
                # If current > reference, we're deeper (negative)
                # If current < reference, we're shallower (positive)
                relative_depth = -(current_depth_m - self.reference_depth)

                return relative_depth

        except Exception as e:
            logging.error(f"Error calculating relative depth: {e}")
            return 0.0

    def _get_pressure_units(self):
        """Get the pressure units constant for the sensor"""
        units_map = {
            "mbar": ms5837.UNITS_mbar,
            "psi": ms5837.UNITS_psi,
            "atm": ms5837.UNITS_atm,
            "torr": ms5837.UNITS_Torr,
        }
        return units_map.get(self.pressure_units.lower(), ms5837.UNITS_mbar)

    def _save_reference_to_cache(self):
        """Save reference values to cache file"""
        try:
            data = {}
            if os.path.exists(self.CACHE_FILENAME):
                with open(self.CACHE_FILENAME, "r") as f:
                    data = json.load(f)

            data["depth_reference"] = {
                "depth": self.reference_depth,
                "surface_depth_cm": self.surface_depth_cm,
            }

            with open(self.CACHE_FILENAME, "w") as f:
                json.dump(data, f, indent=4)

        except Exception as e:
            logging.error(f"Could not save depth reference to cache: {e}")

    def _load_reference_from_cache(self):
        """Load reference values from cache file"""
        try:
            if os.path.exists(self.CACHE_FILENAME):
                with open(self.CACHE_FILENAME, "r") as f:
                    data = json.load(f)

                if "depth_reference" in data:
                    ref_data = data["depth_reference"]
                    with self.reference_lock:
                        self.reference_depth = ref_data.get("depth")

                    if self.reference_depth is not None:
                        logging.info(
                            f"Loaded depth reference from cache: {self.reference_depth:.3f}m"
                        )
                        return True
        except Exception as e:
            logging.warning(f"Could not load depth reference from cache: {e}")
        return False
