#!/usr/bin/python
import time

import ms5837

# Specify the I2C bus your sensor is on
sensor = ms5837.MS5837_30BA(bus=6)

# We must initialize the sensor before reading it
if not sensor.init():
    print("Sensor could not be initialized")
    exit(1)
else:
    print("Sensor initialized successfully!")

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

print(
    ("Pressure: %.2f atm  %.2f Torr  %.2f psi")
    % (
        sensor.pressure(ms5837.UNITS_atm),
        sensor.pressure(ms5837.UNITS_Torr),
        sensor.pressure(ms5837.UNITS_psi),
    )
)

print(
    ("Temperature: %.2f C  %.2f F  %.2f K")
    % (
        sensor.temperature(ms5837.UNITS_Centigrade),
        sensor.temperature(ms5837.UNITS_Farenheit),
        sensor.temperature(ms5837.UNITS_Kelvin),
    )
)

freshwaterDepth = sensor.depth()  # default is freshwater
sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
saltwaterDepth = sensor.depth()  # No need to read() again
# Set back to freshwater for the print statement to be accurate
sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
print(
    ("Depth: %.3f m (freshwater)  %.3f m (saltwater)")
    % (freshwaterDepth, saltwaterDepth)
)

# fluidDensity doesn't matter for altitude() (always MSL air density)
print(
    ("MSL Relative Altitude: %.2f m") % sensor.altitude()
)  # relative to Mean Sea Level pressure in air

time.sleep(5)

# Spew readings
while True:
    try:
        if sensor.read():
            print(
                ("P: %0.1f mbar  %0.3f psi\tT: %0.2f C  %0.2f F")
                % (
                    sensor.pressure(),  # Default is mbar (no arguments)
                    sensor.pressure(ms5837.UNITS_psi),  # Request psi
                    sensor.temperature(),  # Default is degrees C (no arguments)
                    sensor.temperature(ms5837.UNITS_Farenheit),
                )
            )  # Request Farenheit
        else:
            print("Sensor read failed!")
            exit(1)
        time.sleep(0.5)  # A short delay to prevent flooding the screen
    except KeyboardInterrupt:
        print("\nExiting program.")
        break
    except Exception as e:
        print(f"An error occurred: {e}")
        break
