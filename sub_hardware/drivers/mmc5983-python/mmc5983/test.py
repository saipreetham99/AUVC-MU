#!/usr/bin/python3

def main():
    from mmc5983 import MMC5983
    import argparse
    import time

    parser = argparse.ArgumentParser(description="MMC5983 Data Logger")
    parser.add_argument("--bus", type=int, default=None, help="i2c bus")
    parser.add_argument("--cal", type=int, default=None, help="i2c calibration value (unused)")
    parser.add_argument("--frequency", type=float, default=10.0, help="Data read frequency in Hz")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration to run in seconds")
    parser.add_argument("--stop_on_error", action="store_true", help="Stop on error")
    args = parser.parse_args()

    mmc = MMC5983(i2cbus=args.bus)
    lastcal = time.time()

    frequency = args.frequency
    duration = args.duration
    stop_on_error = args.stop_on_error

    start_time = time.time()
    try:
        while time.time() < start_time + duration:
            try:
                if time.time() > lastcal + 1:
                    mmc.calibrate()
                    lastcal = time.time()
                data = mmc.read_data()
                output = (
                    f"{data.x_raw} {data.y_raw} {data.z_raw} {data.t_raw} "
                    f"{data.x:.6f} {data.y:.6f} {data.z:.6f} {data.t:.3f} "
                    f"{mmc.caldata[0]} {mmc.caldata[1]} {mmc.caldata[2]}"
                )
                print(output)
            except Exception as e:
                print(f"ERROR: {e}")
                if stop_on_error:
                    break

            if frequency:
                time.sleep(1.0 / frequency)
    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == '__main__':
    main()
