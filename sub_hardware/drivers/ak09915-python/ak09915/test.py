#!/usr/bin/python3

def main():
    from ak09915 import AK09915
    import argparse
    import time

    parser = argparse.ArgumentParser(description="AK09915 Magnetometer Reader")
    parser.add_argument("--bus", type=int, default=1, help="i2c bus")
    parser.add_argument("--frequency", type=float, default=10.0, help="Read frequency in Hz")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration to run in seconds")
    args = parser.parse_args()

    ak = AK09915(args.bus)

    def data_getter():
        data = ak.measure()
        return f"{data.x_raw} {data.y_raw} {data.z_raw} {data.x:.6f} {data.y:.6f} {data.z:.6f}"

    start_time = time.time()
    try:
        while time.time() - start_time < args.duration:
            print(data_getter())
            time.sleep(1.0 / args.frequency)
    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == '__main__':
    main()
