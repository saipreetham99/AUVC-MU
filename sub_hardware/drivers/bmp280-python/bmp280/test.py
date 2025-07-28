#!/usr/bin/python3

def main():
    from bmp280 import BMP280
    import argparse
    import time

    parser = argparse.ArgumentParser(description="BMP280 Sensor Reader")
    parser.add_argument("--bus", default=1, type=int, help="i2c bus")
    parser.add_argument("--frequency", type=float, default=10.0, help="Read frequency in Hz")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration to run in seconds")
    args = parser.parse_args()

    bmp = BMP280(args.bus)
    compensation = bmp.get_compensation()
    print("ROM:", ' '.join(str(d) for d in compensation.data))
    print("CONFIG:", f'{bmp.osrs_t} {bmp.osrs_p} {bmp.mode} {bmp.t_sb} {bmp.filter}')

    def data_getter():
        data = bmp.get_data()
        return f'{data.pressure:.6f} {data.temperature:.6f} {data.pressure_raw} {data.temperature_raw}'

    start_time = time.time()
    try:
        while time.time() - start_time < args.duration:
            print(data_getter())
            time.sleep(1.0 / args.frequency)
    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == '__main__':
    main()
