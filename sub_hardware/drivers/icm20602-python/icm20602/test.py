#!/usr/bin/python3

def main():
    from icm20602 import ICM20602
    import time

    icm = ICM20602()

    def data_getter():
        data = icm.read_all()
        return (f'{data.a.x} {data.a.y} {data.a.z} {data.g.x} {data.g.y} {data.g.z} {data.t} '
                f'{data.a_raw.x} {data.a_raw.y} {data.a_raw.z} '
                f'{data.g_raw.x} {data.g_raw.y} {data.g_raw.z} {data.t_raw}')

    try:
        while True:
            print(data_getter())
            time.sleep(0.01)  # Adjust the loop frequency as needed
    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == '__main__':
    main()
