#!/usr/bin/python3

def main():
    import argparse
    from pca9685 import PCA9685
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', action='store', type=int, default=None)
    group1 = parser.add_mutually_exclusive_group()
    group1.add_argument('--get', action='store', nargs='+', type=int)
    group1.add_argument('--set', action='store', nargs='+', type=int)
    args = parser.parse_args()

    pca = PCA9685()

    if args.frequency:
        pca.set_pwm_frequency(args.frequency)

    pca.output_enable()

    if args.get:
        for channel in args.get:
            print(f'channel {channel} pwm: {pca.pwm[channel]}') 
    elif args.set:
        for channel in args.set[:-1]:
            try:
                pca.pwm[channel] = args.set[-1]
            except Exception as e:
                print(e)

if __name__ == '__main__':
    main()
