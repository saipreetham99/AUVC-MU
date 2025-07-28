#!/usr/bin/python3

import matplotlib.pyplot as plt

DEVICE = 'bmp280'

def generate_figures(log):
    footer = f'{DEVICE} test report'

    f, spec = log.figure(height_ratios=[1,1], suptitle=f'{DEVICE} data', footer=footer)
    plt.subplot(spec[0,0])
    log.rom.T.ttable(rl=True)
    plt.subplot(spec[0,1])
    log.config.T.ttable(rl=True)

    plt.subplot(spec[1,:])
    log.data.pressure.pplot(log.data.temperature)

def main():
    from llog import LLogReader
    from matplotlib.backends.backend_pdf import PdfPages
    
    parser = LLogReader.create_default_parser(__file__, DEVICE)
    args = parser.parse_args()

    log = LLogReader(args.input, args.meta)

    generate_figures(log)

    if args.output:
        # todo check if it exists!
        with PdfPages(args.output) as pdf:
            [pdf.savefig(n) for n in plt.get_fignums()]

    if args.show:
        print('hello?')
        plt.show()

if __name__ == '__main__':
    main()
