#!/usr/bin/python3

import matplotlib.pyplot as plt

def generate_figures(log):
    magRaw = log.data[['xRaw', 'yRaw', 'zRaw']]
    mag = log.data[['xScaled', 'yScaled', 'zScaled']]

    footer = 'mmc5983 test report'

    f, spec = log.figure(height_ratios=[1, 1], suptitle='mmc5983 data', footer=footer)
    plt.subplot(spec[0,:])
    mag.stats().ttable(rl=True)

    plt.subplot(spec[1,:])
    mag.pplot(title='mmc5983 data')

    if getattr(log, 'error', False) is not False:
        f, spec = log.figure(height_ratios=[1], suptitle='mmc5983 errors', footer=footer)

        plt.subplot(spec[0,:])
        log.error.head(20).ttable(rl=True)

def main():
    from llog import LLogReader
    from matplotlib.backends.backend_pdf import PdfPages
    from pathlib import Path

    parser = LLogReader.create_default_parser(__file__, 'mmc5983')
    args = parser.parse_args()

    log = LLogReader(args.input, args.meta)

    generate_figures(log)

    if args.output:
        if Path(args.output).exists():
            print(f'WARN {args.output} exists! skipping ..')
        else:
            with PdfPages(args.output) as pdf:
                [pdf.savefig(n) for n in plt.get_fignums()]

    if args.show:
        plt.show()

if __name__ == '__main__':
    main()
