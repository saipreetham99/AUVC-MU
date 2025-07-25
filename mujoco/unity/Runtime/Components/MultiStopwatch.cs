using System.Diagnostics;

public class MultiStopwatch
{
    private const int NUM_WATCHES = 3;

    private Stopwatch[] stopwatches = new Stopwatch[NUM_WATCHES];
    private double[] elapsedSeconds = new double[NUM_WATCHES];

    public MultiStopwatch()
    {
        for (int i = 0; i < NUM_WATCHES; i++)
        {
            stopwatches[i] = new Stopwatch();
        }
    }

    public long[] Track(int index)
    {
        if (index == -1) return null;

        if (index < 0 || index >= NUM_WATCHES) return null;

        var sw = stopwatches[index];

        if (!sw.IsRunning)
        {
            sw.Start();
        }
        else
        {
            sw.Stop();
            elapsedSeconds[index] += sw.Elapsed.TotalSeconds;
            sw.Reset(); // optional if you want a clean start
        }

        return null;
    }

    public double GetElapsedSeconds(int index)
    {
        if (index < 0 || index >= NUM_WATCHES) return 0;

        double runningTime = stopwatches[index].IsRunning
            ? stopwatches[index].Elapsed.TotalSeconds
            : 0;

        return elapsedSeconds[index] + runningTime;
    }
}

