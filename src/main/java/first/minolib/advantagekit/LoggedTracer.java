package first.minolib.advantagekit;

import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;

public class LoggedTracer {
    private LoggedTracer() {}
    private static double startTime = -1.0;

    /** Reset the clock. */
    public static void reset() {
        startTime = Timer.getMonotonicTimestamp();
    }

    /** Save the time elapsed since the last reset or record. */
    public static void record(String epochName) {
        double now = Timer.getMonotonicTimestamp();
        Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
        startTime = now;
    }
}