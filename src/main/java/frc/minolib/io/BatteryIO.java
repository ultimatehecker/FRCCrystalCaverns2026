package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

public interface BatteryIO {
    @AutoLog
    public static class BatteryIOInputs {
        public double batteryVoltage = 12.0;
        public double rioCurrent = 0.0;
    }
}
