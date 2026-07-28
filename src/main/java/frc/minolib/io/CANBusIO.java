package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface CANBusIO {
    
    @AutoLog
    public static class CANBusInputs {
        public StatusCode status = StatusCode.OK;
        public double busUtilization = 0.0;
        public int busOffCount = 0;
        public int txFullCount = 0;
        public int REC = 0;
        public int TEC = 0;
        public boolean isNetworkFD = false;
    }
}