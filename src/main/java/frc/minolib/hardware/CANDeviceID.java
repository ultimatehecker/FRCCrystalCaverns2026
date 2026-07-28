package frc.minolib.hardware;

/**
 * Represents a CAN device identifier with bus name and device number. Used to uniquely identify and
 * configure CAN-based devices on the robot.
 */

public class CANDeviceID {
    public static final String kRIOCANbusName = "rio";
    public int deviceNumber;
    public final String CANbusName;

    public CANDeviceID(final int deviceNumber, final String CANbusName) {
        this.deviceNumber = deviceNumber;
        this.CANbusName = CANbusName;
    }
  
    public CANDeviceID(final int deviceNumber) {
        this(deviceNumber, kRIOCANbusName);
    }

    public void newDeviceID(final int deviceNumber) {
        this.deviceNumber = deviceNumber;
    }

    @Override
    public String toString() {
        return "[" + CANbusName + " " + deviceNumber + "]";
    }
  }