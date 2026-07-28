package frc.minolib.hardware;

import com.ctre.phoenix6.CANBus;

import frc.robot.constants.GlobalConstants;

/**
 * Represents a CAN device identifier with bus name and device number. Used to uniquely identify and
 * configure CAN-based devices on the robot.
 */

public class MinoCANDevice {
    /** Default CAN bus name for the RoboRIO */
  
    /** The device number on the CAN bus */
    private int deviceNumber;
  
    /** The name of the CAN bus this device is on */
    private final MinoCANBus CANBus;
  
    /**
     * Creates a new CAN device ID with specified device number and bus name.
     *
     * @param deviceNumber The device's CAN ID number
     * @param CANbusName The name of the CAN bus
     */
    public MinoCANDevice(final int deviceNumber, final MinoCANBus CANbusName) {
        this.deviceNumber = deviceNumber;
        this.CANBus = CANbusName;
    }
  
    /**
     * Creates a new CAN device ID on the default RIO CAN bus.
     *
     * @param deviceNumber The device's CAN ID number
     */
    public MinoCANDevice(final int deviceNumber) {
        this(deviceNumber, GlobalConstants.kRioBus);
    }

    public void newDeviceID(final int deviceNumber) {
        this.deviceNumber = deviceNumber;
    }

    public int getDeviceID() {
        return deviceNumber;
    }

    public CANBus getCANBus() {
        return CANBus.getParent();
    }
  
    /**
     * @return String representation of the CAN device ID in format [busName deviceNumber]
     */
    @Override
    public String toString() {
        return "[" + CANBus + " " + deviceNumber + "]";
    }
  }