package first.minolib.hardware;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import first.minolib.advantagekit.LoggedTunableNumber;
import first.minolib.hardware.io.CANBusInputsAutoLogged;

import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Alert;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;

/**
 * The MinoCANBus class represents a CAN bus interface for the robot. It provides functionality to
 * log and update the status of the CAN bus.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * MinoCANBus canBus = new MinoCANBus();
 * canBus.updateInputs();
 * }</pre>
 *
 * <p>Constructor Summary:
 *
 * <ul>
 *   <li>{@link #MinoCANBus()} - Initializes the CAN bus with the default name "rio".
 *   <li>{@link #MinoCANBus(String canbusName)} - Initializes the CAN bus with the specified name.
 * </ul>
 *
 * <p>Method Summary:
 *
 * <ul>
 *   <li>{@link #updateInputs()} - Updates the CAN bus inputs and logs the current status.
 * </ul>
 *
 * <p>Inner Class:
 *
 * <ul>
 *   <li>{@link MinoCANBus.CANBusInputs} - Represents the inputs for a CAN bus.
 * </ul>
 */

public class MinoCANBus {
    private final String canbusName;
    private final String loggingName;
    private final CANBus canBus;

    private final CANBusInputsAutoLogged inputs = new CANBusInputsAutoLogged();

    private final LoggedTunableNumber kCANTimeoutThreshold;
    private final Debouncer canbusConnectedDebouncer;
    private final Alert canbusErrorAlert;

    public enum CANBusLane {
        S0(0, "can_s0"),
        S1(1, "can_s1"),
        S2(2, "can_s2"),
        S3(3, "can_s3"),
        S4(4, "can_s4");

        private final int index;
        private final String name;
        private final Alert refreshAlert;

        CANBusLane(int index, final String name) {
            this.index = index;
            this.name = name;

            refreshAlert = new Alert("Failed to refresh signals on SystemCore CAN lane " + name, Level.HIGH);
        }

        public int getBusNumber() {
            return index;
        }

        public String getBusName() {
            return name;
        }

        public Alert getAlert() {
            return refreshAlert;
        }
    }

    public MinoCANBus(final CANBusLane canbusLane) {
        this.canbusName = canbusLane.getBusName();
        loggingName = "Inputs/CANBus [" + canbusLane.getBusNumber() + "]";
        canBus = new CANBus(this.canbusName, "./logs/example.hoot");

        kCANTimeoutThreshold = new LoggedTunableNumber("CoreControl/CANBus [" + canbusLane.getBusNumber() + "] Timeout", 0.5);
        canbusConnectedDebouncer = new Debouncer(kCANTimeoutThreshold.get(), DebounceType.kRising);
        canbusErrorAlert = new Alert("CAN error detected on " + canbusLane.getBusName() + ", robot may not be controllable.", Level.HIGH);
    }

    public void updateInputs() {
        CANBusStatus status = canBus.getStatus();

        inputs.status = status.Status;
        inputs.busUtilization = status.BusUtilization;
        inputs.busOffCount = status.BusOffCount;
        inputs.txFullCount = status.TxFullCount;
        inputs.REC = status.REC;
        inputs.TEC = status.TEC;
        inputs.isNetworkFD = canBus.isNetworkFD();

        Logger.processInputs(loggingName, inputs);

        if (kCANTimeoutThreshold.hasChanged(hashCode())) {
            canbusConnectedDebouncer.setDebounceTime(kCANTimeoutThreshold.get());
        }

        boolean debouncedError = canbusConnectedDebouncer.calculate(!status.Status.isOK() || status.REC > 0 || status.TEC > 0);
        canbusErrorAlert.set(debouncedError);
    }

    public CANBus getParent() {
        return canBus;
    }

    public String getName() {
        return canBus.getName();
    }
}