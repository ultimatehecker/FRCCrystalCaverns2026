package first.minolib.phoenix;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.function.Supplier;

import org.wpilib.driverstation.Alert;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import first.minolib.hardware.MinoCANBus.CANBusLane;

public class PhoenixUtility {
    private static final EnumMap<CANBusLane, BaseStatusSignal[]> signalMap = new EnumMap<>(CANBusLane.class);

    static {
        for (CANBusLane lane : CANBusLane.values()) {
            signalMap.put(lane, new BaseStatusSignal[0]);
        }
    }

    /**
     * Invokes the specified CTRE function until it is successful or the number of tries is exceeded.
     * Sets the specified alert if the function fails.
     *
     * @param function CTRE function to invoke
     * @param alert alert to set if the function fails
     * @param numTries number of times to try the function
     * @return true if the function was successful, false otherwise
     */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    /**
     * Checks the specified status code and sets the specified alert to the specified message if the
     * status code is not OK.
     *
     * @param statusCode status code to check
     * @param message message to set in the alert if the status code is not OK
     * @param alert alert to set if the status code is not OK
     */
    public static void checkError(StatusCode statusCode, String message, Alert alert) {
        if (statusCode != StatusCode.OK) {
            alert.setText(message + " " + statusCode);
            alert.set(true);
        } else {
            alert.set(false);
        }
    }

    /** Registers a set of signals to a specific lane for a synchronized refresh.
     * 
     * @param lane Specific lane to register the signals (must correspond to the specific lane the device is attached to)
     * @param signals List of signals to register
     */
    public static void registerSignals(CANBusLane lane, BaseStatusSignal... signals) {
        BaseStatusSignal[] currentSignals = signalMap.get(lane);
        BaseStatusSignal[] newSignals = Arrays.copyOf(currentSignals,currentSignals.length + signals.length);

        System.arraycopy(signals, 0, newSignals, currentSignals.length, signals.length);

        signalMap.put(lane, newSignals);
    }

    /** Refresh all registered signals on each of the respective lanes on the SystemCore */
    public static void refreshAll() {
        for (CANBusLane lane : CANBusLane.values()) {
            BaseStatusSignal[] signals = signalMap.get(lane);

            if (signals.length == 0) {
                continue;
            }

            checkError(BaseStatusSignal.refreshAll(signals), "Failed to refresh signals on " + lane, lane.getAlert());
        }
    }
}
