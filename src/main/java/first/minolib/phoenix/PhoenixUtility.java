package first.minolib.phoenix;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.function.Supplier;

import org.wpilib.driverstation.Alert;
import org.wpilib.driverstation.Alert.Level;

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

    public static void registerSignals(CANBusLane lane, BaseStatusSignal... signals) {
        BaseStatusSignal[] currentSignals = signalMap.get(lane);
        BaseStatusSignal[] newSignals = Arrays.copyOf(currentSignals,currentSignals.length + signals.length);

        System.arraycopy(signals, 0, newSignals, currentSignals.length, signals.length);

        signalMap.put(lane, newSignals);
    }

    public static void refreshAll() {
        for (CANBusLane lane : CANBusLane.values()) {
            BaseStatusSignal[] signals = signalMap.get(lane);

            if (signals.length == 0) {
                continue;
            }

            checkError(BaseStatusSignal.refreshAll(signals), "Failed to refresh signals on " + lane, new Alert("", Level.HIGH));
        }
    }
}
