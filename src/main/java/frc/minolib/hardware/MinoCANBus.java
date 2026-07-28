package frc.minolib.hardware;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.minolib.io.CANBusInputsAutoLogged;
import frc.robot.constants.GlobalConstants;

import org.littletonrobotics.junction.Logger;

/**
 * The MinoCANBus class represents a CAN bus interface for the robot. It provides functionality to
 * log and update the status of the CAN bus.
 *
 * <p>It includes an inner static class CANBusInputs which holds the status and various metrics of
 * the CAN bus.
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

  private final Debouncer canivoreConnectedDebouncer = new Debouncer(GlobalConstants.kCANivoreTimeThreshold, DebounceType.kRising);
  private final Alert canivoreErrorAlert = new Alert("CANivore error detected, robot may not be controllable.", AlertType.kError);

  public MinoCANBus() {
    this("rio");
  }

  public MinoCANBus(final String canbusName) {
    this.canbusName = canbusName;
    loggingName = "Inputs/CANBus [" + canbusName + "]";
    canBus = new CANBus(this.canbusName, "./logs/example.hoot");
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

    boolean debouncedError = canivoreConnectedDebouncer.calculate(!status.Status.isOK() || status.REC > 0 || status.TEC > 0);
    canivoreErrorAlert.set(debouncedError);
  }

  public CANBus getParent() {
    return canBus;
  }

  public String getName() {
    return canBus.getName();
  }
}