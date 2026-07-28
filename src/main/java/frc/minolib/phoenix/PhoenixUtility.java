package frc.minolib.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;

import frc.minolib.math.EqualsUtility;

import java.util.function.Supplier;

public class PhoenixUtility {
  private static final double kEps = 0.1;

  private static Alert canivoreSignalsAlert = new Alert("", Alert.AlertType.kError);
  private static Alert rioSignalsAlert = new Alert("", Alert.AlertType.kError);

  public static void simpleTryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  public static boolean retryUntilSuccess(final Supplier<StatusCode> setterFunction, final String description) {
    return retryUntilSuccess(setterFunction, () -> true, 10, description);
  }

  public static boolean retryUntilSuccess(final Supplier<StatusCode> setterFunction, final Supplier<Boolean> validatorFunction, final String description) {
    return retryUntilSuccess(setterFunction, validatorFunction, 10, description);
  }

  public static boolean retryUntilSuccess(final Supplier<StatusCode> setterFunction, final Supplier<Boolean> validatorFunction, final int numTries, final String description) {
    for (int i = 0; i < numTries; i++) {
      // Call Phoenix setter and check for success.
      final StatusCode code = setterFunction.get();
      if (!code.isOK()) {
        DriverStation.reportWarning(description + ": Retrying (" + (i + 1) + "/" + numTries + ") due to config error code: " + code.getName(), false);
        continue;
      }

      // Status code indicates success, but check to make sure it's actually correct.
      if (!validatorFunction.get()) {
        DriverStation.reportWarning(description + ": Retrying (" + (i + 1) + "/" + numTries + ") due to failed config validation", false);
        continue;
      }

      // We are actually successful!
      return true;
    }

    // We exhausted numTries without success.
    DriverStation.reportError(description, false);
    return false;
  }

  public static boolean CANRangeConfigsEqual(final CANrangeConfiguration expected, final CANrangeConfiguration actual) {
    if (!isEqual(expected.ProximityParams, actual.ProximityParams)) {
      DriverStation.reportWarning("[ProximityParams] Expected: " + expected.ProximityParams + ", Actual: " + actual.ProximityParams, false);
      return false;
    }

    if (!isEqual(expected.ToFParams, actual.ToFParams)) {
      DriverStation.reportWarning("[ToFParams] Expected: " + expected.ToFParams + ", Actual: " + actual.ToFParams, false);
      return false;
    }

    if (!isEqual(expected.FovParams, actual.FovParams)) {
      DriverStation.reportWarning("[FovParams] Expected: " + expected.FovParams + ", Actual: " + actual.FovParams, false);
      return false;
    }

    return true;
  }

  public static boolean isEqual(ProximityParamsConfigs a, ProximityParamsConfigs b) {
    return EqualsUtility.epsilonEquals(a.MinSignalStrengthForValidMeasurement, b.MinSignalStrengthForValidMeasurement, kEps) 
      && EqualsUtility.epsilonEquals(a.ProximityHysteresis, b.ProximityHysteresis, kEps) 
      && EqualsUtility.epsilonEquals(a.ProximityThreshold, b.ProximityThreshold, kEps);
  }

  public static boolean isEqual(ToFParamsConfigs a, ToFParamsConfigs b) {
    return EqualsUtility.epsilonEquals(a.UpdateFrequency, b.UpdateFrequency, kEps) 
      && EqualsUtility.epsilonEquals(a.UpdateMode.value, b.UpdateMode.value, kEps);
  }

  public static boolean isEqual(FovParamsConfigs a, FovParamsConfigs b) {
    return EqualsUtility.epsilonEquals(a.FOVCenterX, b.FOVCenterX, kEps) 
      && EqualsUtility.epsilonEquals(a.FOVCenterY, b.FOVCenterY, kEps) 
      && EqualsUtility.epsilonEquals(a.FOVRangeX, b.FOVRangeX, kEps)
      && EqualsUtility.epsilonEquals(a.FOVRangeY, b.FOVRangeY, kEps);
  }

  public static boolean CANcoderConfigsEqual(final CANcoderConfiguration expected, final CANcoderConfiguration actual) {
    if (!expected.MagnetSensor.SensorDirection.equals(actual.MagnetSensor.SensorDirection)) {
      DriverStation.reportWarning("[SensorDirection] Expected: " + expected.MagnetSensor.SensorDirection + ", Actual: " + actual.MagnetSensor.SensorDirection, false);
      return false;
    }

    if (!EqualsUtility.epsilonEquals(expected.MagnetSensor.MagnetOffset, actual.MagnetSensor.MagnetOffset, kEps)) {
      DriverStation.reportWarning("[MagnetOffset] Expected: " + expected.MagnetSensor.MagnetOffset + ", Actual: " + actual.MagnetSensor.MagnetOffset, false);
      return false;
    }

    if (!EqualsUtility.epsilonEquals(expected.MagnetSensor.AbsoluteSensorDiscontinuityPoint, actual.MagnetSensor.AbsoluteSensorDiscontinuityPoint, kEps)) {
      DriverStation.reportWarning("[AbsoluteSensorDiscontinuityPoint] Expected: " + expected.MagnetSensor.AbsoluteSensorDiscontinuityPoint + ", Actual: " + actual.MagnetSensor.AbsoluteSensorDiscontinuityPoint, false);
      return false;
    }

    return true;
  }

  public static boolean Pigeon2ConfigsEqual(final Pigeon2Configuration expected, final Pigeon2Configuration actual) {
    if (!isEqual(expected.MountPose, actual.MountPose)) {
      DriverStation.reportWarning("[MountPose] Expected: " + expected.MountPose + ", Actual: " + actual.MountPose, false);
      return false;
    }

    if (!isEqual(expected.GyroTrim, actual.GyroTrim)) {
      DriverStation.reportWarning("[GyroTrim] Expected: " + expected.GyroTrim + ", Actual: " + actual.GyroTrim, false);
      return false;
    }

    return true;
  }

  public static boolean isEqual(MountPoseConfigs a, MountPoseConfigs b) {
    return EqualsUtility.epsilonEquals(a.MountPoseRoll, b.MountPoseRoll, kEps) 
      && EqualsUtility.epsilonEquals(a.MountPosePitch, b.MountPosePitch, kEps) 
      && EqualsUtility.epsilonEquals(a.MountPoseYaw, b.MountPoseYaw, kEps);
  }

  public static boolean isEqual(GyroTrimConfigs a, GyroTrimConfigs b) {
    return EqualsUtility.epsilonEquals(a.GyroScalarX, b.GyroScalarX, kEps) 
      && EqualsUtility.epsilonEquals(a.GyroScalarY, b.GyroScalarY, kEps) 
      && EqualsUtility.epsilonEquals(a.GyroScalarZ, b.GyroScalarZ, kEps);
  }

  public static boolean TalonFXConfigsEqual(TalonFXConfiguration expected, TalonFXConfiguration actual) {
    if (!isEqual(expected.MotorOutput, actual.MotorOutput)) {
      DriverStation.reportWarning("[MotorOutput] Expected: " + expected.MotorOutput + ", Actual: " + actual.MotorOutput, false);
      return false;
    }

    if (!isEqual(expected.CurrentLimits, actual.CurrentLimits)) {
      DriverStation.reportWarning("[CurrentLimits] Expected: " + expected.CurrentLimits + ", Actual: " + actual.CurrentLimits, false);
      return false;
    }

    if (!isEqual(expected.TorqueCurrent, actual.TorqueCurrent)) {
      DriverStation.reportWarning("[TorqueCurrent] Expected: " + expected.TorqueCurrent + ", Actual: " + actual.TorqueCurrent, false);
      return false;
    }

    if (!isEqual(expected.Slot0, actual.Slot0)) {
      DriverStation.reportWarning("[Slot0] Expected: " + expected.Slot0 + ", Actual: " + actual.Slot0, false);
      return false;
    }

    if (!isEqual(expected.Slot1, actual.Slot1)) {
      DriverStation.reportWarning("[Slot1] Expected: " + expected.Slot1 + ", Actual: " + actual.Slot1, false);
      return false;
    }

    if (!isEqual(expected.Slot2, actual.Slot2)) {
      DriverStation.reportWarning("[Slot2] Expected: " + expected.Slot2 + ", Actual: " + actual.Slot2, false);
      return false;
    }

    if (!isEqual(expected.MotionMagic, actual.MotionMagic)) {
      DriverStation.reportWarning("[MotionMagic] Expected: " + expected.MotionMagic + ", Actual: " + actual.MotionMagic, false);
      return false;
    }

    // TODO: Check other values
    return true;
  }

  public static boolean isEqual(Slot0Configs a, Slot0Configs b) {
    return EqualsUtility.epsilonEquals(a.kP, b.kP, kEps)
        && EqualsUtility.epsilonEquals(a.kI, b.kI, kEps)
        && EqualsUtility.epsilonEquals(a.kD, b.kD, kEps)
        && EqualsUtility.epsilonEquals(a.kS, b.kS, kEps)
        && EqualsUtility.epsilonEquals(a.kV, b.kV, kEps)
        && EqualsUtility.epsilonEquals(a.kA, b.kA, kEps)
        && EqualsUtility.epsilonEquals(a.kG, b.kG, kEps);
  }

  public static boolean isEqual(Slot1Configs a, Slot1Configs b) {
    return EqualsUtility.epsilonEquals(a.kP, b.kP, kEps)
        && EqualsUtility.epsilonEquals(a.kI, b.kI, kEps)
        && EqualsUtility.epsilonEquals(a.kD, b.kD, kEps)
        && EqualsUtility.epsilonEquals(a.kS, b.kS, kEps)
        && EqualsUtility.epsilonEquals(a.kV, b.kV, kEps)
        && EqualsUtility.epsilonEquals(a.kA, b.kA, kEps)
        && EqualsUtility.epsilonEquals(a.kG, b.kG, kEps);
  }

  public static boolean isEqual(Slot2Configs a, Slot2Configs b) {
    return EqualsUtility.epsilonEquals(a.kP, b.kP, kEps)
        && EqualsUtility.epsilonEquals(a.kI, b.kI, kEps)
        && EqualsUtility.epsilonEquals(a.kD, b.kD, kEps)
        && EqualsUtility.epsilonEquals(a.kS, b.kS, kEps)
        && EqualsUtility.epsilonEquals(a.kV, b.kV, kEps)
        && EqualsUtility.epsilonEquals(a.kA, b.kA, kEps)
        && EqualsUtility.epsilonEquals(a.kG, b.kG, kEps);
  }

  public static boolean isEqual(MotionMagicConfigs a, MotionMagicConfigs b) {
    return EqualsUtility.epsilonEquals(a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity, kEps)
        && EqualsUtility.epsilonEquals(a.MotionMagicAcceleration, b.MotionMagicAcceleration, kEps)
        && EqualsUtility.epsilonEquals(a.MotionMagicJerk, b.MotionMagicJerk, kEps)
        && EqualsUtility.epsilonEquals(a.MotionMagicExpo_kV, b.MotionMagicExpo_kV, kEps)
        && EqualsUtility.epsilonEquals(a.MotionMagicExpo_kA, b.MotionMagicExpo_kA, kEps);
  }

  public static boolean isEqual(MotorOutputConfigs a, MotorOutputConfigs b) {
    return a.NeutralMode == b.NeutralMode
        && a.Inverted == b.Inverted
        && EqualsUtility.epsilonEquals(a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband, kEps);
  }

  public static boolean isEqual(CurrentLimitsConfigs a, CurrentLimitsConfigs b) {
    return a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable
        && EqualsUtility.epsilonEquals(a.StatorCurrentLimit, b.StatorCurrentLimit, kEps)
        && a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable
        && EqualsUtility.epsilonEquals(a.SupplyCurrentLimit, b.SupplyCurrentLimit, kEps)
        && EqualsUtility.epsilonEquals(a.SupplyCurrentLowerLimit, b.SupplyCurrentLowerLimit, kEps)
        && EqualsUtility.epsilonEquals(a.SupplyCurrentLowerTime, b.SupplyCurrentLowerTime, kEps);
  }

  public static boolean isEqual(TorqueCurrentConfigs a, TorqueCurrentConfigs b) {
    return EqualsUtility.epsilonEquals(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent, kEps)
        && EqualsUtility.epsilonEquals(a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent, kEps)
        && EqualsUtility.epsilonEquals(a.TorqueNeutralDeadband, b.TorqueNeutralDeadband, kEps);
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

  /**
   * Invokes the specified CTRE function until it is successful or the number of tries is exceeded.
   * Sets the specified alert if the function fails.
   *
   * @param function CTRE function to invoke
   * @param alert alert to set if the function fails
   * @param numTries number of times to try the function
   * @return true if the function was successful, false otherwise
   */
  public static boolean checkErrorAndRetry(
      Supplier<StatusCode> function, Alert alert, int numTries) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      alert.setText("Retrying CTRE Device Config " + code.getName());
      alert.set(true);
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      alert.setText(
          "Failed to execute phoenix 6 api call after "
              + numTries
              + " attempts. "
              + code.getDescription());
      alert.set(true);
      return false;
    }
    alert.set(false);
    return true;
  }

  /**
   * Checks the specified status code and throws an exception with the specified message of it is
   * not OK.
   *
   * @param statusCode status code to check
   * @param message message to include with the exception if the status code is not OK
   */
  public static void checkErrorWithThrow(StatusCode statusCode, String message) {
    if (statusCode != StatusCode.OK) {
      throw new RuntimeException(message + " " + statusCode);
    }
  }

  /**
   * Invokes the specified CTRE function until it is successful or five tries are exceeded.
   *
   * @param function CTRE function to invoke
   * @param alert Alert to set if the function fails
   * @return true if the function was successful, false otherwise
   */
  public static boolean checkErrorAndRetry(Supplier<StatusCode> function, Alert alert) {
    return checkErrorAndRetry(function, alert, 5);
  }

  /**
   * Applies the specified configuration to the specified TalonFX and checks that the configuration
   * was applied successfully. If not, retries the specified number of tries; eventually, setting
   * the specified alert if the number of tries is exceeded.
   *
   * @param talon TalonFX to which to apply the configuration
   * @param config TalonFX configuration to apply
   * @param alert alert to set if the configuration is not applied successfully
   * @param numTries number of times to try to apply the configuration
   * @return true if the configuration was applied successfully, false otherwise
   */
  public static boolean applyAndCheckConfiguration(
      TalonFX talon, TalonFXConfiguration config, Alert alert, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (checkErrorAndRetry(() -> talon.getConfigurator().apply(config), alert)) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(talon, config, alert)) {
          return true;
        } else {
          alert.setText(
              "Failed to verify config for talon ["
                  + talon.getDescription()
                  + "] (attempt "
                  + (i + 1)
                  + " of "
                  + numTries
                  + ")");
          alert.set(true);
        }
      } else {
        alert.setText(
            "Failed to apply config for talon ["
                + talon.getDescription()
                + "] (attempt "
                + (i + 1)
                + " of "
                + numTries
                + ")");
        alert.set(true);
      }
    }
    alert.setText("Failed to apply config for talon after " + numTries + " attempts");
    alert.set(true);
    return false;
  }

  /**
   * Reads the configuration from the specified TalonFX and verifies that it matches the specified
   * configuration. If the configuration does not match, sets the specified alert.
   *
   * @param talon TalonFX from which to read the configuration
   * @param config TalonFX configuration to verify
   * @param alert alert to set if the configuration does not match
   * @return true if the configuration was read and matched, false otherwise
   */
  public static boolean readAndVerifyConfiguration(
      TalonFX talon, TalonFXConfiguration config, Alert alert) {
    TalonFXConfiguration readConfig = new TalonFXConfiguration();
    if (!checkErrorAndRetry(() -> talon.getConfigurator().refresh(readConfig), alert)) {
      // could not get config!
      alert.setText("Failed to read config for talon [" + talon.getDescription() + "]");
      alert.set(true);
      return false;
    } else if (!TalonFXConfigsEqual(config, readConfig)) {
      // configs did not match
      alert.setText("Configuration verification failed for talon [" + talon.getDescription() + "]");
      alert.set(true);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  /**
   * Applies the specified configuration to the specified TalonFX and checks that the configuration
   * was applied successfully. If not, retries five times; eventually, setting the specified alert
   * if the number of tries is exceeded.
   *
   * @param talon TalonFX to which to apply the configuration
   * @param config TalonFX configuration to apply
   * @param alert alert to set if the configuration is not applied successfully
   * @return true if the configuration was applied successfully, false otherwise
   */
  public static boolean applyAndCheckConfiguration(
      TalonFX talon, TalonFXConfiguration config, Alert alert) {
    return applyAndCheckConfiguration(talon, config, alert, 5);
  }

  // Copyright (c) 2025 FRC 6328
  // http://github.com/Mechanical-Advantage
  //
  // Use of this source code is governed by an MIT-style
  // license that can be found in the LICENSE file at
  // the root directory of this project.

  /** Signals for synchronized refresh. */
  private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

  private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
    if (canivore) {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
      System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
      System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
      canivoreSignals = newSignals;
    } else {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
      System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
      System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
      rioSignals = newSignals;
    }
  }

  /** Refresh all registered signals. */
  public static void refreshAll() {
    if (canivoreSignals.length > 0) {
      checkError(
          BaseStatusSignal.refreshAll(canivoreSignals),
          "failed to refresh signals on CANivore:",
          canivoreSignalsAlert);
    }
    if (rioSignals.length > 0) {
      checkError(
          BaseStatusSignal.refreshAll(rioSignals),
          "failed to refresh signals on RIO:",
          rioSignalsAlert);
    }
  }
}