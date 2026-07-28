package frc.minolib.controller;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class ControllerConstants {
    public static final SimControllerType kSimulationControllerType = SimControllerType.XBOX;
    public static final Time kDebounceTimeSeconds = Seconds.of(0.1);

    public static final double kControllerDeadband = 0.05;

    public enum SimControllerType {
        XBOX,
        DUAL_SENSE,
        VEX
    }

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kDriverConsolePort = 2;
    public static final int kOperatorConsolePort = 3;
}