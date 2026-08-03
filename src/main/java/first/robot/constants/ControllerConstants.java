package first.robot.constants;

public class ControllerConstants {
    public static final SimControllerType kSimulationControllerType = SimControllerType.XBOX;
    public static final double kDebounceTimeSeconds = 0.1;

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