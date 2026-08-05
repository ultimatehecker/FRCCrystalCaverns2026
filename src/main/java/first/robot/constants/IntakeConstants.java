package first.robot.constants;

import org.wpilib.math.system.DCMotor;

import first.Constants;
import first.minolib.hardware.MinoCANDevice;

public class IntakeConstants {
    public static final double kMaximumRotationalVelocity = 4.2;
    public static final double kMaximumRotationalAcceleration = 6.0;
    public static final double kMaximumRotationalJerk = 200 / 60;

    public static final MinoCANDevice kMotor = new MinoCANDevice(14, Constants.kS1);

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final boolean kMotorInverted = false;
    public static final double kMotorReduction = (25.0 / 1.0) * (36.0 / 48.0) * (190.0 / 10.0);
    public static final DCMotor kSimulatedGearbox = DCMotor.getKrakenX60Foc(1);

    public static final double kMotorStatorLimit = 100.0;
    public static final double kMotorSupplyLimit = 40.0;
}