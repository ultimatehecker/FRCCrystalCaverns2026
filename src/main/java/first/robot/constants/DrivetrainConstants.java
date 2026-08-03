package first.robot.constants;

import static org.wpilib.units.Units.KilogramSquareMeters;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.MomentOfInertia;

import first.minolib.swerve.CTRESwerveDrivetrainConstants;
import first.minolib.swerve.SwerveModuleType;
import first.robot.Robot;
import first.robot.subsystems.drivetrain.TunerConstants;

public class DrivetrainConstants {
    public static final SwerveModuleType kSwerveModuleType = SwerveModuleType.MK4N_L2;

    public static final double kMaximumLinearVelocityMetersPerSecond = 4.22;
    public static final double kMaximumLinearAccelerationMetersPerSecond2 = 7.83;
    public static final double kMaximumRotationalVelocityRadiansPerSecond = 5 * Math.PI;
    public static final double kMaximumRotationalAccelerationRadiansPerSecond2 = 10 * Math.PI;

    public static final double dP = 10.0;
    public static final double dI = 0.0;
    public static final double dD = 0.0;
    public static final double dS = 0.0;
    public static final double dV = 1.5;
    public static final double dA = 0.0;

    public static final boolean kDriveMotorInverted = false;
    public static final double kDriveMotorReduction = kSwerveModuleType.getDriveReduction();
    public static final double kDriveMotorStatorCurrentLimit = 120;
    public static final double kDriveMotorSupplyCurrentLimit = 70;
    public static final DCMotor kDriveSimulatedGearbox = DCMotor.getKrakenX60Foc(1);

    public static final double sP = 100.0;
    public static final double sI = 0.0;
    public static final double sD = 0.5;
    public static final double sS = 0.1;
    public static final double sV = 0.0;
    public static final double sA = 0.0;

    public static final boolean kSteerMotorInverted = true;
    public static final double kSteerMotorReduction = kSwerveModuleType.getSteerReduction();
    public static final double kSteerMotorStatorCurrentLimit = 70;
    public static final double kSteerMotorSupplyCurrentLimit = 40;
    public static final DCMotor kSteerSimulatedGearbox = DCMotor.getKrakenX44Foc(1);

    public static final double kWheelRadius = 0.0482;
    public static final double kTrackWidth = 0.5588;
    public static final double kWheelBase = 0.6096;
    public static final double kBumperLengthY = 0.8636;
    public static final double kBumperLengthX = 0.9144;
    public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0)
    };

    public static final double kStoppedLinearTolerenceMetersPerSecond = 0.05;
    public static final double kStoppedRotationalTolerenceRadiansPerSecond = 0.05;
    
    public static final double kRobotMassKilograms = 67.5;
    public static final double kRobotCOGHeightMeters = 0.127;
    public static final MomentOfInertia kRobotMOI = MomentOfInertia.ofBaseUnits(6.883, KilogramSquareMeters);
    public static final MomentOfInertia kSwerveModuleSteerMOI = MomentOfInertia.ofBaseUnits(0.02, KilogramSquareMeters);
    public static final double kWheelCOF = 1.0;

    public static final double kDriveHolonomickP = Robot.isSimulation() ? 5.0 : 0.0;
    public static final double kDriveHolonomickI = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kDriveHolonomickD = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kDriveHolonomicMaxVelocity = Robot.isSimulation() ? kMaximumLinearVelocityMetersPerSecond : 0.0;
    public static final double kDriveHolonomicMaxAcceleration = Robot.isSimulation() ? kMaximumLinearAccelerationMetersPerSecond2 : 0.0;

    public static final double kRotationalHolonomickP = Robot.isSimulation() ? 5.0 : 0.0;
    public static final double kRotationalHolonomickI = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kRotationalHolonomickD = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kRotationalHolonomicMaxVelocity = Robot.isSimulation() ? kMaximumRotationalVelocityRadiansPerSecond : 0.0;
    public static final double kRotationalHolonomicMaxAcceleration = Robot.isSimulation() ? kMaximumRotationalAccelerationRadiansPerSecond2 : 0.0;

    public static final CTRESwerveDrivetrainConstants kDrivetrain = TunerConstants.instantateConstants();
}