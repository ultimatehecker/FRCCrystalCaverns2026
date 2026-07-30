package frc.robot.constants;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;

import frc.minolib.swerve.CTRESwerveDrivetrainConstants;
import frc.minolib.swerve.SwerveModuleType;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class DrivetrainConstants {
    public static final SwerveModuleType kSwerveModuleType = SwerveModuleType.MK4N_L2;

    public static final double kMaximumLinearVelocityMetersPerSecond = 4.4;
    public static final double kMaximumLinearAccelerationMetersPerSecond2 = 6.0;
    public static final double kMaximumRotationalVelocityRadiansPerSecond = 5 * Math.PI;
    public static final double kMaximumRotationalAccelerationRadiansPerSecond2 = 6 * Math.PI;

    public static final double dP = 10.0;
    public static final double dI = 0.0;
    public static final double dD = 0.0;
    public static final double dS = 0.0;
    public static final double dV = 1.5;
    public static final double dA = 0.0;

    public static final boolean kDriveMotorInverted = false;
    public static final double kDriveMotorReduction = kSwerveModuleType.getDriveReduction();
    public static final double kDriveMotorSupplyCurrentLimit = 120;
    public static final DCMotor kDriveSimulatedGearbox = DCMotor.getKrakenX60Foc(1);

    public static final double sP = 100.0;
    public static final double sI = 0.0;
    public static final double sD = 0.5;
    public static final double sS = 0.1;
    public static final double sV = 0.0;
    public static final double sA = 0.0;

    public static final boolean kSteerMotorInverted = true;
    public static final double kSteerMotorReduction = kSwerveModuleType.getSteerReduction();
    public static final double kSteerMotorStatorCurrentLimit = 60;
    public static final DCMotor kSteerSimulatedGearbox = DCMotor.getKrakenX44Foc(1);

    public static final double kWheelRadius = 0.0482;
    public static final double kTrackWidth = 0.55245;
    public static final double kWheelBase = 0.55245;
    public static final double kBumperLengthY = 0.7874;
    public static final double kBumperLengthX = 0.7874;
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
    public static final double kRobotCOGHeightMeters = 0.15;
    public static final MomentOfInertia kRobotMOI = MomentOfInertia.ofBaseUnits(6.883, KilogramSquareMeters);
    public static final MomentOfInertia kSwerveModuleSteerMOI = MomentOfInertia.ofBaseUnits(0.02, KilogramSquareMeters);
    public static final double kWheelCOF = 1.0;

    public static final DriveTrainSimulationConfig kMapleSimConfiguration = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(kModuleTranslations)
        .withRobotMass(Kilograms.of(kRobotMassKilograms))
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                kDriveSimulatedGearbox,
                kSteerSimulatedGearbox,
                kDriveMotorReduction,
                kSteerMotorReduction,
                Volts.of(0.1),
                Volts.of(0.1),
                Meters.of(kWheelRadius),
                kSwerveModuleSteerMOI,
                kWheelCOF
            )
        );

    public static final CTRESwerveDrivetrainConstants kDrivetrain = TunerConstants.instantateConstants();
}