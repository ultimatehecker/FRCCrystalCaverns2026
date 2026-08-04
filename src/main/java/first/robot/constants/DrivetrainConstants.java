package first.robot.constants;

import static org.wpilib.units.Units.KilogramSquareMeters;

import java.util.Arrays;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.MomentOfInertia;

import first.minolib.math.MathUtility;
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

    public class AutoAlignConstants {
        public enum CaveTarget {
            LOW,
            HIGH,
            GEMSTONE
        }

        public enum ClassifierTarget {
            LOWER,
            UPPER
        }

        /* These are temporary for now, the actual offsets will be computed when other subsystems are further programmed (these are good temp)*/
        public static final double kLowerClassifierBumperGap = Units.inchesToMeters(6.0);
        public static final double kUpperClassifierBumperGap = Units.inchesToMeters(12.0);
        public static final double kJewelryBumperGap = Units.inchesToMeters(6.0);

        private static final double kCaveCircumradius = FieldConstants.kBlueCaveOrigin.getDistance(FieldConstants.kBlueCaveVertices[0]);
        private static final double kCaveApothem = kCaveCircumradius * Math.cos(Math.PI / 8.0);

        public static Pose2d getPoseForAlliance(Pose2d bluePose, boolean isRedAlliance) {
            if (!isRedAlliance) {
                return bluePose;
            }

            return new Pose2d(FieldConstants.kFieldLength - bluePose.getX(), bluePose.getY(), Rotation2d.k180deg.minus(bluePose.getRotation()));
        }

        private static List<Rotation2d> getBlueCaveApproachDirections(CaveTarget target) {
            Translation2d[] referencePoints = switch (target) {
                case LOW -> FieldConstants.kBlueLowerBranches;
                case HIGH -> FieldConstants.kBlueUpperBranches;
                case GEMSTONE -> FieldConstants.kBlueLowerBranches;
            };

            return Arrays.stream(referencePoints)
                .map(referencePoint -> referencePoint.minus(FieldConstants.kBlueCaveOrigin).getAngle())
                .toList();
        }

        private static Translation2d getBlueCaveEdgePoint(Rotation2d outwardDirection) {

            double directionRadians = outwardDirection.getRadians();
            double faceSpacingRadians = Math.PI / 4.0;
            double firstFaceNormalRadians = Math.PI / 8.0;

            long nearestFaceIndex = Math.round((directionRadians - firstFaceNormalRadians) / faceSpacingRadians);

            double nearestFaceNormalRadians = firstFaceNormalRadians + nearestFaceIndex * faceSpacingRadians;
            double differenceFromFaceNormal = MathUtility.constrainAngleNegPiToPi(directionRadians - nearestFaceNormalRadians);
            double centerToEdgeDistance = kCaveApothem / Math.cos(differenceFromFaceNormal);

            return FieldConstants.kBlueCaveOrigin.plus(new Translation2d(centerToEdgeDistance, outwardDirection));
        }

        public static List<Pose2d> getCaveAlignmentPoses(CaveTarget target, boolean isRedAlliance) {
            List<Rotation2d> blueApproachDirections = getBlueCaveApproachDirections(target);

            return blueApproachDirections.stream()
                .map(AutoAlignConstants::createBlueCaveAlignmentPose)
                .map(pose -> getPoseForAlliance(pose, isRedAlliance))
                .toList();
        }

        private static Pose2d createBlueCaveAlignmentPose(Rotation2d outwardDirection) {
            Translation2d caveEdgePoint = getBlueCaveEdgePoint(outwardDirection);
            Translation2d robotCenter = caveEdgePoint.plus(new Translation2d(kBumperLengthX / 2.0, outwardDirection));

            Rotation2d robotHeading =
                    outwardDirection.plus(Rotation2d.k180deg);

            return new Pose2d(robotCenter, robotHeading);
        }

        public static Pose2d getJewelryAlignmentPose(boolean isRedAlliance) {
            double jewelryRadiusAlongX = FieldConstants.kCenterOrigin.getDistance(FieldConstants.kJewelryVertices[0]);
            double jewelryCenterToRobotCenter = jewelryRadiusAlongX + kJewelryBumperGap + (kBumperLengthX / 2.0);

            Pose2d blueAlignmentPose = new Pose2d(FieldConstants.kCenterOrigin.getX() - jewelryCenterToRobotCenter, FieldConstants.kFieldWidth / 2.0, Rotation2d.kZero);

            return getPoseForAlliance(blueAlignmentPose, isRedAlliance);
        }

        public static Pose2d getClassifierAlignmentPose(Pose2d blueClassifierFaceCenter, ClassifierTarget target, boolean isRedAlliance) {
            double bumperGap = switch (target) {
                case LOWER -> kLowerClassifierBumperGap;
                case UPPER -> kUpperClassifierBumperGap;
            };

            Rotation2d outwardDirection = blueClassifierFaceCenter.getRotation();

            double faceToRobotCenter = bumperGap + (kBumperLengthX / 2.0);
            Translation2d robotCenter = blueClassifierFaceCenter.getTranslation().plus(new Translation2d(faceToRobotCenter, outwardDirection));

            Rotation2d robotHeading = outwardDirection.plus(Rotation2d.k180deg);

            return getPoseForAlliance(new Pose2d(robotCenter, robotHeading), isRedAlliance);
        }
    }
}