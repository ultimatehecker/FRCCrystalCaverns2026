package first.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.minolib.advantagekit.LoggedTracer;
import first.minolib.advantagekit.LoggedTunableNumber;
import first.minolib.vision.FieldPoseEstimation;
import first.robot.RobotState;
import first.robot.constants.ControllerConstants;
import first.robot.constants.DrivetrainConstants;

public class Drivetrain extends Mechanism {
    private final RobotState robotState;

    private final DrivetrainIO io;
    private final DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

    private final String[] moduleNames = {"Drivetrain/FL", "Drivetrain/FR", "Drivetrain/BL", "Drivetrain/BR"};

    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/kP", DrivetrainConstants.kDriveHolonomickP);
    private static final LoggedTunableNumber drivekI = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/kI", DrivetrainConstants.kDriveHolonomickI);
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/kD", DrivetrainConstants.kDriveHolonomickD);
    private static final LoggedTunableNumber driveMaxVelocity = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/Max Velocity", DrivetrainConstants.kDriveHolonomicMaxVelocity);
    private static final LoggedTunableNumber driveMaxAcceleration = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/Max Acceleration", DrivetrainConstants.kDriveHolonomicMaxAcceleration);

    private static final LoggedTunableNumber rotkP = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kP", DrivetrainConstants.kRotationalHolonomickP);
    private static final LoggedTunableNumber rotkI = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kI", DrivetrainConstants.kRotationalHolonomickI);
    private static final LoggedTunableNumber rotkD = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kD", DrivetrainConstants.kRotationalHolonomickD);
    private static final LoggedTunableNumber rotMaxVelocity = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/Max Velocity", DrivetrainConstants.kRotationalHolonomicMaxVelocity);
    private static final LoggedTunableNumber rotMaxAcceleration = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/Max Acceleration", DrivetrainConstants.kRotationalHolonomicMaxAcceleration);

    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] { 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged()
     };

    private final SwerveRequest.FieldCentricFacingAngle continuousTracking = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withHeadingPID(3.0, 0.0, 0.15) 
        .withDeadband(0.02);

    private final SwerveRequest.FieldCentric teleopRequestFC = new SwerveRequest.FieldCentric()
        .withDesaturateWheelVelocities(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.RobotCentric teleopRequestRC = new SwerveRequest.RobotCentric()
        .withDesaturateWheelVelocities(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.ApplyRobotVelocity robotVelocityRequest = new SwerveRequest.ApplyRobotVelocity()
        .withDesaturateWheelVelocities(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.SwerveDriveBrake idleRequest = new SwerveRequest.SwerveDriveBrake()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final ChassisVelocities zeroChassisVelocities = new ChassisVelocities(0, 0, 0);

    private boolean isFieldCentric = true;
    private boolean fieldCentricPreviousState = false;

    public Drivetrain(RobotState robotState, DrivetrainIO io) {
        this.robotState = robotState;
        this.io = io;

        continuousTracking.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.updateInputs(inputs, moduleInputs);
        io.logModules(inputs);

        for (int i = 0; i < moduleInputs.length; i++) {
            Logger.processInputs(moduleNames[i], moduleInputs[i]);
        }

        Logger.processInputs("Drivetrain", inputs);
        LoggedTracer.record("DrivetrainPeriodic");
    }

    public Command drive(DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier isFieldCentric) {
        return runRepeatedly(() -> {
            ChassisVelocities speeds = calculateSpeedsBasedOnJoystickInputs(throttleSupplier, strafeSupplier, rotationSupplier);

            if (isFieldCentric.getAsBoolean() && !fieldCentricPreviousState) {
                this.isFieldCentric = !this.isFieldCentric;
            }

            fieldCentricPreviousState = isFieldCentric.getAsBoolean();

            if (this.isFieldCentric) {
                applyRequest(teleopRequestFC
                    .withVelocityX(speeds.vx)
                    .withVelocityY(speeds.vy)
                    .withRotationalRate(speeds.omega)
                );
            } else {
                applyRequest(teleopRequestRC
                    .withVelocityX(speeds.vx)
                    .withVelocityY(speeds.vy)
                    .withRotationalRate(speeds.omega)
                );
            }
        }).withPriority(Command.LOWEST_PRIORITY).named("Standard Teleop Drive");
    }

    public Command driveToPose(Supplier<Pose2d> targetSupplier, Supplier<Pose2d> robotSupplier) {
        return run(coroutine -> {
            ProfiledPIDController translationController = new ProfiledPIDController(
                drivekP.get(),
                drivekI.get(),
                drivekD.get(),
                new TrapezoidProfile.Constraints(
                    driveMaxVelocity.get(),
                    driveMaxAcceleration.get()
                )
            );

            ProfiledPIDController rotationController = new ProfiledPIDController(
                rotkP.get(),
                rotkI.get(),
                rotkD.get(),
                new TrapezoidProfile.Constraints(
                    rotMaxVelocity.get(),
                    rotMaxAcceleration.get()
                )
            );

            rotationController.enableContinuousInput(-Math.PI, Math.PI);

            Pose2d targetPose = targetSupplier.get();
            Pose2d currentPose = robotSupplier.get();
            ChassisVelocities currentSpeeds = inputs.Velocity;

            Translation2d translationError = targetPose.minus(currentPose).getTranslation();
            Rotation2d directionToTarget = translationError.getAngle();

            double velocityTowardTarget = currentSpeeds.vx * directionToTarget.getCos() + currentSpeeds.vy * directionToTarget.getSin();

            translationController.reset(translationError.getNorm(), -velocityTowardTarget);
            rotationController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omega);

            while (true) {
                 if (drivekP.hasChanged(hashCode()) || drivekI.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
                    translationController.setPID(drivekP.get(), drivekI.get(), drivekD.get());
                }

                if (rotkP.hasChanged(hashCode()) || rotkI.hasChanged(hashCode()) || rotkD.hasChanged(hashCode())) {
                    rotationController.setPID(rotkP.get(), rotkI.get(), rotkD.get());
                }

                if (driveMaxVelocity.hasChanged(hashCode()) || driveMaxAcceleration.hasChanged(hashCode())) {
                    translationController.setConstraints(
                            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
                }

                if (rotMaxVelocity.hasChanged(hashCode()) || rotMaxAcceleration.hasChanged(hashCode())) {
                    rotationController.setConstraints(new TrapezoidProfile.Constraints(rotMaxVelocity.get(), rotMaxAcceleration.get()));
                }

                targetPose = targetSupplier.get();
                currentPose = robotSupplier.get();

                translationError = targetPose.minus(currentPose).getTranslation();
                directionToTarget = translationError.getAngle();

                double translationOutput = translationController.calculate(translationError.getNorm(), 0.0);
                double rotationOutput = rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

                Translation2d translationVelocity = new Translation2d(-translationOutput, directionToTarget);

                applyRequest(robotVelocityRequest.withVelocity(
                    new ChassisVelocities(
                        translationVelocity.getX(),
                        translationVelocity.getY(),
                        rotationOutput
                    )
                ));

                Logger.recordOutput("DriveToPose/Target Pose", targetPose);
                Logger.recordOutput("DriveToPose/Translation Output", translationOutput);
                Logger.recordOutput("DriveToPose/Rotation Output", rotationOutput);
                Logger.recordOutput("DriveToPose/Translation Error", translationError.getNorm());
                Logger.recordOutput("DriveToPose/Rotation Error", targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
                Logger.recordOutput("DriveToPose/Translation Velocity", translationVelocity);
                Logger.recordOutput("DriveToPose/Direction to Target", directionToTarget);

                coroutine.yield();
            }
        }).whenCanceled(this::stop).named("Drive To Pose");
    }

    public Command driveFacingAngle(DoubleSupplier fieldVelocityX, DoubleSupplier fieldVelocityY, Supplier<Rotation2d> targetHeadingSupplier) {
        return runRepeatedly(() -> applyRequest(continuousTracking
            .withVelocityX(fieldVelocityX.getAsDouble())
            .withVelocityY(fieldVelocityY.getAsDouble())
            .withTargetDirection(targetHeadingSupplier.get()))
        ).whenCanceled(this::stop).named("Drive Facing Angle");
    }

    public Command xLock() {
        return run(coroutine -> {
            applyRequest(idleRequest);
            coroutine.park();
        }).whenCanceled(this::stop).named("X Lock");
    }

    private void applyRequest(SwerveRequest request) {
        io.setSwerveRequest(request);
    }

    public void addVisionMeasurement(FieldPoseEstimation fieldPoseEstimation) {
        io.addVisionMeasurement(fieldPoseEstimation);
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    public void resetRotationBasedOnAlliance() {
        io.resetRotation();
    }

    private ChassisVelocities calculateSpeedsBasedOnJoystickInputs(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier omega) {
        if (MatchState.getAlliance().isEmpty()) {
            return zeroChassisVelocities;
        }

        double magnitudeX = MathUtil.applyDeadband(throttle.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeY = MathUtil.applyDeadband(strafe.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeTheta = MathUtil.applyDeadband(omega.getAsDouble(), ControllerConstants.kControllerDeadband);

        double velocityX = magnitudeX * DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond;
        double velocityY = magnitudeY * DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond;
        double velocityTheta = magnitudeTheta * DrivetrainConstants.kMaximumRotationalVelocityRadiansPerSecond;

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(robotState.getLatestMeasuredRobotRelativeChassisSpeeds().omega * -0.03);
        Rotation2d heading = robotState.getLatestFieldToRobot().getValue().getRotation();

        return new ChassisVelocities(velocityX, velocityY, velocityTheta)
            .toRobotRelative(heading)
            .toFieldRelative(heading.plus(skewCompensationFactor));
    }

    private void stop() {
        applyRequest(robotVelocityRequest.withVelocity(zeroChassisVelocities));
    }
}