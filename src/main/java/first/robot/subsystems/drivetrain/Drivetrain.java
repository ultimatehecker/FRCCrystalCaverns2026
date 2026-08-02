package first.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.minolib.advantagekit.LoggedTracer;
import first.minolib.vision.FieldPoseEstimation;
import first.robot.RobotState;
import first.robot.constants.ControllerConstants;
import first.robot.constants.DrivetrainConstants;

public class Drivetrain extends Mechanism {
    private final RobotState robotState;

    private final DrivetrainIO io;
    private final DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

    private final String[] moduleNames = {"Drivetrain/FL", "Drivetrain/FR", "Drivetrain/BL", "Drivetrain/BR"};

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

            if (isFieldCentric.getAsBoolean()) {
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