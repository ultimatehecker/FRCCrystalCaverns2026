package frc.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.minolib.controller.ControllerConstants;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.swerve.MapleSimulatedSwerveDrivetrain;
import frc.robot.RobotState;
import frc.robot.constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
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

    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.ApplyRobotSpeeds robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.ApplyFieldSpeeds idleRequest = new SwerveRequest.ApplyFieldSpeeds()
        .withSpeeds(new ChassisSpeeds())
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private boolean stopped = true;
    public final Trigger isStopped = new Trigger(() -> stopped).debounce(0.1);

    public Drivetrain(RobotState robotState, DrivetrainIO io) {
        this.robotState = robotState;
        this.io = io;

        continuousTracking.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs, moduleInputs);
        io.logModules(inputs);

        for (int i = 0; i < moduleInputs.length; i++) {
            Logger.processInputs(moduleNames[i], moduleInputs[i]);
        }

        Logger.processInputs("Drivetrain", inputs);

        stopped = 
            MathUtil.isNear(0.0, inputs.Speeds.vxMetersPerSecond, DrivetrainConstants.kStoppedLinearTolerenceMetersPerSecond) &&
            MathUtil.isNear(0.0, inputs.Speeds.vyMetersPerSecond, DrivetrainConstants.kStoppedLinearTolerenceMetersPerSecond) &&
            MathUtil.isNear(0.0, inputs.Speeds.omegaRadiansPerSecond, DrivetrainConstants.kStoppedRotationalTolerenceRadiansPerSecond);

        Logger.recordOutput("Drivetrain/IsStopped?", stopped);
    }

    public Command drive(DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier isFieldCentric) {
        return run(() -> {
            ChassisSpeeds speeds = calculateSpeedsBasedOnJoystickInputs(throttleSupplier, strafeSupplier, rotationSupplier);

            if (isFieldCentric.getAsBoolean()) {
                applyRequest(teleopRequest
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond));
            } else {
                applyRequest(robotSpeedsRequest.withSpeeds(speeds));
            }
        }).withName("Standard Teloeop Drive");
    }

    public Command driveFacingAngle(DoubleSupplier velocityXMetersPerSecond, DoubleSupplier velocityYMetersPerSecond, Supplier<Rotation2d> targetHeading) {
        return run(() -> applyRequest(continuousTracking
            .withVelocityX(velocityXMetersPerSecond.getAsDouble())
            .withVelocityY(velocityYMetersPerSecond.getAsDouble())
            .withTargetDirection(targetHeading.get()))).withName("Drive Continuous Tracking");
    }

    public Command followRobotRelativeSpeeds(Supplier<ChassisSpeeds> speeds) {
        return run(() -> applyRequest(robotSpeedsRequest.withSpeeds(speeds.get()))).withName("Follow Robot-Relative Speeds");
    }

    public Command xLock() {
        return run(() -> applyRequest(brakeRequest)).withName("X Lock");
    }

    public Command idle() {
        return run(() -> applyRequest(idleRequest)).withName("Idle");
    }

    private void applyRequest(SwerveRequest request) {
        io.setSwerveRequest(request);
    }

    public void addVisionMeasurement(WeightedPoseEstimate poseEstimate) {
        io.addVisionMeasurement(poseEstimate);
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    public void resetRotationBasedOnAlliance() {
        io.resetRotation();
    }

    public MapleSimulatedSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof DrivetrainIOSimulation simIo) {
            return simIo.getMapleSimDrive();
        }
        return null;
    }

    private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier omega) {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double magnitudeX = MathUtil.applyDeadband(throttle.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeY = MathUtil.applyDeadband(strafe.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeTheta = MathUtil.applyDeadband(omega.getAsDouble(), ControllerConstants.kControllerDeadband);

        double velocityX = magnitudeX * DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond;
        double velocityY = magnitudeY * DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond;
        double velocityTheta = magnitudeTheta * DrivetrainConstants.kMaximumRotationalVelocityRadiansPerSecond;

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(robotState.getLatestMeasuredRobotRelativeChassisSpeeds().omegaRadiansPerSecond * -0.03);

        return ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(velocityX, velocityY, velocityTheta), robotState.getLatestFieldToRobot().getValue().getRotation()), 
            robotState.getLatestFieldToRobot().getValue().getRotation().plus(skewCompensationFactor)
        );
    }
}