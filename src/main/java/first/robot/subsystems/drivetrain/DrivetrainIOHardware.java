package first.robot.subsystems.drivetrain;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.minolib.hardware.MinoCANBus.CANBusLane;
import first.minolib.phoenix.PhoenixUtility;
import first.minolib.vision.FieldPoseEstimation;
import first.robot.RobotState;

public class DrivetrainIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DrivetrainIO {
    private RobotState robotState;
    private final String[] moduleNames = {"Drivetrain/FL", "Drivetrain/FR", "Drivetrain/BL", "Drivetrain/BR"};
    private String[][] outputNames;

    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> roll;

    private final StatusSignal<AngularVelocity> angularYawVelocity;
    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;

    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;
    private final StatusSignal<LinearAcceleration> accelerationZ;

    @FunctionalInterface
    private interface ModuleInputUpdater {
        public void update(ModuleIOInputs moduleInputs, DrivetrainIOInputs drivetrainInputs);
    }

    private final Map<Integer, ModuleInputUpdater> moduleInputUpdaters = new HashMap<>();

    public DrivetrainIOHardware(RobotState robotState, SwerveDrivetrainConstants drivetrainConstants, @SuppressWarnings("unchecked") SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants);
        this.robotState = robotState;

        for (int i = 0; i < moduleNames.length; i++) {
            int moduleIndex = i;

            var module = getModule(moduleIndex);
            TalonFX driveMotor = module.getDriveMotor();
            TalonFX steerMotor = module.getSteerMotor();

            StatusSignal<Current> driveStatorCurrent = driveMotor.getStatorCurrent(false);
            StatusSignal<Current> driveSupplyCurrent = driveMotor.getSupplyCurrent(false);
            StatusSignal<Voltage> driveAppliedVoltage = driveMotor.getMotorVoltage(false);
            StatusSignal<Temperature> driveTemperature = driveMotor.getDeviceTemp(false);

            StatusSignal<AngularVelocity> steerVelocity = steerMotor.getVelocity(false);
            StatusSignal<Current> steerStatorCurrent = steerMotor.getStatorCurrent(false);
            StatusSignal<Current> steerSupplyCurrent = steerMotor.getSupplyCurrent(false);
            StatusSignal<Voltage> steerAppliedVoltage = steerMotor.getMotorVoltage(false);
            StatusSignal<Temperature> steerTemperature = steerMotor.getDeviceTemp(false);

            BaseStatusSignal.setUpdateFrequencyForAll(
                250.0, 
                driveStatorCurrent,
                driveSupplyCurrent,
                driveAppliedVoltage,
                driveTemperature,
                steerVelocity,
                steerStatorCurrent,
                steerSupplyCurrent,
                steerAppliedVoltage,
                steerTemperature
            );

            PhoenixUtility.registerSignals(
                CANBusLane.S0, 
                driveStatorCurrent,
                driveSupplyCurrent,
                driveAppliedVoltage,
                driveTemperature,
                steerVelocity,
                steerStatorCurrent,
                steerSupplyCurrent,
                steerAppliedVoltage,
                steerTemperature
            );

            moduleInputUpdaters.put(moduleIndex, (moduleInputs, drivetrainInputs) -> {
                moduleInputs.driveConnected = BaseStatusSignal.isAllGood(driveSupplyCurrent, driveStatorCurrent, driveAppliedVoltage, driveTemperature);
                moduleInputs.driveSupplyCurrentAmperes = driveSupplyCurrent.getValueAsDouble();
                moduleInputs.driveStatorCurrentAmperes = driveStatorCurrent.getValueAsDouble();
                moduleInputs.driveAppliedVoltage = driveAppliedVoltage.getValueAsDouble();
                moduleInputs.driveTemperatureCelsius = driveTemperature.getValueAsDouble();

                moduleInputs.steerConnected = BaseStatusSignal.isAllGood(steerVelocity, steerSupplyCurrent, steerStatorCurrent, steerAppliedVoltage, steerTemperature);
                moduleInputs.steerPosition = drivetrainInputs.ModuleVelocities[moduleIndex].angle;
                moduleInputs.steerVelocityRadiansPerSecond = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
                moduleInputs.steerSupplyCurrentAmperes = steerSupplyCurrent.getValueAsDouble();
                moduleInputs.steerStatorCurrentAmperes = steerStatorCurrent.getValueAsDouble();
                moduleInputs.steerAppliedVoltage = steerAppliedVoltage.getValueAsDouble();
                moduleInputs.steerTemperatureCelsius = steerTemperature.getValueAsDouble();
            });
        }

        pitch = getPigeon2().getPitch();
        roll = getPigeon2().getRoll();

        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        angularRollVelocity = getPigeon2().getAngularVelocityXWorld();

        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();
        accelerationZ = getPigeon2().getAccelerationZ();

        BaseStatusSignal.setUpdateFrequencyForAll(250.0, pitch, roll, angularYawVelocity, angularPitchVelocity, angularRollVelocity, accelerationX, accelerationY, accelerationZ);
        PhoenixUtility.registerSignals(CANBusLane.S0, pitch, roll, angularYawVelocity, angularPitchVelocity, angularRollVelocity, accelerationX, accelerationY, accelerationZ);
    }

    @Override
    public void updateInputs(DrivetrainIOInputs drivetrainInputs, ModuleIOInputs... moduleInputs) {
        drivetrainInputs.fromSwerveDriveState(this.getState());

        drivetrainInputs.yawVelocityRadiansPerSecond = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());
        drivetrainInputs.yawAccelerationRadiansPerSecond2 = accelerationZ.getValueAsDouble() * 9.8067;

        for (int i = 0; i < moduleNames.length; i++) {
            moduleInputUpdaters.get(i).update(moduleInputs[i], drivetrainInputs);
        }

        ChassisVelocities measuredRobotRelativeChassisSpeeds = getKinematics().toChassisVelocities(drivetrainInputs.ModuleVelocities);
        ChassisVelocities measuredFieldRelativeChassisSpeeds = measuredRobotRelativeChassisSpeeds.toFieldRelative(drivetrainInputs.Pose.getRotation());
        ChassisVelocities desiredRobotRelativeChassisSpeeds = getKinematics().toChassisVelocities(drivetrainInputs.ModuleTargets);
        ChassisVelocities desiredFieldRelativeChassisSpeeds = desiredRobotRelativeChassisSpeeds.toFieldRelative(drivetrainInputs.Pose.getRotation());
 
        ChassisVelocities fusedFieldRelativeChassisSpeeds = new ChassisVelocities(
            measuredFieldRelativeChassisSpeeds.vx,
            measuredFieldRelativeChassisSpeeds.vy,
            drivetrainInputs.yawVelocityRadiansPerSecond
        );

        robotState.addDriveMotionMeasurements(
            Timer.getMonotonicTimestamp(),
            drivetrainInputs.yawVelocityRadiansPerSecond,
            Units.degreesToRadians(angularPitchVelocity.getValueAsDouble()),
            Units.degreesToRadians(angularRollVelocity.getValueAsDouble()),
            Units.degreesToRadians(pitch.getValueAsDouble()),
            Units.degreesToRadians(roll.getValueAsDouble()),
            accelerationX.getValueAsDouble(),
            accelerationY.getValueAsDouble(),
            desiredRobotRelativeChassisSpeeds,
            desiredFieldRelativeChassisSpeeds,
            measuredRobotRelativeChassisSpeeds,
            measuredFieldRelativeChassisSpeeds,
            fusedFieldRelativeChassisSpeeds
        );
    }

    @Override
    public void logModules(SwerveDriveState driveState) {
        if (driveState.ModuleVelocities == null) {
            return;
        }

        if (outputNames == null) {
            outputNames = new String[4][5];
            for (int i = 0; i < getModules().length; i++) {
                outputNames[i] = new String[5];
                outputNames[i][0] = moduleNames[i] + " /Absolute Encoder Angle";
                outputNames[i][1] = moduleNames[i] + " /Steering Angle";
                outputNames[i][2] = moduleNames[i] + " /Target Steering Angle";
                outputNames[i][3] = moduleNames[i] + " /Drive Velocity";
                outputNames[i][4] = moduleNames[i] + " /Target Drive Velocity";
            }
        }
        for (int i = 0; i < getModules().length; i++) {
            Logger.recordOutput(outputNames[i][0], getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
            Logger.recordOutput(outputNames[i][1], driveState.ModuleVelocities[i].angle);
            Logger.recordOutput(outputNames[i][2], driveState.ModuleTargets[i].angle);
            Logger.recordOutput(outputNames[i][3], driveState.ModuleVelocities[i].velocity);
            Logger.recordOutput(outputNames[i][4], driveState.ModuleTargets[i].velocity);
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void setSwerveRequest(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void resetRotation() {
        this.resetRotation(robotState.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
    }

    @Override
    public void resetToParameterizedRotation(Rotation2d rotation2d) {
        this.resetRotation(rotation2d);
    }

    @Override
    public void addVisionMeasurement(FieldPoseEstimation fieldPoseEstimation) {
        this.addVisionMeasurement(fieldPoseEstimation.getVisionRobotPoseMeters(), fieldPoseEstimation.getTimestampSeconds(), fieldPoseEstimation.getVisionMeasurementStdDevs());
    }
}
