package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DrivetrainIO {
    @AutoLog
    public class DrivetrainIOInputs extends SwerveDriveState {
        public double yawVelocityRadiansPerSecond = 0.0;
        public double yawAccelerationRadiansPerSecond2 = 0.0;

        public double[] steerRatesRadiansPerSecond = new double[4];

        public void fromSwerveDriveState(SwerveDriveState state) {
            this.Pose = state.Pose;
            this.RawHeading = state.RawHeading;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.Speeds = state.Speeds;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
            this.OdometryPeriod = state.OdometryPeriod;
            this.Timestamp = state.Timestamp;
        }
    }

    @AutoLog
    public class ModuleIOInputs {
        public boolean driveConnected = false;
        public double driveSupplyCurrentAmperes = 0.0;
        public double driveStatorCurrentAmperes = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveTempuratureCelsius = 0.0;

        public boolean steerConnected = false;
        public Rotation2d steerPosition = Rotation2d.kZero;
        public double steerVelocity = 0.0;
        public double steerSupplyCurrentAmperes = 0.0;
        public double steerStatorCurrentAmperes = 0.0;
        public double steerAppliedVoltage = 0.0;
        public double steerTempuratureCelsius = 0.0;
    }

    /** Updates the inputs for the drivetrain */
    public void updateInputs(DrivetrainIOInputs inputs);

    /** Updates the inputs for the drivetrain modules */
    public void updateModuleInputs(ModuleIOInputs... inputs);

    /** Logs the state of the drivetrain modules for debugging */
    public void logModules(SwerveDriveState state);

    /** Applies commanded module states to the drivetrain */
    public default void setSwerveRequest(SwerveRequest request) {}

    /** Resets the drivetrain pose to a specific pose */
    public default void resetPose(SwerveDriveState pose) {}

    /** Resets the drivetrain rotation to Rotation2d.kZero (forward) */
    public default void resetRotation() {}

    /** Resets the drivetrain rotation to a specific rotation */
    public default void resetToParamaterizedRotation(Rotation2d rotation) {}
}
