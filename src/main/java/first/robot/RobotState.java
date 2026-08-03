package first.robot;

import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import first.minolib.math.ConcurrentTimeInterpolatableBuffer;
import first.minolib.math.MathUtility;
import first.minolib.vision.FieldPoseEstimation;
import first.Constants;

public class RobotState {
    private final Consumer<FieldPoseEstimation> visionEstimateConsumer;

    public RobotState(final Consumer<FieldPoseEstimation> visionEsimateConsumer) {
        this.visionEstimateConsumer = visionEsimateConsumer;
        fieldToRobot.addSample(0.0, Pose2d.kZero);
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot = ConcurrentTimeInterpolatableBuffer.createBuffer(Constants.kLoopBackTimeSeconds);

    private double visionConfidence = 0.0;
    private double lastUsedVisionEstimateTimestamp = 0.0;
    private Pose2d lastUsedVisionPoseEstimate = Pose2d.kZero;

    private ChassisVelocities measuredRobotRelativeChassisSpeeds = new ChassisVelocities();
    private ChassisVelocities measuredFieldRelativeChassisSpeeds = new ChassisVelocities();
    private ChassisVelocities desiredRobotRelativeChassisSpeeds = new ChassisVelocities();
    private ChassisVelocities desiredFieldRelativeChassisSpeeds = new ChassisVelocities();
    private ChassisVelocities fusedFieldRelativeChassisSpeeds = new ChassisVelocities();

    private int iteration = 0;

    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularPosition = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularPosition = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);

    private final ConcurrentTimeInterpolatableBuffer<Double> driveAccelerationX = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveAccelerationY = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);

    private boolean enablePathCancel = false;

    private double autoStartTime;

    private Optional<Pose2d> trajectoryTargetPose = Optional.empty();
    private Optional<Pose2d> trajectoryCurrentPose = Optional.empty();

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel = true;
    }

    public void disablePathCancel() {
        enablePathCancel = false;
    }

    public boolean getPathCancel() {
        return enablePathCancel;
    }

    public void incrementIterationCount() {
        iteration++;
    }

    public int getIteration() {
        return iteration;
    }

    public IntSupplier getIterationSupplier() {
        return () -> getIteration();
    }

    public void setTrajectoryTargetPose(Pose2d pose) {
        trajectoryTargetPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryTargetPose() {
        return trajectoryTargetPose;
    }

    public void setTrajectoryCurrentPose(Pose2d pose) {
        trajectoryCurrentPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryCurrentPose() {
        return trajectoryCurrentPose;
    }

    public double getDrivePitchRadians() {
        if (this.drivePitchAngularPosition.getInternalBuffer().lastEntry() != null) {
            return drivePitchAngularPosition.getInternalBuffer().lastEntry().getValue();
        }

        return 0.0;
    }

    public double getDriveRollRadians() {
        if (this.driveRollAngularPosition.getInternalBuffer().lastEntry() != null) {
            return driveRollAngularPosition.getInternalBuffer().lastEntry().getValue();
        }

        return 0.0;
    }

    public void addDriveMotionMeasurements(
        double timestamp, 
        double yawAngularVelocity,
        double pitchAngularVelocity,
        double rollAngularVelocity,
        double pitchAngularPosition,
        double rollAngularPostiion,
        double accelerationX,
        double accelerationY,
        ChassisVelocities measuredRobotRelativeChassisSpeeds,
        ChassisVelocities measuredFieldRelativeChassisSpeeds,
        ChassisVelocities desiredRobotRelativeChassisSpeeds,
        ChassisVelocities desiredFieldRelativeChassisSpeeds,
        ChassisVelocities fusedFieldRelativeChasssisSpeeds
    ) {
        driveYawAngularVelocity.addSample(timestamp, yawAngularVelocity);
        drivePitchAngularVelocity.addSample(timestamp, pitchAngularVelocity);
        driveRollAngularVelocity.addSample(timestamp, rollAngularVelocity);
        drivePitchAngularPosition.addSample(timestamp, pitchAngularPosition);
        driveRollAngularPosition.addSample(timestamp, rollAngularPostiion);

        driveAccelerationX.addSample(timestamp, accelerationX);
        driveAccelerationY.addSample(timestamp, accelerationY);

        this.measuredRobotRelativeChassisSpeeds = measuredRobotRelativeChassisSpeeds;
        this.measuredFieldRelativeChassisSpeeds = measuredFieldRelativeChassisSpeeds;
        this.desiredRobotRelativeChassisSpeeds = desiredRobotRelativeChassisSpeeds;
        this.desiredFieldRelativeChassisSpeeds = desiredFieldRelativeChassisSpeeds;
        this.fusedFieldRelativeChassisSpeeds = fusedFieldRelativeChasssisSpeeds;
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTimesSeconds) {
        var potentiallyFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = potentiallyFieldToRobot == null ? Pose2d.kZero : potentiallyFieldToRobot.getValue();

        var dt = getLatestMeasuredRobotRelativeChassisSpeeds();
        dt = dt.times(lookaheadTimesSeconds);

        return fieldToRobot.plus(new Twist2d(dt.vx, dt.vy, dt.omega).exp());
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public ChassisVelocities getLatestMeasuredRobotRelativeChassisSpeeds() {
        return measuredRobotRelativeChassisSpeeds;
    }

    public ChassisVelocities getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds;
    }

    public ChassisVelocities getLatestDesiredRobotRelativeChassisSpeeds() {
        return desiredRobotRelativeChassisSpeeds;
    }

    public ChassisVelocities getLatestDesiredFieldRelativeChassisSpeeds() {
        return desiredFieldRelativeChassisSpeeds;
    }

    public ChassisVelocities getLatestFusedFieldRelativeChassisSpeeds() {
        return fusedFieldRelativeChassisSpeeds;
    }

    public ChassisVelocities getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestMeasuredRobotRelativeChassisSpeeds();
        speeds.omega = getLatestFusedFieldRelativeChassisSpeeds().omega;

        return speeds;
    }

    private Optional<Double> getMaxAbsoluteValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);

        if(max.isEmpty() || min.isEmpty()) {
            return Optional.empty();
        }

        if(Math.abs(max.get()) >= Math.abs(min.get())) return max;
        else return min;
    }

    public Optional<Double> getMaximumAbsoluteDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        if(Robot.isReal()) {
            return getMaxAbsoluteValueInRange(driveYawAngularVelocity, minTime, maxTime);
        }

        return Optional.of(measuredRobotRelativeChassisSpeeds.omega);
    }

    public Optional<Double> getMaximumAbsoluteDrivePitchAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsoluteValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaximumAbsoluteDriveRollAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsoluteValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updateVisionPoseEstimate(FieldPoseEstimation fieldPoseEstimate) {
        lastUsedVisionEstimateTimestamp = fieldPoseEstimate.getTimestampSeconds();
        lastUsedVisionPoseEstimate = fieldPoseEstimate.getVisionRobotPoseMeters();
        visionEstimateConsumer.accept(fieldPoseEstimate);
    }

    public double lastUsedVisionTimestamp() {
        return lastUsedVisionEstimateTimestamp;
    }

    public Pose2d lastUsedVisionPose() {
        return lastUsedVisionPoseEstimate;
    }

    public void setVisionConfidence(double confidence) {
        visionConfidence = MathUtility.clamp(confidence, 0.0, 1.0);
    }

    public double getVisionConfidence() {
        return visionConfidence;
    }

    public boolean isRedAlliance() {
        return MatchState.getAlliance().isPresent() && MatchState.getAlliance().equals(Optional.of(Alliance.RED));
    }

    public void updateLogger() {
        if (this.driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/YawAngularVelocity", this.driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/RollAngularVelocity", this.driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/PitchAngularVelocity", this.drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularPosition.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/PitchPositionRadians", this.drivePitchAngularPosition.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularPosition.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/RollPositionRadians", this.driveRollAngularPosition.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveAccelerationX.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelerationX", this.driveAccelerationX.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveAccelerationY.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelerationY", this.driveAccelerationY.getInternalBuffer().lastEntry().getValue());
        }
        Logger.recordOutput("RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/DesiredChassisSpeedRobotFrame", getLatestDesiredRobotRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/MeasuredChassisSpeedRobotFrame", getLatestMeasuredRobotRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeeds());
    }
}