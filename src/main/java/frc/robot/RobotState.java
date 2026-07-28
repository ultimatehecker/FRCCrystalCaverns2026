package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
    private Pose2d fieldToRobot = Pose2d.kZero;
    private double lastTimestamp = Double.MIN_NORMAL;

    private ChassisSpeeds measuredFieldRelativeChassisSpeeds = new ChassisSpeeds();

    public RobotState() {}

    public void addPoseObservation(double timestamp, Pose2d robotPose, ChassisSpeeds robotRelativeChassisSpeeds, ChassisSpeeds fieldRelativeChassisSpeeds, double yaw, double yawRate, double yawAcceleration) {
        updateRobotPoseIfNewer(timestamp, robotPose);
        measuredFieldRelativeChassisSpeeds = fieldRelativeChassisSpeeds;
    }

    private void updateRobotPoseIfNewer(double timestamp, Pose2d pose) {
        if (timestamp > lastTimestamp) {
            lastTimestamp = timestamp;
            fieldToRobot = pose;
        }
    }

    public Pose2d getFieldToRobotPose() {
        return fieldToRobot;
    }

    public ChassisSpeeds getMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }
}
