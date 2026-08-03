package first.robot.simulation;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Timer;

import first.robot.RobotContainer;

public class SimulatedRobotState {
    private final RobotContainer robotContainer;
    private double lastTimestamp = 0.0;

    private Pose2d fieldToRobotSimulatedTruth = Pose2d.kZero;

    public SimulatedRobotState(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public synchronized void addFieldToRobot(Pose2d pose) {
        updateRobotPoseIfNewer(Timer.getMonotonicTimestamp(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        return fieldToRobotSimulatedTruth;
    }

    public synchronized void updateSim() {
        lastTimestamp = Timer.getMonotonicTimestamp();
    }

    private void updateRobotPoseIfNewer(double timestamp, Pose2d pose) {
        if (timestamp > lastTimestamp) {
            lastTimestamp = timestamp;
            fieldToRobotSimulatedTruth = pose;
        }
    }
}