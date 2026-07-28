package frc.robot.simulation;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class SimulatedRobotState {
    private SwerveDriveSimulation simulatedDrivetrain;
    private final RobotContainer container;
    private double lastTimestamp = 0.0;

    private Pose2d fieldToRobotSimulatedTruth = Pose2d.kZero;

    public SimulatedRobotState(RobotContainer robotContainer) {
        this.container = robotContainer;
    }

    public void init() {
        this.simulatedDrivetrain = this.container.getDrivetrain().getMapleSimDrive().mapleSimDrive;
    }

    public synchronized void addFieldToRobot(Pose2d pose) {
        updateRobotPoseIfNewer(Timer.getFPGATimestamp(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        return fieldToRobotSimulatedTruth;
    }

    public synchronized void updateSim() {
        lastTimestamp = Timer.getFPGATimestamp();
    }

    private void updateRobotPoseIfNewer(double timestamp, Pose2d pose) {
        if (timestamp > lastTimestamp) {
            lastTimestamp = timestamp;
            fieldToRobotSimulatedTruth = pose;
        }
    }
}
