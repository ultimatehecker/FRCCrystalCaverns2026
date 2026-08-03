package first.robot.subsystems.drivetrain;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import first.Constants;
import first.robot.RobotState;
import first.robot.simulation.SimulatedRobotState;

public class DrivetrainIOSimulation extends DrivetrainIOHardware {
    private SimulatedRobotState simulatedRobotState = null;

    private Notifier simNotifier = null;

    private double lastSimTime;

    Consumer<SwerveDriveState> simTelemetryConsumer = swerveDriveState -> {
        if (simulatedRobotState == null) {
            return;
        }

        simulatedRobotState.addFieldToRobot(swerveDriveState.Pose);
    };

    public DrivetrainIOSimulation(RobotState robotState, SimulatedRobotState simulatedRobotState, SwerveDrivetrainConstants constants, @SuppressWarnings("unchecked") SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants) {
        super(robotState, constants, moduleConstants);

        this.simulatedRobotState = simulatedRobotState;

        registerTelemetry(simTelemetryConsumer);
        startSimThread();
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs, ModuleIOInputs... moduleInputs) {
        super.updateInputs(inputs, moduleInputs);

        Pose2d pose = simulatedRobotState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drivetrain/Viz/SimPose", simulatedRobotState.getLatestFieldToRobot());
        }
    }

    public void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        simNotifier.startPeriodic(Constants.kSimLoopPeriodSeconds);
    }
}
