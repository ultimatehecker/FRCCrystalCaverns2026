package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Consumer;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.minolib.swerve.MapleSimulatedSwerveDrivetrain;
import frc.robot.RobotState;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.simulation.SimulatedRobotState;

public class DrivetrainIOSimulation extends DrivetrainIOHardware {

    private SimulatedRobotState simulatedRobotState = null;
    public MapleSimulatedSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    private Notifier simNotifier = null;
    private final SwerveModuleConstants[] moduleConstants;

    private double lastSimTime;

    Consumer<SwerveDriveState> simTelemetryConsumer = swerveDriveState -> {
        if (simulatedRobotState == null) {
            return;
        }

        // Override pose with MapleSim physics if enabled
        if (GlobalConstants.kUseMapleSim && mapleSimSwerveDrivetrain != null) {
            swerveDriveState.Pose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }

        simulatedRobotState.addFieldToRobot(swerveDriveState.Pose);
    };

    public DrivetrainIOSimulation(RobotState robotState, SimulatedRobotState simulatedRobotState, SwerveDrivetrainConstants constants, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants) {
        super(robotState, constants, moduleConstants);

        this.simulatedRobotState = simulatedRobotState;
        this.moduleConstants = moduleConstants;

        registerTelemetry(simTelemetryConsumer);
        startSimThread();
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs, ModuleIOInputs... moduleInputs) {
        super.updateInputs(inputs, moduleInputs);

        var pose = simulatedRobotState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drivetrain/Viz/SimPose", simulatedRobotState.getLatestFieldToRobot());
        }
    }

    @SuppressWarnings("unchecked")
    public void startSimThread() {
        if (GlobalConstants.kUseMapleSim) {
            mapleSimSwerveDrivetrain = new MapleSimulatedSwerveDrivetrain(
                Seconds.of(GlobalConstants.kLoopPeriodSeconds), 
                DrivetrainConstants.kRobotMassKilograms, 
                Inches.of(31), 
                Inches.of(31), 
                DrivetrainConstants.kDriveSimulatedGearbox, 
                DrivetrainConstants.kSteerSimulatedGearbox, 
                1.2, 
                getModuleLocations(), 
                getPigeon2(), 
                getModules(), 
                moduleConstants[0],
                moduleConstants[1],
                moduleConstants[2],
                moduleConstants[3]
            );

            simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        } else {
            lastSimTime = Utils.getCurrentTimeSeconds();
            simNotifier = new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
        }

        simNotifier.startPeriodic(GlobalConstants.kLoopPeriodSeconds);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (GlobalConstants.kUseMapleSim) {
            if (mapleSimSwerveDrivetrain != null) {
                mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
                Timer.delay(0.05);
            }
        }
        
        super.resetPose(pose);
    }

    public MapleSimulatedSwerveDrivetrain getMapleSimDrive() {
        return mapleSimSwerveDrivetrain;
    }

}