// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import java.util.function.Consumer;

import org.wpilib.command3.Command;
import org.wpilib.driverstation.internal.DriverStationBackend;

import first.minolib.vision.FieldPoseEstimation;
import first.robot.constants.DrivetrainConstants;
import first.robot.oi.DriverController;
import first.robot.oi.DriverControllerXbox;
import first.robot.simulation.SimulatedRobotState;
import first.robot.subsystems.drivetrain.Drivetrain;
import first.robot.subsystems.drivetrain.DrivetrainIOHardware;
import first.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import first.robot.subsystems.drivetrain.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private RobotState robotState;
  private SimulatedRobotState simulatedRobotState;

  private Drivetrain drivetrain;

  private DriverController controller;

  private RobotState buildRobotState() {
    if (Robot.isSimulation()) {
      return new RobotState(visionEstimateConsumer);
    } else {
      return new RobotState(visionEstimateConsumer);
    }
  }

  private SimulatedRobotState buildSimulatedRobotState() {
    if (Robot.isSimulation()) {
      return new SimulatedRobotState(this);
    } else return null;
  }

  @SuppressWarnings("unchecked")
  private Drivetrain buildDrivetrain() {
    if (Robot.isSimulation()) {
      return new Drivetrain(robotState, new DrivetrainIOSimulation(robotState, simulatedRobotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), TunerConstants.kFrontLeft, TunerConstants.kFrontRight, TunerConstants.kBackLeft, TunerConstants.kBackRight));
    } else {
      return new Drivetrain(robotState, new DrivetrainIOHardware(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), TunerConstants.kFrontLeft, TunerConstants.kFrontRight, TunerConstants.kBackLeft, TunerConstants.kBackRight));
    }
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public SimulatedRobotState getSimulatedRobotState() {
    return simulatedRobotState;
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public DriverController buildDriverController() {
    return new DriverController(new DriverControllerXbox());
  }

  public RobotContainer() {
    robotState = buildRobotState();
    simulatedRobotState = buildSimulatedRobotState();

    drivetrain = buildDrivetrain();

    controller = buildDriverController();
    
    DriverStationBackend.silenceJoystickConnectionWarning(Robot.isSimulation());
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(drivetrain.drive(controller::getThrottle, controller::getStrafe, controller::getRotation, controller::toggleDrivingMode));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  private final Consumer<FieldPoseEstimation> visionEstimateConsumer = new Consumer<FieldPoseEstimation>() {
    @Override
    public void accept(FieldPoseEstimation estimate) {
        drivetrain.addVisionMeasurement(estimate);
    }
  };
}
