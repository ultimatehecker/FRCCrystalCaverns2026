// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIOHardware;
import frc.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import frc.robot.subsystems.drivetrain.SimulationTunerConstants;

public class RobotContainer {
  private Drivetrain drivetrain;
  private RobotState robotState;
  private SimulatedRobotState simulatedRobotState;

  @SuppressWarnings("unchecked")
  public Drivetrain buildDrivetrain() {
    if (Robot.isSimulation()) {
      return new Drivetrain(robotState, new DrivetrainIOSimulation(robotState, simulatedRobotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), SimulationTunerConstants.kFrontLeft, SimulationTunerConstants.kFrontRight, SimulationTunerConstants.kBackLeft, SimulationTunerConstants.kBackRight));
    } else {
      return new Drivetrain(robotState, new DrivetrainIOHardware(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), CompetitionTunerConstants.kFrontLeft, CompetitionTunerConstants.kFrontRight, CompetitionTunerConstants.kBackLeft, CompetitionTunerConstants.kBackRight));
    }
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public RobotContainer() {
    robotState = new RobotState();

    if (Robot.isSimulation()) {
      simulatedRobotState = new SimulatedRobotState(this);
    }

    drivetrain = buildDrivetrain();

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
