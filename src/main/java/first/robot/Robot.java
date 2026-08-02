// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      Scheduler.getDefault().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      Scheduler.getDefault().cancel(autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void utilityInit() {
    Scheduler.getDefault().cancelAll();
  }

  @Override
  public void utilityPeriodic() {}
}
