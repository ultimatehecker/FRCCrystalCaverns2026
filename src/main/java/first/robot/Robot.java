// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

import com.ctre.phoenix6.SignalLogger;

import first.BuildConstants;
import first.Constants;
import first.minolib.advantagekit.LoggedTracer;
import first.minolib.phoenix.PhoenixUtility;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("GitDirty", switch(BuildConstants.DIRTY) {
        case 0 -> "All changes committed";
        case 1 -> "Uncommitted changes";
        default -> "Unknown";
    });

    switch(Constants.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false);
        String path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    SignalLogger.setPath("/media/sda1/");
    SignalLogger.enableAutoLogging(false);
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();

    PhoenixUtility.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    Scheduler.getDefault().run();
    LoggedTracer.record("Scheduler");
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