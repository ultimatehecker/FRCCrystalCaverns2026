package first.robot.oi;

import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsXboxController;

import first.robot.Robot;
import first.robot.constants.ControllerConstants;

public class DriverControllerXbox implements DriverControllerIO {
    private final CommandNiDsXboxController controller;

    public DriverControllerXbox() {
        if (Robot.isSimulation()) {
            controller = new CommandNiDsXboxController(ControllerConstants.kDriverControllerPort); //TODO: Change to simulated controller at a later date
        } else {
            controller = new CommandNiDsXboxController(ControllerConstants.kDriverControllerPort);
        }
    }

    @Override
    public double getThrottle() {
        return -(Math.pow(Math.abs(controller.getLeftY()), 1.5)) * Math.signum(controller.getLeftY());
    }

    @Override
    public double getStrafe() {
        return -(Math.pow(Math.abs(controller.getLeftX()), 1.5)) * Math.signum(controller.getLeftX());
    }

    @Override
    public double getRotation() {
        return -(Math.pow(Math.abs(controller.getRightX()), 2.0)) * Math.signum(controller.getRightX());
    }

    @Override
    public double getRotationY() {
        return -(Math.pow(Math.abs(controller.getRightY()), 2.0)) * Math.signum(controller.getRightY());
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }   
}