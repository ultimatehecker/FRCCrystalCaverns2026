package first.robot.oi;

import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsPS5Controller;

import first.robot.Robot;
import first.robot.constants.ControllerConstants;

public class DriverControllerPS5 implements DriverControllerIO {
    private final CommandNiDsPS5Controller controller;

    public DriverControllerPS5() {
        if (Robot.isSimulation()) {
            controller = new CommandNiDsPS5Controller(ControllerConstants.kDriverControllerPort); //TODO: Change to simulated controller at a later date
        } else {
            controller = new CommandNiDsPS5Controller(ControllerConstants.kDriverControllerPort);
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
        return controller.touchpad(); //TODO: Fix these buttons
    }

    @Override
    public Trigger toggleDrivingMode() {
        return controller.PS(); //TODO: Fix these buttons
    }
}
