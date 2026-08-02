package first.robot.oi;

import org.wpilib.command3.Trigger;

public class DriverController {
    private final DriverControllerIO io;

    public DriverController(DriverControllerIO io) {
        this.io = io;
    }

    public double getThrottle() {
        return io.getThrottle();
    }

    public double getStrafe() {
        return io.getStrafe();
    }

    public double getRotation() {
        return io.getRotation();
    }

    public double getRotationY() {
        return io.getRotationY();
    }

    public Trigger resetGyro() {
        return io.resetGyro();
    }

    public boolean toggleDrivingMode() {
        return io.toggleDrivingMode().getAsBoolean();
    }
}
