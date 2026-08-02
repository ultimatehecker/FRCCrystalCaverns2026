package first.robot.oi;

import org.wpilib.command3.Trigger;

public interface DriverControllerIO {
    public double getThrottle();

    public double getStrafe();

    public double getRotation();

    public double getRotationY();

    public Trigger resetGyro();
}