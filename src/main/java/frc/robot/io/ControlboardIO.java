package frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlboardIO {
    public double getThrottle();

    public double getStrafe();

    public double getRotation();

    public double getRotationY();

    public Trigger resetGyro();
}
