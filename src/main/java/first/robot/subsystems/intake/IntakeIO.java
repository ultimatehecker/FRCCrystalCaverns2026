package first.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public boolean motorConnected = false;
        public double positionRadians = 0.0;
        public double velocityRadiansPerSecond = 0.0;
        public double accelerationRadiansPerSecond2 = 0.0;
        public double statorCurrentAmperes = 0.0;
        public double supplyCurrentAmperes = 0.0;
        public double appliedVoltage = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setVoltage(double voltage);

    public void setVelocity(double setpoint);

    public void setTorqueCurrent(double setpoint);

    public void stop();
}
