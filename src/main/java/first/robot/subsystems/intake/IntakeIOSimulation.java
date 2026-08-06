package first.robot.subsystems.intake;

import static org.wpilib.units.Units.KilogramSquareMeters;

import org.wpilib.math.jni.LinearSystemUtilJNI;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystemUtil;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.MathUtil;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.LinearSystemSim;

import first.Constants;
import first.minolib.math.MathUtility;
import first.robot.constants.IntakeConstants;

public class IntakeIOSimulation implements IntakeIO {
    private final DCMotor gearbox;
    private final DCMotorSim simulation;

    private double appliedVoltage = 0.0;

    public IntakeIOSimulation() {
        gearbox = IntakeConstants.kSimulatedGearbox;
        simulation = new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                gearbox,
                IntakeConstants.kMOI,
                IntakeConstants.kMotorReduction
            ),
            gearbox
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.positionRadians = simulation.getAngularPosition();
        inputs.velocityRadiansPerSecond = simulation.getAngularVelocity();
        inputs.appliedVoltage = appliedVoltage;
        inputs.statorCurrentAmperes = gearbox.getCurrent(simulation.getTorque());
        inputs.supplyCurrentAmperes = simulation.getCurrentDraw();
        inputs.temperatureCelsius = 0.0;
        inputs.temperatureFault = false;

        simulation.update(Constants.kSimLoopPeriodSeconds);
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = MathUtility.clamp(voltage, -12.0, 12.0);
        simulation.setInputVoltage(appliedVoltage);
    }

    @Override
    public void setVelocity(double velocity) {
        
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }
}