package first.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

import static first.minolib.phoenix.PhoenixUtility.tryUntilOk;

import first.minolib.hardware.MinoCANBus.CANBusLane;
import first.minolib.phoenix.PhoenixUtility;
import first.robot.constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    private final TalonFX motor;
    private final TalonFXConfiguration configuration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Temperature> temperature;

    private final VoltageOut voltageRequest = new VoltageOut(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0)
        .withUpdateFreqHz(0.0);

    private final NeutralOut neutralRequest = new NeutralOut()
        .withUpdateFreqHz(0.0);
    
    public IntakeIOHardware() {
        motor = new TalonFX(IntakeConstants.kMotor.getDeviceID(), IntakeConstants.kMotor.getCANBus());
        configuration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(IntakeConstants.kMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            ).withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(IntakeConstants.kMotorStatorLimit)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IntakeConstants.kMotorSupplyLimit)
            ).withSlot0(new Slot0Configs()
                .withKP(IntakeConstants.kP)
                .withKI(IntakeConstants.kI)
                .withKD(IntakeConstants.kD)
                .withKS(IntakeConstants.kS)
                .withKV(IntakeConstants.kV)
                .withKA(IntakeConstants.kA)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IntakeConstants.kMotorReduction)
                    .withVelocityFilterTimeConstant(0.1)
            );

        tryUntilOk(5, () -> motor.getConfigurator().apply(configuration, 0.25));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        acceleration = motor.getAcceleration();
        statorCurrent = motor.getStatorCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        appliedVoltage = motor.getMotorVoltage();
        temperature = motor.getDeviceTemp();

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(250.0, position, velocity, acceleration, statorCurrent, supplyCurrent, appliedVoltage, temperature));
        tryUntilOk(5, () -> motor.optimizeBusUtilization(0.0, 0.25));

        PhoenixUtility.registerSignals(CANBusLane.S1, position, velocity, acceleration, statorCurrent, supplyCurrent, appliedVoltage, temperature);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.positionRadians = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadiansPerSecond = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.accelerationRadiansPerSecond2 = Units.rotationsToRadians(acceleration.getValueAsDouble());
        inputs.statorCurrentAmperes = statorCurrent.getValueAsDouble();
        inputs.supplyCurrentAmperes = supplyCurrent.getValueAsDouble();
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.temperatureCelsius = temperature.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setVelocity(double setpoint) {
        motor.setControl(velocityRequest.withVelocity(setpoint));
    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(() -> {
            configuration.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> motor.getConfigurator().apply(configuration, 0.0));
        });
    }
}
