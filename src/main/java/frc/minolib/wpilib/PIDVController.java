// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.minolib.wpilib;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Implements a PID control loop. */
public class PIDVController implements Sendable, AutoCloseable {
    private static int instances;

    // Factor for "proportional" control
    private double kP;

    // Factor for "integral" control
    private double kI;

    // Factor for "derivative" control
    private double kD;

    // The error range where "integral" control applies
    private double iZone = Double.POSITIVE_INFINITY;

    // The period (in seconds) of the loop that calls the controller
    private final double period;

    private double maximumIntegral = 1.0;
    private double minimumIntegral = -1.0;

    private double maximumInput;
    private double minimumInput;

    // Do the endpoints wrap around? e.g. Absolute encoder
    private boolean continuous;

    // The error at the time of the most recent call to calculate()
    private double positionError;
    private double velocityError;

    // The sum of the errors for use in the integral calc
    private double totalError;

    // The error that is considered at setpoint.
    private double positionTolerance = 0.05;
    private double velocityTolerance = Double.POSITIVE_INFINITY;

    private double setpoint;
    private double setpointVelocity;
    private double measurement;
    private double measurementVelocity;;

    private boolean haveMeasurement;
    private boolean haveSetpoint;

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a
     * default period of
     * 0.02 seconds.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public PIDVController(double kp, double ki, double kd) {
        this(kp, ki, kd, 0.02);
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     *
     * @param kp     The proportional coefficient.
     * @param ki     The integral coefficient.
     * @param kd     The derivative coefficient.
     * @param period The period between controller updates in seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    @SuppressWarnings("this-escape")
    public PIDVController(double kp, double ki, double kd, double period) {
        kP = kp;
        kI = ki;
        kD = kd;

        if (kp < 0.0) {
            throw new IllegalArgumentException("Kp must be a non-negative number!");
        }
        if (ki < 0.0) {
            throw new IllegalArgumentException("Ki must be a non-negative number!");
        }
        if (kd < 0.0) {
            throw new IllegalArgumentException("Kd must be a non-negative number!");
        }
        if (period <= 0.0) {
            throw new IllegalArgumentException("Controller period must be a positive number!");
        }

        this.period = period;

        instances++;
        SendableRegistry.addLW(this, "PIDController", instances);

        MathSharedStore.reportUsage(MathUsageId.kController_PIDController2, instances);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>
     * Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp The proportional coefficient. Must be &gt;= 0.
     */
    public void setP(double kp) {
        kP = kp;
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki The integral coefficient. Must be &gt;= 0.
     */
    public void setI(double ki) {
        kI = ki;
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd The differential coefficient. Must be &gt;= 0.
     */
    public void setD(double kd) {
        kD = kd;
    }

    /**
     * Sets the IZone range. When the absolute value of the position error is
     * greater than IZone, the
     * total accumulated error will reset to zero, disabling integral gain until the
     * absolute value of
     * the position error is less than IZone. This is used to prevent integral
     * windup. Must be
     * non-negative. Passing a value of zero will effectively disable integral gain.
     * Passing a value
     * of {@link Double#POSITIVE_INFINITY} disables IZone functionality.
     *
     * @param iZone Maximum magnitude of error to allow integral control.
     * @throws IllegalArgumentException if iZone &lt; 0
     */
    public void setIZone(double iZone) {
        if (iZone < 0) {
            throw new IllegalArgumentException("IZone must be a non-negative number!");
        }

        this.iZone = iZone;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return kP;
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return kI;
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return kD;
    }

    /**
     * Get the IZone range.
     *
     * @return Maximum magnitude of error to allow integral control.
     */
    public double getIZone() {
        return iZone;
    }

    /**
     * Returns the period of this controller.
     *
     * @return the period of the controller.
     */
    public double getPeriod() {
        return period;
    }

    /**
     * Returns the position tolerance of this controller.
     *
     * @return the position tolerance of the controller.
     */
    public double getPositionTolerance() {
        return positionTolerance;
    }

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    public double getVelocityTolerance() {
        return velocityTolerance;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint, double setpointVelocity) {
        this.setpoint = setpoint;
        this.setpointVelocity = setpointVelocity;
        haveSetpoint = true;

        if (continuous) {
            double errorBound = (maximumInput - minimumInput) / 2.0;
            positionError = MathUtil.inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            positionError = setpoint - measurement;
        }

        velocityError = setpointVelocity - measurementVelocity;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     * <p>
     * This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return haveMeasurement
                && haveSetpoint
                && Math.abs(positionError) < positionTolerance
                && Math.abs(velocityError) < velocityTolerance;
    }

    /**
     * Enables continuous input.
     *
     * <p>
     * Rather then using the max and min input range as constraints, it considers
     * them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        continuous = false;
    }

    /**
     * Returns true if continuous input is enabled.
     *
     * @return True if continuous input is enabled.
     */
    public boolean isContinuousInputEnabled() {
        return continuous;
    }

    /**
     * Sets the minimum and maximum values for the integrator.
     *
     * <p>
     * When the cap is reached, the integrator value is added to the controller
     * output rather than
     * the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        this.minimumIntegral = minimumIntegral;
        this.maximumIntegral = maximumIntegral;
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return positionError;
    }

    /**
     * Returns the velocity error.
     *
     * @return The velocity error.
     */
    public double getVelocityError() {
        return velocityError;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double measurementVelocity, double setpoint, double setpointVelocity) {
        this.setpoint = setpoint;
        this.setpointVelocity = setpointVelocity;
        haveSetpoint = true;
        return calculate(measurement, measurementVelocity);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement, double measurementVelocity) {
        this.measurement = measurement;
        this.measurementVelocity = measurementVelocity;
        haveMeasurement = true;

        if (continuous) {
            double errorBound = (maximumInput - minimumInput) / 2.0;
            positionError = MathUtil.inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            positionError = setpoint - measurement;
        }

        velocityError = setpointVelocity - measurementVelocity;

        // If the absolute value of the position error is greater than IZone, reset the
        // total error
        if (Math.abs(positionError) > iZone) {
            totalError = 0;
        } else if (kI != 0) {
            totalError = MathUtil.clamp(
                    totalError + positionError * period,
                    minimumIntegral / kI,
                    maximumIntegral / kI);
        }

        return kP * positionError + kI * totalError + kD * velocityError;
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        positionError = 0;
        totalError = 0;
        velocityError = 0;
        haveMeasurement = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty(
                "izone",
                this::getIZone,
                (double toSet) -> {
                    try {
                        setIZone(toSet);
                    } catch (IllegalArgumentException e) {
                        MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace());
                    }
                });
        // builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}