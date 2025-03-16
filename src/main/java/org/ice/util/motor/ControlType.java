package org.ice.util.motor;

import com.ctre.phoenix6.controls.*;
import com.revrobotics.spark.SparkBase;

import java.util.function.DoubleFunction;

/**
 * Enum of the different possible ways of controlling {@link GenericMotorController} instances.
 */
public enum ControlType {
    /**
     * Moves the motor using PID to the given position in rotations
     * @see #MM_POSITION
     */
    POSITION(SparkBase.ControlType.kPosition,PositionDutyCycle::new),
    /**
     * Sets the motor's power as a percent represented by a value from -1 to 1
     */
    DUTY_CYCLE(SparkBase.ControlType.kDutyCycle,DutyCycleOut::new),
    /**
     * Sets the motor's velocity using PID to the given velocity in rpm
     * @see #MM_VELOCITY
     */
    VELOCITY(SparkBase.ControlType.kVelocity,VelocityDutyCycle::new),
    /**
     * Sets the motor's velocity using Motion Magic/MAX Motion to the given velocity in rpm
     * @see #VELOCITY
     */
    MM_VELOCITY(SparkBase.ControlType.kMAXMotionVelocityControl, MotionMagicVelocityDutyCycle::new),
    /**
     * Moves the motor using Motion Magic/MAX Motion to the given position in rotations
     * @see #POSITION
     */
    MM_POSITION(SparkBase.ControlType.kMAXMotionPositionControl, MotionMagicDutyCycle::new),
    /**
     * Sets the voltage that the motor operates at in Volts
     */
    VOLTAGE(SparkBase.ControlType.kVoltage,VoltageOut::new);

    private SparkBase.ControlType sparkControl;
    private DoubleFunction<ControlRequest> talonControl;

    ControlType(SparkBase.ControlType sparkControl, DoubleFunction<ControlRequest> talonControl) {
        this.sparkControl = sparkControl;
        this.talonControl = talonControl;
    }
    SparkBase.ControlType asSparkControl() {
        return sparkControl;
    }

    ControlRequest asTalonControl(double input) {
        return talonControl.apply(input);
    }
}
