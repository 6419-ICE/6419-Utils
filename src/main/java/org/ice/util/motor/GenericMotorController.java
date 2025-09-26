package org.ice.util.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import org.ice.util.sendable.AnnotatedSendable;
import org.ice.util.swerve.PIDValues;

/**
 * Generic top-level interface that defines multiple methods for controlling and reading motors.
 * Instances can be directly defined using the {@link GenericSpark} and {@link GenericTalon} subclasses, or using the {@link #from(TalonFX)}, or {@link #from(SparkBase, SparkBaseConfig)} methods.
 * @param <T> The motor represented by the controller
 */
public interface GenericMotorController<T> extends AnnotatedSendable {

    /**
     * Gets the underlying motor controller used to control the motor.
     * @return The underlying motor controller used to control the motor.
     */
    T getMotor();

    /**
     * Sets the motor to use the given {@link ControlType} with the given input without applying the controller's {@link #getPositionConversionFactor() conversion factor}
     * @param input the input.
     * @param type the control mode for the motor
     */
    void controlRaw(double input, ControlType type);

    /**
     * Sets the motor to use the given {@link ControlType} with the given input, applying the controller's {@link #getPositionConversionFactor() conversion factor} for affected control types
     * @param input the input for the given control type
     * @param type the control type
     */
    default void control(double input, ControlType type) {
        switch(type) {
            case DUTY_CYCLE, VOLTAGE -> controlRaw(input,type);
            case POSITION, MM_POSITION -> controlRaw(input*getPositionConversionFactor(),type);
            case VELOCITY, MM_VELOCITY -> controlRaw(input*getVelocityConversionFactor(),type);
        }
    }

    /**
     * Sets the motor's power as a range from -1 to 1. This is equivalent to using the {@link ControlType#DUTY_CYCLE DUTY_CYCLE control type}
     * @param power the percent power the motor should be set to (-1 to 1)
     * @see #get()
     * @see #control(double, ControlType)
     */
    default void set(double power) {
        controlRaw(power,ControlType.DUTY_CYCLE);
    }

    /**
     * Gets the current percent power of the motor as a double ranging from -1 to 1.
     * @return the motor's current power as a double ranging from -1 to 1
     * @see #set(double)
     */
    double get();

    /**
     * The motor's current temperature in Celsius.
     */
    double getTemp();

    /**
     * The output current of the motor in Amps.
     */
    double getOutputCurrent();

    /**
     * The position reading of the motor's encoder without the {@link #getPositionConversionFactor() conversion factor} applied.
     * @see #getPosition()
     */
    double getRawPosition();


    /**
     * The velocity reading of the motor's encoder without the {@link #getPositionConversionFactor() conversion factor} applied.
     * @see #getVelocity()
     */
    double getRawVelocity();

    /**
     * The position conversion factor of the motor. This is applied when reading and setting the motor's position.
     * @return The motor's position conversion factor.
     * @see #setPositionConversionFactor(double)
     */
    double getPositionConversionFactor();

    /**
     * Sets the motor's {@link #getPositionConversionFactor() position conversion factor} to the given value.
     * @param factor the new conversion factor.
     * @see #setPositionConversionFactor(GearRatio)
     * @see #getPositionConversionFactor()
     */
    void setPositionConversionFactor(double factor);

    /**
     * Sets the motor's {@link #getPositionConversionFactor()}  position conversion factor} to the given gear ratio. This is equivalent to {@code setPositionConversionFactor(ratio.getConversionFactor())}
     * @param ratio the gear ratio to use.
     * @see #setPositionConversionFactor(double)
     * @see #setVelocityConversionFactor(GearRatio)
     */
    default void setPositionConversionFactor(GearRatio ratio) {
        setPositionConversionFactor(ratio.getConversionFactor());
    }

    /**
     * The velocity conversion factor of the motor. This is applied when reading and setting the motor's velocity.
     * @return The motor's velocity conversion factor.
     * @see #setVelocityConversionFactor(double)
     */
    double getVelocityConversionFactor();

    /**
     * Sets the motor's {@link #getVelocityConversionFactor() velocity conversion factor} to the given value.
     * @param factor the new conversion factor.
     * @see #setVelocityConversionFactor(GearRatio)
     * @see #setPositionConversionFactor(double)
     */
    void setVelocityConversionFactor(double factor);
    /**
     * Sets the motor's {@link #getVelocityConversionFactor() velocity conversion factor} to the given gear ratio. This is equivalent to {@code setVelocityConversionFactor(ratio.getConversionFactor())}
     * @param ratio the gear ratio to use.
     * @see #setVelocityConversionFactor(double)
     * @see #setPositionConversionFactor(GearRatio)
     */
    default void setVelocityConversionFactor(GearRatio ratio) {
        setVelocityConversionFactor(ratio.getConversionFactor());
    }
    /**
     * The motor's CAN bus ID
     */
    int getMotorID();

    /**
     * Configures the motor to use the given PID values.
     * @param pid the PID values to configure the motor with
     * @see #linkToPIDValues(PIDValues)
     */
    void setPID(PIDValues pid);

    /**
     * Links this motor to the given {@link PIDValues} object, causing this motor's configured PID
     * to automatically update whenever the linked pid values are changed
     * @param pidValues the PIDValues instance to link this motor to
     * @see PIDValues#linkToMotor(GenericMotorController)
     */
    default void linkToPIDValues(PIDValues pidValues) {
        pidValues.linkToMotor(this);
    }

    /**
     * Sets this motor to {@link #follow(GenericMotorController, boolean) follow} the given leader motor.
     * @param leader the motor to follow
     * @see #follow(GenericMotorController, boolean)
     * @see #addFollower(GenericMotorController)
     */
    default void follow(GenericMotorController<T> leader) {
        follow(leader,false);
    }

    /**
     * Sets this motor to follow the given leader motor by copying its velocity.
     * The inverted parameter can be used invert the direction the motor follows the leader.
     * @param leader the motor to follow.
     * @param inverted sets the direction that this motor follows the leader.
     * @see #follow(GenericMotorController)
     * @see #addFollower(GenericMotorController)
     */
    void follow(GenericMotorController<T> leader, boolean inverted);

    /**
     * Sets the given motor to {@link #follow(GenericMotorController, boolean) follow} this motor.
     * this is a shortcut for {@code motor.follow(this);}
     * @param motor the motor that should follow this motor.
     * @see #follow(GenericMotorController, boolean)
     * @see #follow(GenericMotorController)
     */
    default void addFollower(GenericMotorController<T> motor) {
        motor.follow(this);
    }

    /**
     * The position reading of the motor's encoder with the {@link #getPositionConversionFactor() conversion factor} applied.
     * @see #getRawPosition()
     */
    @Getter(key="Position")
    default double getPosition() {
        return getRawPosition()/getPositionConversionFactor();
    }

    /**
     * The velocity reading of the motor's encoder with the {@link #getPositionConversionFactor() conversion factor} applied.
     * @see #getRawVelocity()
     */
    @Getter(key="Velocity")
    default double getVelocity() {
        return getRawVelocity()/getVelocityConversionFactor();
    }

    /**
     * Sets the encoder position to the given value. This does NOT move the motor, it only sets the encoder value.
     * @param value the new encoder value
     */
    void setEncoderPosition(double value);

    /**
     * Creates a new {@link GenericSpark} instance from the given motor and config. This is equivalent to {@link GenericSpark#GenericSpark(SparkBase, SparkBaseConfig) new GenericSpark(motor,config)}
     * @param motor the motor controller to use when creating the GenericSpark
     * @param config the motor config to use when creating the GenericSpark
     * @return the new GenericSpark instance.
     * @see GenericSpark
     * @see GenericSpark#GenericSpark(SparkBase, SparkBaseConfig)
     */
    static GenericSpark from(SparkBase motor, SparkBaseConfig config) {
        return new GenericSpark(motor,config);
    }

    /**
     * Creates a new {@link GenericTalon} instance from the given motor. This is equivalent to {@link GenericTalon#GenericTalon(CommonTalon) new GenericTalon(motor)}
     * @param motor the motor controller to use when creating the GenericTalon
     * @return the new GenericTalon instance.
     * @see GenericTalon
     * @see GenericTalon#GenericTalon(CommonTalon)
     */
    static GenericTalon from(TalonFX motor) {
        return new GenericTalon(motor);
    }

    /**
     * Stops all motor movement until the motor is told to move again.
     */
    void stop();

}
