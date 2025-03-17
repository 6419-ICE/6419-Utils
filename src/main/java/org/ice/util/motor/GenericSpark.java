package org.ice.util.motor;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * Spark MAX and Spark FLEX implementation of {@link GenericMotorController}
 * @see GenericTalon
 */
public class GenericSpark implements GenericMotorController<SparkBase> {

    private SparkBase motor;

    private SparkClosedLoopController controller;

    private double posConversion = 1.0, veloConversion = 1.0;

    private SparkBaseConfig config;

    /**
     * Constructs a new GenericSpark instance using the given motor and config.
     * The motor config is necessary for {@link #follow(GenericMotorController, boolean) following} other motors, and is immediately applied to the motor.
     * @param motor the motor that this object uses.
     * @param config the config of that motor.
     * @see #from(SparkBase, SparkBaseConfig)
     */
    public GenericSpark(SparkBase motor, SparkBaseConfig config) {
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        this.motor = motor;
        controller = motor.getClosedLoopController();
        this.config = config;
    }

    /**{@inheritDoc}*/
    @Override
    public SparkBase getMotor() {
        return motor;
    }

    /**{@inheritDoc}*/
    @Override
    public void controlRaw(double input, ControlType type) {
        controller.setReference(input,type.asSparkControl());
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Power")
    public double get() {
        return motor.get();
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Temperature")
    public double getTemp() {
        return motor.getMotorTemperature();
    }

    /**{@inheritDoc}*/
    @Override
    public double getRawPosition() {
        return motor.getEncoder().getPosition();
    }

    /**{@inheritDoc}*/
    @Override
    public double getRawVelocity() {
        return motor.getEncoder().getVelocity();
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Conversion Factor")
    public double getPositionConversionFactor() {
        return posConversion;
    }

    /**{@inheritDoc}*/
    @Override
    public void setPositionConversionFactor(double factor) {
        posConversion = factor;
    }

    /**{@inheritDoc}*/
    @Override
    public double getVelocityConversionFactor() {
        return veloConversion;
    }

    /**{@inheritDoc}*/
    @Override
    public void setVelocityConversionFactor(double factor) {
        veloConversion = factor;
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Motor ID")
    public int getMotorID() {
        return motor.getDeviceId();
    }

    /**{@inheritDoc}*/
    @Override
    public void follow(GenericMotorController<SparkBase> leader, boolean inverted) {
        config.follow(leader.getMotor(),inverted);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    /**{@inheritDoc}*/
    @Override
    public void setEncoderPosition(double value) {
        motor.getEncoder().setPosition(value);
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Output Current")
    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    /**{@inheritDoc}*/
    @Override
    public void stop() {
        motor.stopMotor();
    }


}
