package org.example.motor;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;

public class GenericSpark implements GenericMotorController<SparkBase> {
    private SparkBase motor;
    private SparkClosedLoopController controller;
    private double conversionFactor = 1.0;
    private SparkBaseConfig config;
    public GenericSpark(SparkBase motor, SparkBaseConfig config) {
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        this.motor = motor;
        controller = motor.getClosedLoopController();
        this.config = config;
    }
    @Override
    public SparkBase getMotor() {
        return motor;
    }
    @Override
    public void controlRaw(double input, ControlType type) {
        controller.setReference(input,type.asSparkControl());
    }
    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public double getTemp() {
        return motor.getMotorTemperature();
    }

    @Override
    public double getRawPosition() {
        return motor.getEncoder().getPosition();
    }
    @Override
    public double getRawAbsolutePosition() {
        return motor.getAbsoluteEncoder().getPosition();
    }
    @Override
    public double getRawVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public double getConversionFactor() {
        return conversionFactor;
    }

    @Override
    public void setConversionFactor(double factor) {
        conversionFactor = factor;
    }

    @Override
    public void follow(GenericMotorController<SparkBase> leader, boolean inverted) {
        config.follow(leader.getMotor(),inverted);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
