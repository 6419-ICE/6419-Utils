package org.example.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

public interface GenericMotorController<T> {
    T getMotor();
    void controlRaw(double input, ControlType type);
    default void control(double input, ControlType type) {
        if (type==ControlType.DUTY_CYCLE||type==ControlType.VOLTAGE) controlRaw(input,type);
        else controlRaw(input*getConversionFactor(),type);
    }
    default void set(double power) {
        controlRaw(power,ControlType.DUTY_CYCLE);
    }
    double get();
    double getTemp();
    double getOutputCurrent();
    double getRawPosition();
    double getRawAbsolutePosition();
    double getRawVelocity();
    double getConversionFactor();
    void setConversionFactor(double factor);
    default void follow(GenericMotorController<T> leader) {
        follow(leader,false);
    }
    void follow(GenericMotorController<T> leader, boolean inverted);
    default void addFollower(GenericMotorController<T> motor) {
        motor.follow(this);
    }
    default double getPosition() {
        return getRawPosition()*getConversionFactor();
    }
    default double getAbsolutePosition() {
        return getRawAbsolutePosition()*getConversionFactor();
    }
    default double getVelocity() {
        return getRawVelocity()*getConversionFactor();
    }
    static GenericSpark fromSpark(SparkBase motor, SparkBaseConfig config) {
        return new GenericSpark(motor,config);
    }
    static GenericTalonFX fromTalonFX(TalonFX motor) {
        return new GenericTalonFX(motor);
    }
    void stop();
}
