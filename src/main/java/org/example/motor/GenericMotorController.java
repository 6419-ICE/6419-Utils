package org.example.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import org.example.sendable.AnnotatedSendable;

public interface GenericMotorController<T> extends AnnotatedSendable {

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

    int getMotorID();

    default void follow(GenericMotorController<T> leader) {
        follow(leader,false);
    }

    void follow(GenericMotorController<T> leader, boolean inverted);

    default void addFollower(GenericMotorController<T> motor) {
        motor.follow(this);
    }

    @Getter(key="Position")
    default double getPosition() {
        return getRawPosition()*getConversionFactor();
    }

    //No Getter here because it's not always supported
    default double getAbsolutePosition() {
        return getRawAbsolutePosition()*getConversionFactor();
    }

    @Getter(key="Velocity")
    default double getVelocity() {
        return getRawVelocity()*getConversionFactor();
    }

    static GenericSpark fromSpark(SparkBase motor, SparkBaseConfig config) {
        return new GenericSpark(motor,config);
    }

    static GenericTalon fromTalonFX(TalonFX motor) {
        return new GenericTalon(motor);
    }

    void stop();

}
