package org.example.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class GenericTalonFX implements GenericMotorController<TalonFX> {
    private TalonFX motor;
    private StatusSignal<Temperature> tempSignal;
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<Current> currentSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private double conversionFactor = 1;
    public GenericTalonFX(TalonFX motor) {
        this.motor = motor;
        tempSignal = motor.getDeviceTemp();
        positionSignal = motor.getPosition();
        currentSignal = motor.getTorqueCurrent();
        velocitySignal = motor.getVelocity();
    }
    @Override
    public TalonFX getMotor() {
        return motor;
    }

    @Override
    public void controlRaw(double input, ControlType type) {
        motor.setControl(type.asTalonControl(input));
    }

    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public double getTemp() {
        return tempSignal.refresh().getValue().in(Units.Fahrenheit);
    }

    @Override
    public double getRawPosition() {
        return positionSignal.refresh().getValue().in(Units.Rotations);
    }

    @Override
    public double getRawAbsolutePosition() {
        throw new UnsupportedOperationException("Talon FX does not support absolute encoder stuff");
    }

    @Override
    public double getOutputCurrent() {
        return currentSignal.refresh().getValue().in(Units.Amps);
    }

    @Override
    public double getRawVelocity() {
        return velocitySignal.getValue().in(Units.RotationsPerSecond);
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
    public void follow(GenericMotorController<TalonFX> leader, boolean inverted) {
        motor.setControl(new Follower(leader.getMotor().getDeviceID(),inverted));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
