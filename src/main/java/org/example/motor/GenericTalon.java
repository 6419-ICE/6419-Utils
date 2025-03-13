package org.example.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class GenericTalon implements GenericMotorController<CommonTalon> {
    private TalonWrapper motor;
    private StatusSignal<Temperature> tempSignal;
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<Current> currentSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private double conversionFactor = 1;
    public GenericTalon(CommonTalon motor) {
        this.motor = new TalonWrapper(motor);
        tempSignal = motor.getDeviceTemp();
        positionSignal = motor.getPosition();
        currentSignal = motor.getTorqueCurrent();
        velocitySignal = motor.getVelocity();
    }
    @Override
    public CommonTalon getMotor() {
        return motor.asTalon();
    }

    @Override
    public void controlRaw(double input, ControlType type) {

        motor.asTalon().setControl(type.asTalonControl(input));
    }

    @Override
    @Getter(key="Power")
    public double get() {
        return motor.isTalonFX() ? motor.asTalonFX().get() : motor.asTalonFXS().get();
    }

    @Override
    @Getter(key="Temperature")
    public double getTemp() {
        return tempSignal.refresh().getValue().in(Units.Fahrenheit);
    }

    @Override
    public double getRawPosition() {
        return positionSignal.refresh().getValue().in(Units.Rotations);
    }

    @Override
    public double getRawAbsolutePosition() {
        throw new UnsupportedOperationException("Talon does not support absolute encoder stuff maybe?");
    }

    @Override
    @Getter(key="Output Current")
    public double getOutputCurrent() {
        return currentSignal.refresh().getValue().in(Units.Amps);
    }

    @Override
    public double getRawVelocity() {
        return velocitySignal.getValue().in(Units.RotationsPerSecond);
    }

    @Override
    @Getter(key="Conversion Factor")
    public double getConversionFactor() {
        return conversionFactor;
    }

    @Override
    public void setConversionFactor(double factor) {
        conversionFactor = factor;
    }

    @Override
    @Getter(key="Motor ID")
    public int getMotorID() {
        return motor.isTalonFX() ? motor.asTalonFX().getDeviceID() : motor.asTalonFXS().getDeviceID();
    }

    @Override
    public void follow(GenericMotorController<CommonTalon> leader, boolean inverted) {
        motor.asTalon().setControl(new Follower(leader.getMotorID(),inverted));
    }

    @Override
    public void stop() {
        if (motor.isTalonFX()) motor.asTalonFX().stopMotor();
        else motor.asTalonFXS().stopMotor();
    }


    private static class TalonWrapper {

        private CommonTalon motor;

        private TalonFX motorFX = null;

        private TalonFXS motorFXS = null;

        public TalonWrapper(CommonTalon motor) {
            this.motor = motor;
            switch(motor) {
                case TalonFX fx -> motorFX = fx;
                case TalonFXS fxs -> motorFXS = fxs;
                default -> throw new IllegalStateException("Illegal Motor type: " + motor);
            }
        }

        public boolean isTalonFX() {
            return motorFXS == null;
        }

        public TalonFX asTalonFX() {
            return motorFX;
        }

        public TalonFXS asTalonFXS() {
            return motorFXS;
        }

        public CommonTalon asTalon() {
            return motor;
        }

    }
}
