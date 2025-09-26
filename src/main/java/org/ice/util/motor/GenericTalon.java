package org.ice.util.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.ice.util.swerve.PIDValues;

/**
 * TalonFX and TalonFXS implementation of {@link GenericMotorController}
 * @see GenericSpark
 */
public class GenericTalon implements GenericMotorController<CommonTalon> {
    private TalonWrapper motor;
    private StatusSignal<Temperature> tempSignal;
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<Current> currentSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private double posConversion = 1.0, veloConversion = 1.0;

    /**
     * Constructs a new GenericTalon instance using the given motor.
     * @param motor the motor
     */
    public GenericTalon(CommonTalon motor) {
        this.motor = new TalonWrapper(motor);
        tempSignal = motor.getDeviceTemp();
        positionSignal = motor.getPosition();
        currentSignal = motor.getTorqueCurrent();
        velocitySignal = motor.getVelocity();
    }

    public GenericTalon(TalonFX motor, TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
        this.motor = new TalonWrapper(motor);
        tempSignal = motor.getDeviceTemp();
        positionSignal = motor.getPosition();
        currentSignal = motor.getTorqueCurrent();
        velocitySignal = motor.getVelocity();
    }

    public GenericTalon(TalonFXS motor, TalonFXSConfiguration config) {
        motor.getConfigurator().apply(config);
        this.motor = new TalonWrapper(motor);
        tempSignal = motor.getDeviceTemp();
        positionSignal = motor.getPosition();
        currentSignal = motor.getTorqueCurrent();
        velocitySignal = motor.getVelocity();
    }

    /**{@inheritDoc}*/
    @Override
    public CommonTalon getMotor() {
        return motor.asTalon();
    }

    /**{@inheritDoc}*/
    @Override
    public void controlRaw(double input, ControlType type) {
        motor.asTalon().setControl(type.asTalonControl(input));
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Power")
    public double get() {
        return motor.isTalonFX() ? motor.asTalonFX().get() : motor.asTalonFXS().get();
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Temperature")
    public double getTemp() {
        return tempSignal.refresh().getValue().in(Units.Celsius);
    }

    /**{@inheritDoc}*/
    @Override
    public double getRawPosition() {
        return positionSignal.refresh().getValue().in(Units.Rotations);
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Output Current")
    public double getOutputCurrent() {
        return currentSignal.refresh().getValue().in(Units.Amps);
    }

    /**{@inheritDoc}*/
    @Override
    public double getRawVelocity() {
        return velocitySignal.getValue().in(Units.RotationsPerSecond);
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
        return motor.isTalonFX() ? motor.asTalonFX().getDeviceID() : motor.asTalonFXS().getDeviceID();
    }

    /**{@inheritDoc}*/
    @Override
    public void setPID(PIDValues pid) {
        if (motor.isTalonFX()) motor.asTalonFX().getConfigurator().apply(pid.asSlotConfig());
        else motor.asTalonFXS().getConfigurator().apply(pid.asSlotConfig());
    }

    /**{@inheritDoc}*/
    @Override
    public void follow(GenericMotorController<CommonTalon> leader, boolean inverted) {
        motor.asTalon().setControl(new Follower(leader.getMotorID(),inverted));
    }

    /**{@inheritDoc}*/
    @Override
    public void setEncoderPosition(double value) {
        motor.asTalon().setPosition(value);
    }

    /**{@inheritDoc}*/
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
            if (motor instanceof TalonFXS fxs) motorFXS = fxs;
            else if (motor instanceof TalonFX fx) motorFX = fx;
            else  throw new IllegalStateException("Illegal Motor type: " + motor);
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
