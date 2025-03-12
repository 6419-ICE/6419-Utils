package org.example.subsystem;

import org.example.motor.ControlType;
import org.example.motor.GenericMotorController;

public abstract class GeneralSubsystemBase extends PositionSubsystemBase implements DutyCycleSubsystem, VelocitySubsystem {

    private GenericMotorController<?> motor;

    private double goal;

    public GeneralSubsystemBase(GenericMotorController<?> motor) {
        super(motor);
        this.motor = motor;
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    @Getter(key="Power")
    public double getPower() {
        return motor.get();
    }

    @Override
    @Getter(key="Velocity")
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public void setVelocity(double velo) {
        motor.control(velo,ControlType.VELOCITY);
    }
}
