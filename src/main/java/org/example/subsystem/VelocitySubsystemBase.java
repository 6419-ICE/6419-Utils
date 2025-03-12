package org.example.subsystem;

import org.example.motor.ControlType;
import org.example.motor.GenericMotorController;
import org.example.sendable.AnnotatedSubsystemBase;

public abstract class VelocitySubsystemBase extends AnnotatedSubsystemBase implements VelocitySubsystem {

    private GenericMotorController<?> motor;

    public VelocitySubsystemBase(GenericMotorController<?> motor) {
        this.motor = motor;
    }

    @Override
    @Getter(key="Velocity")
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public void setVelocity(double velo) {
        motor.control(velo, ControlType.VELOCITY);
    }
}
