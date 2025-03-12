package org.example.subsystem;

import org.example.motor.ControlType;
import org.example.motor.GenericMotorController;
import org.example.sendable.AnnotatedSubsystemBase;

public abstract class DutyCycleSubsystemBase extends AnnotatedSubsystemBase implements DutyCycleSubsystem {

    private GenericMotorController<?> motor;

    public DutyCycleSubsystemBase(GenericMotorController<?> motor) {
        this.motor = motor;
    }

    @Override
    public void setPower(double power) {
        motor.control(power, ControlType.DUTY_CYCLE);
    }

    @Override
    @Getter(key="Power")
    public double getPower() {
        return motor.get();
    }
}
