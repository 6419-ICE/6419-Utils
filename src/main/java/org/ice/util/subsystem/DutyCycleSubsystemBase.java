package org.ice.util.subsystem;

import org.ice.util.motor.ControlType;
import org.ice.util.motor.GenericMotorController;
import org.ice.util.sendable.AnnotatedSubsystemBase;

/**
 * Abstract implementation of {@link DutyCycleSubsystem} using a {@link GenericMotorController} to control the subsystem's motor.
 * @see DutyCycleSubsystem
 */
public abstract class DutyCycleSubsystemBase extends AnnotatedSubsystemBase implements DutyCycleSubsystem {

    private GenericMotorController<?> motor;

    /**
     * Constructs the subsystem with the given motor. subclasses should fill this value out on their own, instead of keeping it as a constructor parameter.
     * @param motor the motor used by this subsystem
     */
    public DutyCycleSubsystemBase(GenericMotorController<?> motor) {
        this.motor = motor;
    }

    /**{@inheritDoc}*/
    @Override
    public void setPower(double power) {
        motor.control(power, ControlType.DUTY_CYCLE);
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Power")
    public double getPower() {
        return motor.get();
    }

    @Override
    public GenericMotorController<?> getMotor() {
        return motor;
    }
}
