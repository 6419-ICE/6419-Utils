package org.ice.util.subsystem;

import org.ice.util.motor.ControlType;
import org.ice.util.motor.GenericMotorController;
import org.ice.util.sendable.AnnotatedSubsystemBase;

public abstract class VelocitySubsystemBase extends AnnotatedSubsystemBase implements VelocitySubsystem {

    private GenericMotorController<?> motor;

    /**
     * Constructs the subsystem with the given motor. subclasses should fill this value out on their own, instead of keeping it as a constructor parameter.
     * @param motor the motor used by this subsystem
     */
    public VelocitySubsystemBase(GenericMotorController<?> motor) {
        this.motor = motor;
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Velocity")
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**{@inheritDoc}*/
    @Override
    public void setVelocity(double velo) {
        motor.control(velo, ControlType.VELOCITY);
    }
}
