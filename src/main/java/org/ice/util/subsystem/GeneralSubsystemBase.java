package org.ice.util.subsystem;

import org.ice.util.motor.ControlType;
import org.ice.util.motor.GenericMotorController;

/**
 * Abstract implementation of {@link DutyCycleSubsystem}, {@link PositionSubsystem}, and {@link VelocitySubsystem}, using a {@link GenericMotorController} to control the subsystem's motor.
 */
public abstract class GeneralSubsystemBase extends PositionSubsystemBase implements DutyCycleSubsystem, VelocitySubsystem {

    private GenericMotorController<?> motor;

    private double goal;
    /**
     * Constructs the subsystem with the given motor. subclasses should fill this value out on their own, instead of keeping it as a constructor parameter.
     * @param motor the motor used by this subsystem
     */
    public GeneralSubsystemBase(GenericMotorController<?> motor) {
        super(motor);
        this.motor = motor;
    }

    /**{@inheritDoc}*/
    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Power")
    public double getPower() {
        return motor.get();
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
        motor.control(velo,ControlType.VELOCITY);
    }
}
