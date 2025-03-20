package org.ice.util.subsystem;

import edu.wpi.first.math.MathUtil;
import org.ice.util.motor.ControlType;
import org.ice.util.motor.GenericMotorController;
import org.ice.util.sendable.AnnotatedSubsystemBase;

public abstract class PositionSubsystemBase extends AnnotatedSubsystemBase implements PositionSubsystem {

    private GenericMotorController<?> motor;

    private double goal;

    /**
     * Constructs the subsystem with the given motor. subclasses should fill this value out on their own, instead of keeping it as a constructor parameter.
     * @param motor the motor used by this subsystem
     */
    public PositionSubsystemBase(GenericMotorController<?> motor) {
        this.motor = motor;
    }

    /**{@inheritDoc}*/
    @Override
    public void setPosition(double position) {
        motor.control(goal = position, ControlType.POSITION);
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Position")
    public double getPosition() {
        return motor.getPosition();
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Goal")
    public double getGoal() {
        return goal;
    }

    /**{@inheritDoc}*/
    @Override
    @Getter(key="Tolerance")
    public abstract double getTolerance();

    /**{@inheritDoc}*/
    @Override
    @Getter(key="At Goal")
    public boolean atGoal() {
        return MathUtil.isNear(goal,getPosition(),getTolerance());
    }

    @Override
    public GenericMotorController<?> getMotor() {
        return motor;
    }



}
