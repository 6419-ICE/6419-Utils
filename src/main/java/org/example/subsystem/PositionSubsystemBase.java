package org.example.subsystem;

import edu.wpi.first.math.MathUtil;
import org.example.motor.ControlType;
import org.example.motor.GenericMotorController;
import org.example.sendable.AnnotatedSubsystem;

public abstract class PositionSubsystemBase extends AnnotatedSubsystem implements PositionSubsystem {
    private GenericMotorController<?> motor;
    private double goal;
    public PositionSubsystemBase(GenericMotorController<?> motor) {
        this.motor = motor;
    }
    @Override
    public void setPosition(double position) {
        motor.control(goal = position, ControlType.POSITION);
    }
    @Override
    @Getter(key="Position")
    public double getPosition() {
        return motor.getPosition();
    }
    @Override
    @Getter(key="Goal")
    public double getGoal() {
        return goal;
    }
    @Getter(key="Tolerance")
    public abstract double getTolerance();
    @Override
    @Getter(key="At Goal")
    public boolean atGoal() {
        return MathUtil.isNear(goal,getPosition(),getTolerance());
    }

}
