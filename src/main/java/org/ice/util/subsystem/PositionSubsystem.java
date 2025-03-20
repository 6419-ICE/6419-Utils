package org.ice.util.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.ice.util.motor.ControlType;
import org.ice.util.motor.GenericMotorController;

/**
 * Interface defining methods for a Subsystem used for controlling a motor via {@link ControlType#POSITION position control}.
 * @see PositionSubsystemBase
 * @see GeneralSubsystemBase
 */
public interface PositionSubsystem extends Subsystem {

    /**
     * Sets the {@link ControlType#POSITION position} of the subsystem's motor.
     * @param position the position to set the motor to
     */
    void setPosition(double position);

    /**
     * Gets the current position of the subsystem's motor
     * @return the current position of the subsystem's motor
     */
    double getPosition();

    /**
     * The target position of the subsystem's motor as set by {@link #setPosition(double)}.
     * @return the target position of the subsystem's motor
     */
    double getGoal();

    /**
     * Returns if the subsystem's motor is within {@link #getTolerance() tolerance} of its {@link #getGoal() goal}
     * @return if the subsystem's motor is at its goal
     */
    boolean atGoal();

    /**
     * The allowable amount that the subsystem's motor's position can be off and still be considered {@link #atGoal() at its goal}
     * @return the amount that the motor's position can be off.
     */
    double getTolerance();

    GenericMotorController<?> getMotor();
}
