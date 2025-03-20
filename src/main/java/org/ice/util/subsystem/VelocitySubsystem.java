package org.ice.util.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.ice.util.motor.ControlType;
import org.ice.util.motor.GenericMotorController;

/**
 * Interface defining methods for a Subsystem used for controlling a motor via {@link ControlType#VELOCITY velocity control}.
 * @see VelocitySubsystemBase
 * @see GeneralSubsystemBase
 */
public interface VelocitySubsystem extends Subsystem {
    /**
     * Gets the current velocity of the subsystem's motor
     * @return the current velocity of the subsystem's motor
     */
    double getVelocity();

    /**
     * Sets the target velocity of the subsystem's motor.
     * @param velo the target velocity of the subsystem's motor.
     */
    void setVelocity(double velo);

    GenericMotorController<?> getMotor();
}
