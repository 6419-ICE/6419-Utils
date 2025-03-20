package org.ice.util.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.ice.util.motor.ControlType;
import org.ice.util.motor.GenericMotorController;

/**
 * Interface defining methods for a Subsystem used for controlling a motor via {@link ControlType#DUTY_CYCLE duty cycle control}.
 * @see DutyCycleSubsystemBase
 * @see GeneralSubsystemBase
 */
public interface DutyCycleSubsystem extends Subsystem {

    /**
     * Sets the power of the subsystem's motor as a percent from -1 to 1
     * @param power the power of the subsystem's motor
     * @see ControlType#DUTY_CYCLE
     */
    void setPower(double power);

    /**
     * The power of the subsystem's motor
     */
    double getPower();

    GenericMotorController<?> getMotor();
}
