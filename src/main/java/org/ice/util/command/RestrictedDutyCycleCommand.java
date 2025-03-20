package org.ice.util.command;

import edu.wpi.first.wpilibj2.command.Command;
import org.ice.util.constants.DoubleConstant;
import org.ice.util.subsystem.DutyCycleSubsystem;

import java.util.function.Supplier;

/**
 * Abstract command subclass that represents a command with a restricted pool of powers that it allows as input.
 * Subclasses should use an {@code enum} that implements the {@link DoubleConstant} class to store the set of allowed values.
 * An example implementation might look something like this:
 * <pre>
 * {@code
 *  public class SpinIntakeCommand extends RestrictedDutyCycleCommand<Power, IntakeSubsystem> {
 *      //enum with all of the allowed power inputs
 *      public enum Power implements DoubleConstant {
 *          intake  {public double getValue() {return ...;}},
 *          outtake {public double getValue() {return ...;}}
 *      }
 *      //other code...
 *  }
 * }
 * NOTE: This command does not end on its own
 * </pre>
 * @param <T> Enum that implements DoubleConstant, and represents all the allowed values this command accepts.
 * @param <S> The subsystem that this command uses and requires.
 */
public abstract class RestrictedDutyCycleCommand<T extends Enum<T> & DoubleConstant , S extends DutyCycleSubsystem> extends Command {

    private Supplier<T> powerSupplier;

    private S subsystem;

    public RestrictedDutyCycleCommand(T power, S subsystem) {
        this(()->power,subsystem);
    }

    public RestrictedDutyCycleCommand(Supplier<T> powerSupplier, S subsystem) {
        this.powerSupplier = powerSupplier;
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        subsystem.setPower(powerSupplier.get().getValue());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
