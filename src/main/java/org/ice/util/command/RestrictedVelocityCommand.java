package org.ice.util.command;

import edu.wpi.first.wpilibj2.command.Command;
import org.ice.util.constants.DoubleConstant;
import org.ice.util.subsystem.VelocitySubsystem;

import java.util.function.Supplier;

/**
 * Abstract command subclass that represents a command with a restricted pool of velocities that it allows as input.
 * Subclasses should use an {@code enum} that implements the {@link DoubleConstant} class to store the set of allowed values.
 * An example implementation might look something like this:
 * <pre>
 * {@code
 *  public class SpinFlywheelCommand extends RestrictedVelocityCommand<Velocity, FlywheelSubsystem> {
 *      //enum with all of the allowed position inputs
 *      public enum Velocity implements DoubleConstant {
 *          idle          {public double getValue() {return ...;}},
 *          spinning      {public double getValue() {return ...;}},
 *      }
 *      //other code...
 *  }
 * }
 * </pre>
 * @param <T> Enum that implements DoubleConstant, and represents all the allowed values this command accepts.
 * @param <S> The subsystem that this command uses and requires.
 */
public abstract class RestrictedVelocityCommand<T extends DoubleConstant, S extends VelocitySubsystem> extends Command {

    private Supplier<T> veloSupplier;

    private S subsystem;

    public RestrictedVelocityCommand(T velocity, S subsystem) {
        this(()->velocity,subsystem);
    }

    public RestrictedVelocityCommand(Supplier<T> velocitySupplier, S subsystem) {
        veloSupplier = velocitySupplier;
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        subsystem.setVelocity(veloSupplier.get().getValue());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
