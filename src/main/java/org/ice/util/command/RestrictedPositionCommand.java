package org.ice.util.command;

import edu.wpi.first.wpilibj2.command.Command;
import org.ice.util.constants.DoubleConstant;
import org.ice.util.subsystem.PositionSubsystem;

import java.util.function.Supplier;

/**
 * Abstract command subclass that represents a command with a restricted pool of positions that it allows as input.
 * Subclasses should use an {@code enum} that implements the {@link DoubleConstant} class to store the set of allowed values.
 * An example implementation might look something like this:
 * <pre>
 * {@code
 *  public class MoveElevatorCommand extends RestrictedPositionCommand<Position, ElevatorSubsystem> {
 *      //enum with all of the allowed position inputs
 *      public enum Position implements DoubleConstant {
 *          inside          {public double getValue() {return ...;}},
 *          extendedPartial {public double getValue() {return ...;}},
 *          extendedFull    {public double getValue() {return ...;}}
 *      }
 *      //other code...
 *  }
 * }
 * </pre>
 * @param <T> Enum that implements DoubleConstant, and represents all the allowed values this command accepts.
 * @param <S> The subsystem that this command uses and requires.
 */
public abstract class RestrictedPositionCommand<T extends Enum<T> & DoubleConstant, S extends PositionSubsystem> extends Command {

    private Supplier<T> posSupplier;

    private S subsystem;

    public RestrictedPositionCommand(T position, S subsystem) {
        this(()->position,subsystem);
    }

    public RestrictedPositionCommand(Supplier<T> positionSupplier, S subsystem) {
        posSupplier = positionSupplier;
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        subsystem.setPosition(posSupplier.get().getValue());
    }

    @Override
    public boolean isFinished() {
        return subsystem.atGoal();
    }

}
