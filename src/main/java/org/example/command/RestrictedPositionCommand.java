package org.example.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.example.constants.DoubleConstant;
import org.example.subsystem.PositionSubsystem;

import java.util.function.Supplier;

public abstract class RestrictedPositionCommand<T extends DoubleConstant, S extends PositionSubsystem> extends Command {
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
