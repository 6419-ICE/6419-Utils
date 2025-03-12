package org.example.command;

import edu.wpi.first.wpilibj2.command.Command;
import org.example.constants.DoubleConstant;
import org.example.subsystem.VelocitySubsystem;

import java.util.function.Supplier;

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
