package org.example.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import org.example.constants.DoubleConstant;
import org.example.subsystem.DutyCycleSubsystem;
import org.example.subsystem.PositionSubsystem;

import java.util.function.Supplier;

public abstract class RestrictedDutyCycleCommand<T extends DoubleConstant, S extends DutyCycleSubsystem> extends Command {
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
    public abstract double getTolerance();
    @Override
    public boolean isFinished() {
        return false;
    }
}
