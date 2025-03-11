package org.example.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DutyCycleSubsystem extends Subsystem {
    void setPower(double power);
    double getPower();
}
