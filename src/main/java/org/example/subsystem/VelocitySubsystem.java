package org.example.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface VelocitySubsystem extends Subsystem {

    double getVelocity();

    void setVelocity(double velo);
}
