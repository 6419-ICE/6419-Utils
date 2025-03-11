package org.example.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface PositionSubsystem extends Subsystem {
    void setPosition(double position);
    double getPosition();
    double getGoal();
    boolean atGoal();
}
