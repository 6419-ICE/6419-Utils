package org.ice.util.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ice.util.sendable.AnnotatedSendable;

public interface SwerveModule extends AnnotatedSendable {


    double getWheelAngle();

    double getRawAngle();

    SwerveModuleState getState();

    SwerveModuleState getDesiredState();

    SwerveModulePosition getPosition();

    void setDesiredState(SwerveModuleState desiredState);

    void resetDriveEncoder();

    double getDriveTempCelsius();

    double getTurnTempCelsius();

    double getDriveCurrent();

    double getTurnCurrent();

    double getDesiredSpeed();

    double getDesiredAngleDegrees();

    double getVelocity();

}
