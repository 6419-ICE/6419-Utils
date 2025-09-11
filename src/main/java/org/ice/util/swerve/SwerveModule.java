package org.ice.util.swerve;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ice.util.motor.ControlType;
import org.ice.util.motor.GenericMotorController;
import org.ice.util.motor.GenericSpark;
import org.ice.util.sendable.AnnotatedSendable;

public abstract class SwerveModule implements AnnotatedSendable {
    protected GenericMotorController<?> driveMotor, turningMotor;
    private double angularOffset;
    private SwerveModuleState desiredState;

    protected SwerveModule(double angularOffset) {
        this.angularOffset = angularOffset;
    }

    @Getter(key="Module Angle")
    public double getWheelAngle() {
        return getRawAngle()-angularOffset;
    }

    public abstract double getRawAngle();

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(),
                new Rotation2d((getWheelAngle())));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition(),
                new Rotation2d((getWheelAngle()))
        );
    }
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angularOffset));
        correctedDesiredState.optimize(getPosition().angle);
        // Optimize the reference state to avoid spinning further than 90 degrees.
        // SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        //     new Rotation2d(turnEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        driveMotor.control(correctedDesiredState.speedMetersPerSecond, ControlType.VELOCITY);
        turningMotor.control(correctedDesiredState.angle.getRadians(),ControlType.POSITION);
        this.desiredState = correctedDesiredState;
    }
    public void driveRaw(Rotation2d angle, double power) {
        setDesiredState(new SwerveModuleState(0,angle));
        driveMotor.control(power, ControlType.DUTY_CYCLE);
    }
    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
    }
    @Getter(key="Drive Temperature")
    public double getDriveTemp() {
        return driveMotor.getTemp();
    }
    @Getter(key="Turn Temperature")
    public double getTurnTemp() {
        return turningMotor.getTemp();
    }
    @Getter(key="Drive Current")
    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }
    @Getter(key="Turn Current")
    public double getTurnCurrent() {
        return turningMotor.getOutputCurrent();
    }
    @Getter(key="Desired Speed")
    public double getDesiredSpeed() {
        return desiredState.speedMetersPerSecond;
    }
    @Getter(key="Desired Angle")
    public double getDesiredAngleDegrees() {
        return desiredState.angle.getDegrees();
    }
    @Getter(key="Drive Velocity")
    public double getVelocity() {
        return driveMotor.getVelocity();
    }
}
