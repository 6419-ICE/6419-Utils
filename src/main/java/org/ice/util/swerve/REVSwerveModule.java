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

public class REVSwerveModule implements SwerveModule {
    private SparkAbsoluteEncoder turnEncoder;
    protected GenericMotorController<?> driveMotor, turningMotor;
    private double angularOffset;
    private SwerveModuleState desiredState;

    public REVSwerveModule(int driveMotorID, int turnMotorID, ModuleConfig moduleConfig, double angularOffset) {
        configureDriveMotor(driveMotorID,moduleConfig);
        configureTurningMotor(turnMotorID,moduleConfig);
        driveMotor.setEncoderPosition(0.0);
        this.angularOffset = angularOffset;
    }

    private void configureTurningMotor(int motorID, ModuleConfig config) {
        TurnConfig turnConfig = config.turnConfig;
        SparkBase motor;
        SparkBaseConfig sparkConfig;
        if (turnConfig.motorType == MotorType.SPARK_FLEX) {
            motor = new SparkFlex(motorID, SparkLowLevel.MotorType.kBrushless);
            sparkConfig = new SparkFlexConfig();
        } else {
            motor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
            sparkConfig = new SparkMaxConfig();
        }
        sparkConfig.closedLoop
                .pidf(turnConfig.pid.getP(),turnConfig.pid.getI(),turnConfig.pid.getD(),turnConfig.pid.getFF())
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0.0,Math.PI*2.0)
                .outputRange(turnConfig.minOutput,turnConfig.maxOutput);
        sparkConfig.absoluteEncoder.setSparkMaxDataPortConfig();
        sparkConfig.absoluteEncoder.positionConversionFactor(Math.PI*2);
        sparkConfig.absoluteEncoder.velocityConversionFactor(Math.PI*2/60);
        sparkConfig.smartCurrentLimit(turnConfig.currentLimit);
        sparkConfig.inverted(turnConfig.inverted);
        sparkConfig.absoluteEncoder.inverted(turnConfig.encoderInverted);
        sparkConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        turnEncoder = motor.getAbsoluteEncoder();
        turningMotor = new GenericSpark(motor, sparkConfig);
        motor.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    private void configureDriveMotor(int motorID, ModuleConfig config) {
        DriveConfig driveConfig = config.driveConfig;
        SparkBase motor;
        SparkBaseConfig sparkConfig;
        if (driveConfig.motorType == MotorType.SPARK_FLEX) {
            motor = new SparkFlex(motorID, SparkLowLevel.MotorType.kBrushless);
            sparkConfig = new SparkFlexConfig();
        } else {
            motor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
            sparkConfig = new SparkMaxConfig();
        }
        sparkConfig.closedLoop
                .pidf(driveConfig.pid.getP(),driveConfig.pid.getI(),driveConfig.pid.getD(),driveConfig.pid.getFF())
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .outputRange(driveConfig.minOutput, driveConfig.maxOutput);
        sparkConfig.encoder
                //meters
                .positionConversionFactor((config.wheelDiameter * Math.PI) / driveConfig.reduction)
                //meters per second
                .velocityConversionFactor((config.wheelDiameter * Math.PI) / driveConfig.reduction / 60.0);
        sparkConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        sparkConfig.smartCurrentLimit(driveConfig.currentLimit);
        sparkConfig.inverted(driveConfig.inverted);

        driveMotor = new GenericSpark(motor,sparkConfig);
        motor.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    @Getter(key="Module Angle")
    public double getWheelAngle() {
        return getRawAngle()-angularOffset;
    }

    @Override
    @Getter(key="Raw Module Angle")
    public double getRawAngle() {
        return turnEncoder.getPosition();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(),
                new Rotation2d((getWheelAngle())));
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition(),
                new Rotation2d((getWheelAngle()))
        );
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angularOffset));
        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(getPosition().angle);
        // Command driving and turning SPARKS MAX towards their respective setpoints.
        driveMotor.control(correctedDesiredState.speedMetersPerSecond, ControlType.VELOCITY);
        turningMotor.control(correctedDesiredState.angle.getRadians(),ControlType.POSITION);
        this.desiredState = correctedDesiredState;
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.setEncoderPosition(0);
    }

    @Override
    @Getter(key="Drive Temperature")
    public double getDriveTempCelsius() {
        return driveMotor.getTemp();
    }

    @Override
    @Getter(key="Turn Temperature")
    public double getTurnTempCelsius() {
        return turningMotor.getTemp();
    }

    @Override
    @Getter(key="Drive Current")
    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }

    @Override
    @Getter(key="Turn Current")
    public double getTurnCurrent() {
        return turningMotor.getOutputCurrent();
    }

    @Override
    @Getter(key="Desired Speed")
    public double getDesiredSpeed() {
        return desiredState.speedMetersPerSecond;
    }

    @Override
    @Getter(key="Desired Angle")
    public double getDesiredAngleDegrees() {
        return desiredState.angle.getDegrees();
    }

    @Override
    @Getter(key="Drive Velocity")
    public double getVelocity() {
        return driveMotor.getVelocity();
    }

    public enum MotorType {
        SPARK_FLEX,
        SPARK_MAX
    }
    public record ModuleConfig(
            DriveConfig driveConfig,
            TurnConfig turnConfig,
            double wheelDiameter
    ) {}
    public record DriveConfig(
            MotorType motorType,
            PIDValues pid,
            double minOutput,
            double maxOutput,
            int currentLimit,
            boolean inverted,
            double reduction
    ) {}
    public record TurnConfig(
            MotorType motorType,
            PIDValues pid,
            double minOutput,
            double maxOutput,
            int currentLimit,
            boolean inverted,
            boolean encoderInverted
    ) {}
}
