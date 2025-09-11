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
import org.ice.util.motor.GenericSpark;
import org.ice.util.sendable.AnnotatedSendable;

public class REVSwerveModule extends SwerveModule {
    private SparkAbsoluteEncoder turnEncoder;


    public REVSwerveModule(int driveMotorID, int turnMotorID, ModuleConfig moduleConfig, double angularOffset) {
        super(angularOffset);
        configureDriveMotor(driveMotorID,moduleConfig);
        configureTurningMotor(turnMotorID,moduleConfig);
        driveMotor.setEncoderPosition(0.0);
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
    @Getter(key="Raw Module Angle")
    public double getRawAngle() {
        return turnEncoder.getPosition();
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
