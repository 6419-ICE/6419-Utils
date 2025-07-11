package org.ice.util.swerve;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ice.util.motor.GenericSpark;
import org.ice.util.motor.GenericTalon;

public class CTRESwerveModule extends SwerveModule {
    // import and translate what would be the Talon equivalent of the abs encoder
    private GenericTalon talon;
    private SwerveModuleState desiredState;

    public CTRESwerveModule(int driveMotorID, int turnMotorID, ModuleConfig moduleConfig, double angularOffset) {
        super(angularOffset);
        configureDriveMotor(driveMotorID,moduleConfig);
        configureTurningMotor(turnMotorID,moduleConfig);
        desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(talon.getPosition()));
        driveMotor.setEncoderPosition(0.0);
    }

    private void configureDriveMotor(int driveMotorID, ModuleConfig moduleConfig) {
//
//        SparkBaseConfig sparkConfig;
//        if (driveConfig.motorType == REVSwerveModule.MotorType.SPARK_FLEX) {
//            motor = new SparkFlex(motorID, SparkLowLevel.MotorType.kBrushless);
//            sparkConfig = new SparkFlexConfig();
//        } else {
//            motor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
//            sparkConfig = new SparkMaxConfig();
//        }
//        sparkConfig.closedLoop
//                .pidf(driveConfig.pid.getP(),driveConfig.pid.getI(),driveConfig.pid.getD(),driveConfig.pid.getFF())
//                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
//                .outputRange(driveConfig.minOutput, driveConfig.maxOutput);
//        sparkConfig.encoder
//                //meters
//                .positionConversionFactor((config.wheelDiameter * Math.PI) / driveConfig.reduction)
//                //meters per second
//                .velocityConversionFactor((config.wheelDiameter * Math.PI) / driveConfig.reduction / 60.0);
//        sparkConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
//        sparkConfig.smartCurrentLimit(driveConfig.currentLimit);
//        sparkConfig.inverted(driveConfig.inverted);
//
//        driveMotor = new GenericSpark(motor,sparkConfig);
//        motor.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        DriveConfig driveConfig = moduleConfig.driveConfig;
        GenericTalon talon;
        if (driveConfig.motorType == MotorType.TALONFX) {
            TalonFX motor = new TalonFX(driveMotorID);
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.Slot0.withKP(driveConfig.pid.getP()).withKI(driveConfig.pid.getI()).withKD(driveConfig.pid.getD()).withKV(1/driveConfig.pid.getFF());
            config.CurrentLimits.StatorCurrentLimit = driveConfig.currentLimit;
            config.MotorOutput.Inverted = driveConfig.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.PeakReverseDutyCycle = driveConfig.maxOutput;
            config.MotorOutput.PeakForwardDutyCycle = driveConfig.maxOutput;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            talon = new GenericTalon(motor);
        } else {
            TalonFXS motor = new TalonFXS(driveMotorID);
            TalonFXSConfiguration config = new TalonFXSConfiguration();
            config.Slot0.withKP(driveConfig.pid.getP()).withKI(driveConfig.pid.getI()).withKD(driveConfig.pid.getD()).withKV(1/driveConfig.pid.getFF());
            config.CurrentLimits.StatorCurrentLimit = driveConfig.currentLimit;
            config.MotorOutput.Inverted = driveConfig.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.PeakReverseDutyCycle = driveConfig.maxOutput;
            config.MotorOutput.PeakForwardDutyCycle = driveConfig.maxOutput;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            talon = new GenericTalon(motor);
        }
        this.driveMotor = talon;
        driveMotor.setPositionConversionFactor((moduleConfig.wheelDiameter * Math.PI) / driveConfig.reduction);
        driveMotor.setVelocityConversionFactor((moduleConfig.wheelDiameter * Math.PI) / driveConfig.reduction / 60.0);
    }

    private void configureTurningMotor(int turnMotorID, ModuleConfig moduleConfig) {
//        sparkConfig.closedLoop
//                .pidf(turnConfig.pid.getP(),turnConfig.pid.getI(),turnConfig.pid.getD(),turnConfig.pid.getFF())
//                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
//                .positionWrappingEnabled(true)
//                .positionWrappingInputRange(0.0,Math.PI*2.0)
//                .outputRange(turnConfig.minOutput,turnConfig.maxOutput);
//        sparkConfig.absoluteEncoder.setSparkMaxDataPortConfig();
//        sparkConfig.absoluteEncoder.positionConversionFactor(Math.PI*2);
//        sparkConfig.absoluteEncoder.velocityConversionFactor(Math.PI*2/60);
//        sparkConfig.smartCurrentLimit(turnConfig.currentLimit);
//        sparkConfig.inverted(turnConfig.inverted);
//        sparkConfig.absoluteEncoder.inverted(turnConfig.encoderInverted);
//        sparkConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
//        turnEncoder = motor.getAbsoluteEncoder();
//        turningMotor = new GenericSpark(motor, sparkConfig);
//        motor.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        GenericTalon talon;
        TurnConfig turnConfig = moduleConfig.turnConfig;
        if (turnConfig.motorType == MotorType.TALONFX) {
            TalonFX motor = new TalonFX(turnMotorID);
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.Slot0.withKP(turnConfig.pid().getP()).withKI(turnConfig.pid().getI()).withKD(turnConfig.pid().getD()).withKV(1/turnConfig.pid().getFF());
            config.Feedback.FeedbackSensorSource = turnConfig.feedbackSensorSource;
            config.ClosedLoopGeneral.ContinuousWrap = true;
            config.MotorOutput.PeakReverseDutyCycle = turnConfig.maxOutput;
            config.MotorOutput.PeakForwardDutyCycle = turnConfig.maxOutput;
            config.CurrentLimits.StatorCurrentLimit = turnConfig.currentLimit;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = turnConfig.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            talon = new GenericTalon(motor);
        } else {
            TalonFXS motor = new TalonFXS(turnMotorID);
            TalonFXSConfiguration config = new TalonFXSConfiguration();
        }
    }

    @Override
    public double getRawAngle() {
        return 0;
    }

    public enum MotorType {
        TALONFX,
        TALONFXS
    }
    public record ModuleConfig(
            DriveConfig driveConfig,
            TurnConfig turnConfig,
            double wheelDiameter
    ) {}
    public record DriveConfig(
            MotorType motorType,
            PIDValues pid,
            double maxOutput,
            int currentLimit,
            boolean inverted,
            double reduction
    ) {}
    public record TurnConfig(
            MotorType motorType,
            FeedbackSensorSourceValue feedbackSensorSource,
            PIDValues pid,
            double maxOutput,
            int currentLimit,
            boolean inverted,
            boolean encoderInverted
    ) {}
}
