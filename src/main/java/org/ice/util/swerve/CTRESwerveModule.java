package org.ice.util.swerve;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.jni.CANBusJNI;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import org.ice.util.motor.GenericTalon;

//TODO FIX THIS WHOLE CLASS
public class CTRESwerveModule extends SwerveModule {
    private com.ctre.phoenix6.swerve.SwerveModule<?,?,?> internalModule;

    public CTRESwerveModule(int driveMotorID, int turnMotorID, ModuleConfig moduleConfig, double angularOffset, Translation2d location) {
        super(angularOffset);

        SwerveModuleConstantsFactory<ParentConfiguration,ParentConfiguration,?> factory = new SwerveModuleConstantsFactory<>();
        DriveConfig driveConfig = moduleConfig.driveConfig;
        TurnConfig turnConfig = moduleConfig.turnConfig;
        factory.withDriveMotorGearRatio(driveConfig.reduction)
                .withSteerMotorGearRatio(0.0) //TODO
                .withCouplingGearRatio(0.0) //TODO
                .withWheelRadius(moduleConfig.wheelDiameter/2f)
                .withSteerMotorGains(turnConfig.pid.asSlot0Config())
                .withDriveMotorGains(driveConfig.pid.asSlot0Config())
                .withSteerMotorClosedLoopOutput(moduleConfig.hasTalonPro ? SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC : SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(moduleConfig.hasTalonPro ? SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC : SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                //.withSlipCurrent(SlipCurrent) todo maybe add?
                //.withSpeedAt12Volts(SpeedAt12Volts) only used in open loop
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated) //maybe add config for this later?
                .withFeedbackSource(moduleConfig.hasTalonPro ? SwerveModuleConstants.SteerFeedbackType.FusedCANcoder : SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
                .withDriveMotorInitialConfigs(driveConfig.getParentConfig())
                .withSteerMotorInitialConfigs(turnConfig.getParentConfig());
                //.withEncoderInitialConfigs(EncoderInitialConfigs) not needed
                //.withSteerInertia(SteerInertia) todo maybe add?
                //.withDriveInertia(DriveInertia) todo maybe add?
                //.withSteerFrictionVoltage(SteerFrictionVoltage) todo maybe add?
                //.withDriveFrictionVoltage(DriveFrictionVoltage) todo maybe add?

        SwerveModuleConstants<?,?,?> constants = factory.createModuleConstants(
                turnMotorID,
                driveMotorID,
                turnConfig.encoderID,
                Units.Degrees.of(0), //handled by superclass
                location.getMeasureX(),  //skipping for now
                location.getMeasureY(),  //skipping for now
                driveConfig.inverted,
                turnConfig.inverted,
                turnConfig.encoderInverted
        );
        internalModule = new com.ctre.phoenix6.swerve.SwerveModule<CommonTalon, CommonTalon, ParentDevice>(
                driveConfig.motorType == MotorType.TALONFX ? TalonFX::new : TalonFXS::new,
                turnConfig.motorType == MotorType.TALONFX ? TalonFX::new : TalonFXS::new,
                CANcoder::new,
                constants,
                "",
                0,
                moduleConfig.moduleIndex
        );
        driveMotor = new GenericTalon(internalModule.getDriveMotor());
        turningMotor = new GenericTalon(internalModule.getSteerMotor());
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
            double wheelDiameter,
            int moduleIndex,
            boolean hasTalonPro
    ) {}
    public record DriveConfig(
            MotorType motorType,
            PIDValues pid,
            double maxOutput,
            int currentLimit,
            boolean inverted,
            double reduction
    ) {
        private ParentConfiguration getParentConfig() {
            return switch (motorType) {
                case TALONFX -> new TalonFXConfiguration()
                        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(currentLimit))
                        .withMotorOutput(new MotorOutputConfigs().withPeakForwardDutyCycle(maxOutput).withPeakReverseDutyCycle(-maxOutput));
                case TALONFXS -> new TalonFXSConfiguration()
                        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(currentLimit))
                        .withMotorOutput(new MotorOutputConfigs().withPeakForwardDutyCycle(maxOutput).withPeakReverseDutyCycle(-maxOutput));
            };
        }
    }
    public record TurnConfig(
            MotorType motorType,
            int encoderID,
            PIDValues pid,
            double maxOutput,
            int currentLimit,
            boolean inverted,
            boolean encoderInverted
    ) {
        private ParentConfiguration getParentConfig() {
            return switch (motorType) {
                case TALONFX -> new TalonFXConfiguration()
                        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(currentLimit))
                        .withMotorOutput(new MotorOutputConfigs().withPeakForwardDutyCycle(maxOutput).withPeakReverseDutyCycle(-maxOutput));
                case TALONFXS -> new TalonFXSConfiguration()
                        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(currentLimit))
                        .withMotorOutput(new MotorOutputConfigs().withPeakForwardDutyCycle(maxOutput).withPeakReverseDutyCycle(-maxOutput));
            };
        }
    }
}
