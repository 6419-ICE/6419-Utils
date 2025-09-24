package org.ice.util.swerve;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import static com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;

public class CTRESwerveModule implements SwerveModule {
    private com.ctre.phoenix6.swerve.SwerveModule<CommonTalon, CommonTalon, ParentDevice> internalModule;
    private StatusSignal<Temperature> driveTemp, turnTemp;
    private StatusSignal<Current> driveCurrent, turnCurrent;
    private final boolean hasSub;
    public CTRESwerveModule(int driveMotorID, int turnMotorID, ModuleConfig moduleConfig, double angularOffset, Translation2d location) {
        hasSub = moduleConfig.hasTalonPro;

        SwerveModuleConstantsFactory<ParentConfiguration,ParentConfiguration,?> factory = new SwerveModuleConstantsFactory<>();
        DriveConfig driveConfig = moduleConfig.driveConfig;
        TurnConfig turnConfig = moduleConfig.turnConfig;
        factory.withDriveMotorGearRatio(driveConfig.reduction)
                .withSteerMotorGearRatio(turnConfig.reduction) //TODO
                .withCouplingGearRatio(1.0) //TODO
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
                Units.Degrees.of(angularOffset), //handled by superclass
                location.getMeasureX(),  //skipping for now
                location.getMeasureY(),  //skipping for now
                driveConfig.inverted,
                turnConfig.inverted,
                turnConfig.encoderInverted
        );
        internalModule = new com.ctre.phoenix6.swerve.SwerveModule<>(
                driveConfig.motorType == MotorType.TALONFX ? TalonFX::new : TalonFXS::new,
                turnConfig.motorType == MotorType.TALONFX ? TalonFX::new : TalonFXS::new,
                CANcoder::new,
                constants,
                "",
                0,
                moduleConfig.moduleIndex
        );
        driveTemp = internalModule.getDriveMotor().getDeviceTemp();
        turnTemp = internalModule.getSteerMotor().getDeviceTemp();
        driveCurrent = internalModule.getDriveMotor().getStatorCurrent();
        turnCurrent = internalModule.getSteerMotor().getStatorCurrent();
    }

    @Override
    public double getWheelAngle() {
        return getRawAngle();
    }

    @Override
    public double getRawAngle() {
        return getState().angle.getDegrees();
    }

    @Override
    public SwerveModuleState getState() {
        return internalModule.getCurrentState();
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return internalModule.getTargetState();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return internalModule.getPosition(true);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        internalModule.apply(
                new ModuleRequest()
                        .withState(desiredState)
                        .withDriveRequest(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
                        .withSteerRequest(com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.Position)
                        .withEnableFOC(hasSub)
        );
    }

    @Override
    public void resetDriveEncoder() {
        internalModule.resetPosition();
    }

    @Override
    public double getDriveTempCelsius() {
        return driveTemp.refresh().getValueAsDouble();
    }

    @Override
    public double getTurnTempCelsius() {
        return turnTemp.refresh().getValueAsDouble();
    }

    @Override
    public double getDriveCurrent() {
        return driveCurrent.refresh().getValueAsDouble();
    }

    @Override
    public double getTurnCurrent() {
        return turnCurrent.refresh().getValueAsDouble();
    }

    @Override
    public double getDesiredSpeed() {
        return getDesiredState().speedMetersPerSecond;
    }

    @Override
    public double getDesiredAngleDegrees() {
        return getDesiredState().angle.getDegrees();
    }

    @Override
    public double getVelocity() {
        return getState().speedMetersPerSecond;
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
            double reduction,
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
