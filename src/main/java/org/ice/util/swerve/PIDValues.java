package org.ice.util.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import org.ice.util.motor.GenericMotorController;
import org.ice.util.sendable.AnnotatedSendable;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * Mutable set of PID values. Can be put on SmartDashboard and tuned in real time
 */
public class PIDValues implements AnnotatedSendable {

    private double kP, kI, kD, kFF;

    private GenericMotorController<?> linkedMotor;

    public PIDValues(double kP, double kI, double kD) {
        this(kP,kI,kD,0.0);
    }
    public PIDValues(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
    }
    @Setter(key="P")
    public PIDValues withP(double kP) {
        this.kP = kP;
        updateLinked();
        return this;
    }
    @Setter(key="I")
    public PIDValues withI(double kI) {
        this.kI = kI;
        updateLinked();
        return this;
    }
    @Setter(key="D")
    public PIDValues withD(double kD) {
        this.kD = kD;
        updateLinked();
        return this;
    }
    @Setter(key="FF")
    public PIDValues withFF(double kFF) {
        this.kFF = kFF;
        updateLinked();
        return this;
    }

    public PIDValues withPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        updateLinked();
        return this;
    }

    public PIDValues withPIDF(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
        updateLinked();
        return this;
    }
    @Getter(key="P")
    public double getP() {
        return kP;
    }
    @Getter(key="I")
    public double getI() {
        return kI;
    }
    @Getter(key="D")
    public double getD() {
        return kD;
    }
    @Getter(key="FF")
    public double getFF() {
        return kFF;
    }
    public PIDController asController() {
        if (kFF != 0) DriverStation.reportError("PIDValues with a non-zero kFF value converted to PIDController, which does not support kFF values.", false);
        return new PIDController(kP,kI,kD);
    }

    public void linkToMotor(GenericMotorController<?> motor) {
        linkedMotor = motor;
        updateLinked();
    }

    private void updateLinked() {
        if (linkedMotor != null) linkedMotor.setPID(this);
    }

    public Slot0Configs asSlot0Config() {
        return new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKA(1/kFF);
    }
    public Slot1Configs asSlot1Config() {
        return new Slot1Configs().withKP(kP).withKI(kI).withKD(kD).withKA(1/kFF);
    }
    public Slot2Configs asSlot2Config() {
        return new Slot2Configs().withKP(kP).withKI(kI).withKD(kD).withKA(1/kFF);
    }
    public SlotConfigs asSlotConfig() {
        return new SlotConfigs().withKP(kP).withKI(kI).withKD(kD).withKA(1/kFF);
    }
    public static PIDValues from(double kP, double kI, double kD) {
        return new PIDValues(kP,kI,kD);
    }
    public static PIDValues from(double kP, double kI, double kD, double kFF) {
        return new PIDValues(kP,kI,kD,kFF);
    }
    public static PIDValues from(PIDController pidController) {
        return new PIDValues(pidController.getP(),pidController.getI(),pidController.getD());
    }
}

