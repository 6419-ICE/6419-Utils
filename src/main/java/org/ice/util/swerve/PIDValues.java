package org.ice.util.swerve;

import org.ice.util.sendable.AnnotatedSendable;

public class PIDValues implements AnnotatedSendable {
    //I know, hungarian notation bad, especially when it isn't even true. But like kP kI and kD just make sense.
    private double kP, kI, kD, kFF;
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
        return this;
    }
    @Setter(key="I")
    public PIDValues withI(double kI) {
        this.kI = kI;
        return this;
    }
    @Setter(key="D")
    public PIDValues withD(double kD) {
        this.kD = kD;
        return this;
    }
    @Setter(key="FF")
    public PIDValues withFF(double kFF) {
        this.kFF = kFF;
        return this;
    }
    public void withPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public void withPIDF(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
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
}

